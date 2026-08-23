#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3-Dongle 音频取流工具

从 dongle 的 USB CDC 口把 ESP-NOW 收到的 PCM 拉下来，存成 WAV 或者直接吐到
标准输出给别的程序用。

  链路：INMP441 x2 -> NodeMCU-32S -> ESP-NOW -> S3 Dongle -> USB -> 本脚本

用法：
  python recv_audio.py                        # 自动找口，录到 rec_时间戳.wav
  python recv_audio.py -o test.wav -t 10      # 录 10 秒到 test.wav
  python recv_audio.py --list                 # 只列出串口
  python recv_audio.py -p COM5                # 指定串口
  python recv_audio.py --play                 # 边收边放（需要 sounddevice）
  python recv_audio.py -o - | ffplay -f s16le -ar 16000 -ch_layout stereo -i -

只依赖 pyserial：  pip install pyserial
"""

import argparse
import math
import os
import struct
import sys
import time
import wave

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial，先执行：pip install pyserial")

# ---- 帧格式，必须和固件 main/usb_stream.h 一致 ----
SYNC = b"\xA5\x5A"
HDR_FMT = "<BBBBHHH"          # sync0 sync1 type ch seq len crc
HDR_LEN = struct.calcsize(HDR_FMT)   # 10
FRAME_PCM = 0x01
FRAME_INFO = 0x10
MAX_PAYLOAD = 4096            # 超过就当是误同步，正常包 1280

# dongle 的 USB Serial/JTAG 固定 VID/PID（乐鑫官方分配）
ESP_VID, ESP_PID = 0x303A, 0x1001


def crc16_ccitt(data: bytes) -> int:
    """和固件里那份实现一一对应：多项式 0x1021，初值 0xFFFF，不反射不异或"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def find_port() -> str:
    """优先按 VID/PID 认 dongle。认不出来就把候选列给用户，别瞎猜一个连上去"""
    cands = list(list_ports.comports())
    exact = [p for p in cands if p.vid == ESP_VID and p.pid == ESP_PID]
    if len(exact) == 1:
        return exact[0].device
    if len(exact) > 1:
        sys.exit("插了不止一块 ESP32 设备，用 -p 指定：\n  " +
                 "\n  ".join(f"{p.device}  {p.description}" for p in exact))
    print("没找到 VID:PID = 303A:1001 的设备。当前串口：", file=sys.stderr)
    for p in cands:
        print(f"  {p.device}  {p.description}", file=sys.stderr)
    sys.exit("确认 dongle 已插好；蓝牙那些虚拟串口不是它。也可以用 -p 手动指定。")


def list_all():
    ports = list(list_ports.comports())
    if not ports:
        print("没有任何串口设备")
        return
    for p in ports:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "  -  "
        mark = "  <== ESP32-S3 Dongle" if (p.vid, p.pid) == (ESP_VID, ESP_PID) else ""
        print(f"  {p.device:<8} {vidpid}  {p.description}{mark}")


class Stats:
    def __init__(self):
        self.frames = 0        # 收到的合法 PCM 帧
        self.lost = 0          # 按 seq 推算的丢包（空口 + USB 全算上）
        self.crc_err = 0       # CRC 不过
        self.resync = 0        # 误同步 / 断帧后重新对齐的次数
        self.bytes = 0         # 写出去的 PCM 字节
        self.last_seq = None
        self.t0 = time.time()


def dbfs(pcm: bytes, ch: int, nch: int) -> float:
    """取某一路的 RMS 并换算成 dBFS。安静约 -70，正常说话 -35 上下"""
    n = len(pcm) // 2
    if n <= ch:
        return -120.0
    vals = struct.unpack_from(f"<{n}h", pcm)
    sel = vals[ch::nch]
    if not sel:
        return -120.0
    acc = math.sqrt(sum(float(v) * v for v in sel) / len(sel))
    if acc < 1.0:
        return -120.0
    return 20.0 * math.log10(acc / 32768.0)


def bar(db: float, width: int = 20) -> str:
    """-60dBFS 到 0dBFS 映射成一条条形图，比裸数字容易看出有没有声音"""
    lo, hi = -60.0, 0.0
    frac = max(0.0, min(1.0, (db - lo) / (hi - lo)))
    n = int(frac * width)
    return "#" * n + "-" * (width - n)


def main():
    ap = argparse.ArgumentParser(
        description="从 ESP32-S3-Dongle 取 ESP-NOW 音频流",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="串口，如 COM5。不给就自动找")
    ap.add_argument("-o", "--out", default=None,
                    help="输出 WAV 路径；写 - 表示把裸 PCM 吐到标准输出")
    ap.add_argument("-t", "--seconds", type=float, default=0,
                    help="录多少秒后自动停，0 = 一直录到 Ctrl+C")
    ap.add_argument("--play", action="store_true",
                    help="边收边放（需要 pip install sounddevice）")
    ap.add_argument("--no-fill", action="store_true",
                    help="丢包时不补静音。默认补，这样 WAV 时长和真实时间对得上")
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    args = ap.parse_args()

    if args.list:
        list_all()
        return

    port = args.port or find_port()

    to_stdout = (args.out == "-")
    if args.out is None:
        args.out = time.strftime("rec_%Y%m%d_%H%M%S.wav")

    log = sys.stderr if to_stdout else sys.stdout

    print(f"打开 {port} ...", file=log)
    # USB CDC 不看波特率，随便填；timeout 决定 read 最多阻塞多久
    ser = serial.Serial(port, 921600, timeout=0.2)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 告诉固件开始推流。固件收到 S 之后会闭掉日志，链路上就只剩纯二进制帧
    ser.write(b"S")
    ser.flush()

    st = Stats()
    rate, nch, bits, frames_per_pkt = 16000, 2, 16, 320
    got_info = False

    wav = None
    raw = None
    stream = None

    def open_sink():
        nonlocal wav, raw
        if to_stdout:
            raw = sys.stdout.buffer
        else:
            wav = wave.open(args.out, "wb")
            wav.setnchannels(nch)
            wav.setsampwidth(bits // 8)
            wav.setframerate(rate)

    def write_pcm(data: bytes):
        st.bytes += len(data)
        if raw is not None:
            raw.write(data)
        elif wav is not None:
            wav.writeframes(data)
        if stream is not None:
            try:
                import numpy as np
                stream.write(np.frombuffer(data, dtype="<i2").reshape(-1, nch))
            except Exception:
                pass

    buf = bytearray()
    last_print = 0.0
    db0 = db1 = -120.0

    try:
        while True:
            chunk = ser.read(8192)
            if chunk:
                buf += chunk

            # ---- 解帧。可以从任意字节开始对齐：先找同步字，再用长度和 CRC 复核 ----
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    # 同步字可能被切在两次 read 中间，留最后一个字节
                    if len(buf) > 1:
                        txt = bytes(buf[:-1])
                        if txt.strip():
                            print(txt.decode("utf-8", "replace"), end="", file=log)
                        del buf[:-1]
                    break

                if i > 0:
                    # 同步字之前的是固件日志之类的文本，原样转出来
                    txt = bytes(buf[:i])
                    if txt.strip():
                        print(txt.decode("utf-8", "replace"), end="", file=log)
                    del buf[:i]

                if len(buf) < HDR_LEN:
                    break

                _, _, ftype, ch, seq, length, crc = struct.unpack_from(HDR_FMT, buf)

                if length > MAX_PAYLOAD:
                    # 长度离谱，说明这两个字节只是碰巧长得像同步字
                    st.resync += 1
                    del buf[:2]
                    continue

                if len(buf) < HDR_LEN + length:
                    break        # 数据还没到齐，等下一次 read

                payload = bytes(buf[HDR_LEN:HDR_LEN + length])
                if crc16_ccitt(payload) != crc:
                    st.crc_err += 1
                    st.resync += 1
                    del buf[:2]  # 只跳过这个假同步字，真帧头可能就在后面
                    continue

                del buf[:HDR_LEN + length]

                if ftype == FRAME_INFO and length >= 8:
                    rate, nch, bits, frames_per_pkt = struct.unpack("<IBBH", payload[:8])
                    got_info = True
                    print(f"固件报告格式：{rate} Hz / {nch} 声道 / {bits} bit / "
                          f"每包 {frames_per_pkt} 帧", file=log)
                    open_sink()
                    if args.play:
                        try:
                            import sounddevice as sd
                            stream = sd.OutputStream(samplerate=rate, channels=nch,
                                                     dtype="int16")
                            stream.start()
                            print("已开启实时播放", file=log)
                        except Exception as e:
                            print(f"播放不可用（{e}），继续只录文件", file=log)
                    continue

                if ftype != FRAME_PCM:
                    continue

                if not got_info:
                    # 没收到 INFO（比如脚本是中途接上的），按默认格式先开起来
                    got_info = True
                    open_sink()

                # ---- 丢包：seq 是空口音频包号，差几个就是中间掉了几包 ----
                if st.last_seq is not None:
                    d = (seq - st.last_seq) & 0xFFFF
                    if d == 0:
                        continue                 # 重复包
                    if 1 < d < 1000:
                        st.lost += d - 1
                        if not args.no_fill:
                            # 补静音，保证 WAV 的时间轴和真实时间一致。
                            # 不补的话录出来的音频会整体变短、口型对不上
                            write_pcm(b"\x00" * (len(payload) * (d - 1)))
                st.last_seq = seq

                st.frames += 1
                write_pcm(payload)

                if nch >= 2:
                    db0, db1 = dbfs(payload, 0, nch), dbfs(payload, 1, nch)
                else:
                    db0 = db1 = dbfs(payload, 0, 1)

            # ---- 状态行 ----
            now = time.time()
            if now - last_print > 0.25:
                last_print = now
                el = now - st.t0
                secs = st.bytes / (rate * nch * bits / 8) if rate else 0
                total = st.frames + st.lost
                good = 100.0 * st.frames / total if total else 0.0
                print(f"\r 录 {secs:6.1f}s | 包 {st.frames:6d} 丢 {st.lost:4d} "
                      f"({good:5.1f}%) CRC错 {st.crc_err:3d} 重同步 {st.resync:3d} | "
                      f"L[{bar(db0)}]{db0:6.1f}  R[{bar(db1)}]{db1:6.1f} dBFS",
                      end="", file=log, flush=True)

            if args.seconds and (time.time() - st.t0) >= args.seconds:
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("", file=log)
        try:
            ser.write(b"X")     # 让固件停流、把日志放回来
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        if stream is not None:
            stream.stop(); stream.close()
        if wav is not None:
            wav.close()
            dur = st.bytes / (rate * nch * bits / 8)
            print(f"已保存 {args.out}  {dur:.1f} 秒  "
                  f"{os.path.getsize(args.out)/1024:.0f} KB", file=log)
        total = st.frames + st.lost
        if total:
            print(f"统计：收 {st.frames} 包，丢 {st.lost} 包 "
                  f"({100.0*st.lost/total:.2f}%)，CRC 错 {st.crc_err}，"
                  f"重同步 {st.resync}", file=log)


if __name__ == "__main__":
    main()
