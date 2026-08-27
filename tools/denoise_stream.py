# -*- coding: utf-8 -*-
"""
ESP-NOW Wireless Mic 实时双麦降噪 / 人声增强（PC 端，边收边处理）
================================================================

链路：INMP441 x2 --I2S--> NodeMCU-32S --ESP-NOW--> S3 Dongle --USB UAC1.0--> 本机
音频：16 kHz / 双声道 int16 / 两麦间距 5 cm（ch0=左槽，ch1=右槽）

和离线版的区别：全程因果、不看未来、不做全局统计。
  * 噪声空间协方差 Rnn(f) 离线从「纯噪声」录音算好（profile 子命令），运行时只读；
  * 噪声电平用 Doblinger 连续最小值跟踪在线校正，应对增益漂移 / 环境变化；
  * 导向矢量 d(f) 边跑边更新（默认从正前方 [1,1] 起步，攒够语音帧后每秒重算一次）；
  * 后置 MMSE-LSA 用 decision-directed 先验 SNR，只依赖上一帧；
  * 输出电平用慢速 AGC + 软限幅，不做全局峰值归一。

算法时延 = 窗长 - 帧移 = 512-128 = 384 采样 = 24 ms，再加一个采集块（10~20 ms）。

用法：
    python denoise_stream.py --list                       # 列出输入设备
    python denoise_stream.py profile -o noise.npz -s 10   # 对着噪声环境录 10 s 建噪声画像
    python denoise_stream.py profile -o noise.npz -i 噪声.wav      # 或从已有 wav 建
    python denoise_stream.py run -p noise.npz -o out.wav  # 实时降噪，Ctrl+C 停
    python denoise_stream.py file -p noise.npz -i in.wav -o out.wav   # 用同一引擎离线验证
"""
from __future__ import print_function
import os
import sys
import time
import wave
import queue
import argparse
import threading
import numpy as np

DEV_NAME = 'ESP-NOW'            # USB UAC 设备名（README: ESP-NOW Wireless Mic）
SR = 16000
CH = 2
N_FFT = 512                     # 32 ms —— 16k 下的实测最优（256 频率分辨率不够，1024 会糊音节）
HOP = 128                       # 8 ms
HOSTAPI_PREF = ['WASAPI', 'WDM-KS', 'DirectSound', 'MME']


# --------------------------------------------------------------------------
# WAV 读写（16/24/32-bit PCM）
# --------------------------------------------------------------------------
def wav_read(path):
    w = wave.open(path, 'rb')
    ch, sw, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
    raw = w.readframes(n)
    w.close()
    if sw == 2:
        x = np.frombuffer(raw, dtype='<i2').astype(np.float32) / 32768.0
    elif sw == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        v = np.where(v & 0x800000, v - (1 << 24), v)
        x = v.astype(np.float32) / 8388608.0
    elif sw == 4:
        x = np.frombuffer(raw, dtype='<i4').astype(np.float32) / 2147483648.0
    else:
        raise ValueError('unsupported sample width: %d' % sw)
    return x.reshape(-1, ch), sr


class WavWriter(object):
    """边收边写：先占位 WAV 头，关闭时回填长度。中途 Ctrl+C 也是完整可用的文件。"""

    def __init__(self, path, sr, ch):
        self.w = wave.open(path, 'wb')
        self.w.setnchannels(ch)
        self.w.setsampwidth(2)
        self.w.setframerate(sr)
        self.n = 0

    def write(self, x):
        x = np.clip(x, -1.0, 1.0 - 1e-7)
        self.w.writeframes((x * 32767.0).astype('<i2').tobytes())
        self.n += len(x)

    def close(self):
        try:
            self.w.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# DSP 基础件
# --------------------------------------------------------------------------
def _win(n):
    return np.sqrt(np.hanning(n + 1)[:n]).astype(np.float32)


def expint1(x):
    """指数积分 E1(x)，A&S 5.1.53 / 5.1.56 近似（不依赖 scipy）"""
    x = np.maximum(np.asarray(x, np.float64), 1e-8)
    out = np.empty_like(x)
    m = x <= 1.0
    xs = x[m]
    out[m] = (-np.log(xs) - 0.57721566 + 0.99999193 * xs - 0.24991055 * xs ** 2
              + 0.05519968 * xs ** 3 - 0.00976004 * xs ** 4 + 0.00107857 * xs ** 5)
    xl = x[~m]
    num = xl ** 4 + 8.5733287401 * xl ** 3 + 18.059016973 * xl ** 2 + 8.6347608925 * xl + 0.2677737343
    den = xl ** 4 + 9.5733223454 * xl ** 3 + 25.6329561486 * xl ** 2 + 21.0996530827 * xl + 3.9584969228
    out[~m] = num / den * np.exp(-xl) / xl
    return out.astype(np.float32)


def smooth_freq(v, k=3):
    """频率轴滑动平均（1 维，逐帧调用）"""
    if k <= 1:
        return v
    pad = k // 2
    vp = np.pad(v, (pad, pad), mode='edge')
    acc = np.zeros_like(v)
    for i in range(k):
        acc += vp[i:i + len(v)]
    return acc / k


def smooth_freq2(M, k=5):
    """频率轴滑动平均（(F,C) 复数矩阵）"""
    if k <= 1:
        return M
    pad = k // 2
    Mp = np.pad(M, ((pad, pad), (0, 0)), mode='edge')
    acc = np.zeros_like(M)
    for i in range(k):
        acc += Mp[i:i + M.shape[0]]
    return acc / k


# --------------------------------------------------------------------------
# 噪声最小值跟踪（Doblinger）——画像标定和运行时必须用同一套常数
# --------------------------------------------------------------------------
MT_A, MT_B, MT_G = 0.7, 0.96, 0.998


def min_track_step(P_sm, P_min, Py):
    prev = P_sm
    P_sm = MT_A * prev + (1 - MT_A) * Py
    up = P_min < P_sm
    P_min = np.where(up, MT_G * P_min + (1 - MT_G) / (1 - MT_B) * (P_sm - MT_B * prev), P_sm)
    return P_sm, P_min


def min_track_bias(N, Pn_ref, band):
    """最小值跟踪器天然低估：它跟的是平滑周期图的**最小值**，而画像里存的是**均值**，
    两者差 4~6 dB。直接相比会让运行时的电平校正系数偏小、噪声减不干净。
    这里在纯噪声上把同一个跟踪器跑一遍，量出真实偏差存进画像，运行时除掉即可。"""
    P = (np.abs(N[:, :, 0]) ** 2).astype(np.float32)
    P_sm = P[0].copy()
    P_min = P[0].copy()
    r = []
    for t in range(len(P)):
        P_sm, P_min = min_track_step(P_sm, P_min, P[t])
        if t > len(P) // 4:                       # 跳过收敛段
            r.append(float(np.median(P_min[band] / np.maximum(Pn_ref[band], 1e-20))))
    return float(np.median(r)) if r else 1.0


# --------------------------------------------------------------------------
# 噪声画像（离线算一次，运行时只读）
# --------------------------------------------------------------------------
def build_profile(noise, sr, n_fft=N_FFT, hop=HOP, skip_s=0.5):
    """从纯噪声录音估计空间协方差 Rnn(f)。
    截尾均值：按帧能量丢掉最低 5% / 最高 15% —— 摆放麦克风的磕碰瞬态会把均值抬高好几 dB，
    进而导致运行时过减、把人声一起吃掉。"""
    C = noise.shape[1]
    s = int(skip_s * sr)
    nz = noise[s:] if len(noise) > s * 4 else noise
    w = _win(n_fft)
    idx = np.arange(0, len(nz) - n_fft, hop)
    if len(idx) < 32:
        raise ValueError('噪声太短：至少需要 %.1f 秒' % (32.0 * hop / sr))
    fr = np.stack([nz[i:i + n_fft] for i in idx])            # (T,N,C)
    N = np.fft.rfft(fr * w[None, :, None], axis=1)           # (T,F,C)
    e = (np.abs(N[:, :, 0]) ** 2).sum(1)
    m = (e > np.percentile(e, 5)) & (e < np.percentile(e, 85))
    Ns = N[m]
    R = np.einsum('tfi,tfj->fij', Ns, np.conj(Ns)) / len(Ns)
    freqs = np.fft.rfftfreq(n_fft, 1.0 / sr)
    lowband = (freqs > 120) & (freqs < 3500)
    bias = min_track_bias(N, np.maximum(np.real(R[:, 0, 0]).astype(np.float32), 1e-20), lowband)
    return {'Rnn': R.astype(np.complex64), 'sr': sr, 'n_fft': n_fft, 'hop': hop,
            'ch': C, 'frames': len(Ns), 'keep': float(m.mean()),
            'minbias': np.float32(max(bias, 1e-6))}


def save_profile(path, prof):
    np.savez(path, **prof)


def load_profile(path, sr, n_fft, hop, ch):
    z = np.load(path)
    if int(z['sr']) != sr or int(z['n_fft']) != n_fft or int(z['hop']) != hop:
        raise SystemExit('噪声画像参数不匹配：画像 %d Hz/nfft %d/hop %d，当前 %d Hz/nfft %d/hop %d'
                         % (int(z['sr']), int(z['n_fft']), int(z['hop']), sr, n_fft, hop))
    R = z['Rnn']
    if R.shape[1] < ch:
        raise SystemExit('噪声画像只有 %d 声道，当前需要 %d 声道' % (R.shape[1], ch))
    bias = float(z['minbias']) if 'minbias' in z.files else 1.0
    return R[:, :ch, :ch].astype(np.complex64), bias


# --------------------------------------------------------------------------
# 流式引擎
# --------------------------------------------------------------------------
class StreamDenoiser(object):
    def __init__(self, R_nn, minbias=1.0, sr=SR, ch=CH, n_fft=N_FFT, hop=HOP,
                 floor_db=-18.0, over_sub=1.4, alpha_dd=0.98,
                 hp_hz=90.0, agc_db=-20.0, agc=True,
                 beam=True, adaptive_steer=False, steer_update_s=1.0, verbose=False):
        self.sr, self.ch, self.n_fft, self.hop = sr, ch, n_fft, hop
        self.F = n_fft // 2 + 1
        self.win = _win(n_fft)
        self.freqs = np.fft.rfftfreq(n_fft, 1.0 / sr).astype(np.float32)
        self.floor = np.float32(10 ** (floor_db / 20.0))
        self.over_sub = np.float32(over_sub)
        self.alpha_dd = np.float32(alpha_dd)
        self.beam = beam and ch >= 2
        self.verbose = verbose

        # WOLA 归一化常数：sum_k w^2(n - k*hop)。sqrt-hann + hop=N/4 满足 COLA，
        # 稳态区各点该和是同一个常数，直接量出来用（换 hop 也不会算错）。
        ww = (self.win ** 2).astype(np.float64)
        L = n_fft * 4
        acc = np.zeros(L)
        for k in range(0, (L - n_fft) // hop + 1):
            acc[k * hop:k * hop + n_fft] += ww
        mid = acc[n_fft:L - n_fft]
        self.wola_norm = float(np.median(mid))
        if mid.max() - mid.min() > 1e-6 * self.wola_norm:
            raise ValueError('窗/帧移不满足 COLA：n_fft=%d hop=%d' % (n_fft, hop))

        # ---- 缓冲 ----
        self.abuf = np.zeros((n_fft, ch), np.float32)         # 分析滑窗
        self.obuf = np.zeros(n_fft, np.float32)               # 合成 overlap-add
        self.pend = np.zeros((0, ch), np.float32)             # 不足一个 hop 的余数

        # ---- 噪声统计 ----
        self.R_nn0 = R_nn.copy()
        self.Pn_prof = np.maximum(np.real(R_nn[:, 0, 0]).astype(np.float32), 1e-20)
        self.scale = np.float32(1.0)                          # 在线电平校正
        self.minbias = np.float32(max(minbias, 1e-6))         # 跟踪器低估量，建画像时标定
        # Doblinger 连续最小值跟踪
        self.P_sm = self.Pn_prof.copy()
        self.P_min = self.Pn_prof.copy()

        # ---- 波束 ----
        self.d = np.ones((self.F, ch), np.complex64)          # 起步：正前方（两麦等距）
        self.Rxx = np.zeros((self.F, ch, ch), np.complex64)
        self.n_spch = 0
        # 默认固定 d=[1,1]：两麦等距、说话人在正前方时这就是正确的导向矢量，
        # 而且不会被误判成语音的噪声突发带跑。实测在线自适应（每次只攒得到几十帧）
        # 相比固定值是平局甚至略差，所以做成可选。
        self.adaptive = bool(adaptive_steer) and self.beam
        self.steer_every = max(int(steer_update_s * sr / hop), 1) if self.adaptive else 10 ** 9
        self.w = None
        self.n_resteer = 0
        self._update_beam()
        self.Pn_ref = self.Pn_ref0.copy()

        # ---- 后置滤波状态 ----
        self.G_prev = np.ones(self.F, np.float32)
        self.gamma_prev = np.ones(self.F, np.float32)
        self.G_out = np.ones(self.F, np.float32)

        # ---- 频段整形 ----
        self.shape = np.clip((self.freqs - hp_hz * 0.5) / max(hp_hz * 0.5, 1e-6), 0.0, 1.0) ** 2
        self.shape = self.shape.astype(np.float32)

        # ---- AGC ----
        self.agc_on = agc
        self.agc_target = 10 ** (agc_db / 20.0)
        self.agc_gain = 1.0
        self.rms_slow = 0.0

        # ---- 统计 ----
        self.band = (self.freqs > 250) & (self.freqs < 3800)
        self.lowband = (self.freqs > 120) & (self.freqs < 3500)
        self.n_frames = 0
        self.n_speech = 0
        self.snr_db = 0.0
        self.hangover = 0
        self.dsp_time = 0.0

    # -------------------------------------------------------------- 噪声跟踪
    def _track_noise(self, Py):
        """Doblinger 连续最小值跟踪：不需要缓存历史帧，适合流式。
        只用来校正整体电平，噪声的频谱形状和空间结构仍来自离线画像。"""
        self.P_sm, self.P_min = min_track_step(self.P_sm, self.P_min, Py)
        # 除掉建画像时标定出的低估量，才能和画像里的均值口径对齐
        m = self.lowband
        r = float(np.median(self.P_min[m] / self.Pn_prof[m])) / float(self.minbias)
        r = min(max(r, 0.25), 4.0)
        # 每帧只挪一点点：噪声电平变化远慢于语音
        self.scale = np.float32(0.999 * self.scale + 0.001 * r)

    # -------------------------------------------------------------- 波束权重
    def _update_beam(self):
        """算 MVDR 权重和「scale=1 时」的噪声功率基准。
        w = R^-1 d / (d^H R^-1 d) 对 R 的整体缩放是不变的，所以电平校正只需每帧
        缩放 Pn 基准，不必重算权重。"""
        R = self.R_nn0
        self.Pn_ref0 = np.maximum(np.real(R[:, 0, 0]).astype(np.float32), 1e-20)
        if not self.beam:
            self.w = None
            self.Pn_bf0 = self.Pn_ref0
            return
        C = self.ch
        eye = np.eye(C, dtype=np.complex64)[None, :, :]
        load = (np.trace(R, axis1=1, axis2=2).real / C)[:, None, None] * 1e-2 + 1e-20
        Rinv = np.linalg.inv(R + load * eye)
        num = np.einsum('fij,fj->fi', Rinv, self.d)
        den = np.real(np.einsum('fi,fi->f', np.conj(self.d), num)) + 1e-12
        self.w = (num / den[:, None]).astype(np.complex64)
        self.Pn_bf0 = np.maximum(
            np.real(np.einsum('fi,fij,fj->f', np.conj(self.w), R, self.w)).astype(np.float32),
            self.Pn_ref0 * 1e-3)

    def _resteer(self):
        """用攒下来的语音帧 Rxx 重估导向矢量（协方差白化法）。"""
        R = self.R_nn0
        self.n_resteer += 1
        Rxx = self.Rxx / max(self.n_spch, 1)
        C = self.ch
        d = np.zeros((self.F, C), np.complex64)
        for f in range(self.F):
            Rl = R[f] + np.eye(C) * (np.trace(R[f]).real / C) * 1e-3
            try:
                L = np.linalg.cholesky(Rl)
                Li = np.linalg.inv(L)
                M = Li.dot(Rxx[f]).dot(Li.conj().T)
                M = 0.5 * (M + M.conj().T)
                _, vec = np.linalg.eigh(M)
                d[f] = L.dot(vec[:, -1])
            except np.linalg.LinAlgError:
                d[f, 0] = 1.0
        d = smooth_freq2(d, 5)
        ph = d[:, 0:1]
        d = d * (np.conj(ph) / (np.abs(ph) + 1e-12))
        d = d / (np.abs(d[:, 0:1]) + 1e-12)
        # 和旧值混合，避免一次跳变；同时给 Rxx 加遗忘
        self.d = (0.5 * self.d + 0.5 * d.astype(np.complex64)).astype(np.complex64)
        self.Rxx *= 0.5
        self.n_spch = int(self.n_spch * 0.5)
        self._update_beam()

    # -------------------------------------------------------------- 单帧处理
    def _frame(self):
        X = np.fft.rfft(self.abuf * self.win[:, None], axis=0)     # (F,C)

        Py_ref = (np.abs(X[:, 0]) ** 2).astype(np.float32)
        self._track_noise(Py_ref)
        # 在线电平校正每帧生效（只是缩放，不必重算波束权重）
        self.Pn_ref = self.Pn_ref0 * self.scale

        # 空间滤波
        if self.w is not None:
            Y = np.einsum('fc,fc->f', X, np.conj(self.w))
            Pn = self.Pn_bf0 * self.scale
        else:
            Y = X[:, 0]
            Pn = self.Pn_ref

        # 在线 VAD（带 hangover）：语音带后验 SNR
        b = self.band
        snr = 10 * np.log10(max(float(Py_ref[b].mean()), 1e-20) /
                            max(float(self.Pn_ref[b].mean()), 1e-20))
        self.snr_db = 0.9 * self.snr_db + 0.1 * snr
        if snr > 6.0:
            self.hangover = 12                                      # ~100 ms 挂起
        elif self.hangover > 0:
            self.hangover -= 1
        speech = self.hangover > 0

        if speech:
            self.n_speech += 1
        # 只用高置信语音帧攒 Rxx
        if self.adaptive and snr > 8.0:
            self.Rxx += np.einsum('fi,fj->fij', X, np.conj(X))
            self.n_spch += 1
        # 周期性重估导向矢量。这个检查必须独立于上面的 SNR 分支 ——
        # 塞进去的话只有「恰好是高 SNR 帧且帧号整除周期」才会触发，等于没有自适应。
        if self.adaptive and self.n_spch >= 32 and self.n_frames % self.steer_every == 0:
            self._resteer()

        # 后置 MMSE-LSA（decision-directed）
        Py = smooth_freq((np.abs(Y) ** 2).astype(np.float32), 3)
        gamma = np.minimum(Py / np.maximum(Pn * self.over_sub, 1e-20), 1e4).astype(np.float32)
        xi = self.alpha_dd * (self.G_prev ** 2) * self.gamma_prev + \
            (1 - self.alpha_dd) * np.maximum(gamma - 1.0, 0.0)
        xi = np.maximum(xi, 1e-3)
        v = np.minimum(xi / (1.0 + xi) * gamma, 500.0)
        G = xi / (1.0 + xi) * np.exp(0.5 * expint1(v))
        G = np.clip(G, self.floor, 1.0).astype(np.float32)
        self.G_prev = G
        self.gamma_prev = gamma

        G = smooth_freq(G, 3)
        # 只平滑下降沿：上升沿（起音）原样通过。流式没有前瞻，起音全靠这一条不被压掉。
        G = np.where(G < self.G_out, 0.6 * G + 0.4 * self.G_out, G).astype(np.float32)
        self.G_out = G

        Yf = (Y * G * self.shape).astype(np.complex64)
        y = np.fft.irfft(Yf, n=self.n_fft).astype(np.float32) * self.win / self.wola_norm

        self.obuf += y
        out = self.obuf[:self.hop].copy()
        self.obuf = np.concatenate([self.obuf[self.hop:], np.zeros(self.hop, np.float32)])

        self.n_frames += 1
        return out, speech

    # -------------------------------------------------------------- AGC
    def _agc(self, y, speech):
        if not self.agc_on:
            return y
        r = float(np.sqrt(np.mean(y ** 2)) + 1e-12)
        if speech:
            self.rms_slow = 0.98 * self.rms_slow + 0.02 * r if self.rms_slow else r
        if self.rms_slow > 1e-6:
            want = self.agc_target / self.rms_slow
            want = min(max(want, 0.25), 32.0)
            # 慢速逼近，避免抽气感
            self.agc_gain += (want - self.agc_gain) * (0.02 if want < self.agc_gain else 0.005)
        y = y * self.agc_gain
        a = np.abs(y)                                              # 软限幅
        t, ceil = 0.7, 0.97
        over = a > t
        if over.any():
            y[over] = np.sign(y[over]) * (t + (ceil - t) * np.tanh((a[over] - t) / (ceil - t)))
        return y

    # -------------------------------------------------------------- 对外接口
    def process(self, block):
        """block: (n, ch) float32 -> (m,) float32 增强后的单声道。
        m 是 hop 的整数倍，不一定等于 n（余数留到下一块）。"""
        t0 = time.time()
        x = np.concatenate([self.pend, block.astype(np.float32)]) if len(self.pend) else block.astype(np.float32)
        outs = []
        spk = False
        i = 0
        while len(x) - i >= self.hop:
            self.abuf = np.concatenate([self.abuf[self.hop:], x[i:i + self.hop]])
            o, s = self._frame()
            outs.append(o)
            spk = spk or s
            i += self.hop
        self.pend = x[i:]
        y = np.concatenate(outs) if outs else np.zeros(0, np.float32)
        if len(y):
            y = self._agc(y, spk)
        self.dsp_time += time.time() - t0
        return y

    @property
    def latency_ms(self):
        return 1000.0 * (self.n_fft - self.hop) / self.sr


# --------------------------------------------------------------------------
# 设备选择
# --------------------------------------------------------------------------
def pick_device(sd, name_sub):
    devs = sd.query_devices()
    cand = []
    for i, d in enumerate(devs):
        if d['max_input_channels'] >= 1 and name_sub.lower() in d['name'].lower():
            api = sd.query_hostapis(d['hostapi'])['name']
            rank = next((k for k, p in enumerate(HOSTAPI_PREF) if p.lower() in api.lower()),
                        len(HOSTAPI_PREF))
            cand.append((rank, i, d, api))
    if not cand:
        return None
    cand.sort()
    return cand[0][1], cand[0][2], cand[0][3]


def list_devices(sd):
    print('%-4s %-45s %-16s %s' % ('id', '设备名', 'HostAPI', '输入声道'))
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] > 0:
            print('%-4d %-45s %-16s %d' % (i, d['name'][:45],
                                           sd.query_hostapis(d['hostapi'])['name'],
                                           d['max_input_channels']))


# --------------------------------------------------------------------------
# 子命令
# --------------------------------------------------------------------------
def cmd_profile(a):
    if a.input:
        x, sr = wav_read(a.input)
        print('从文件建画像: %s  %d ch  %.1f s  @%d Hz' % (a.input, x.shape[1], len(x) / float(sr), sr))
    else:
        import sounddevice as sd
        got = pick_device(sd, a.device)
        if not got:
            raise SystemExit('找不到名字含 "%s" 的输入设备。先跑 --list 看看。' % a.device)
        idx, dev, api = got
        print('设备: [%d] %s (%s)' % (idx, dev['name'], api))
        print('保持环境噪声、不要说话，录 %d 秒 ...' % a.seconds)
        x = sd.rec(int(a.seconds * SR), samplerate=SR, channels=CH, dtype='int16', device=idx)
        sd.wait()
        x = x.astype(np.float32) / 32768.0
        sr = SR
    if sr != SR:
        raise SystemExit('采样率必须是 %d Hz，当前 %d Hz' % (SR, sr))
    prof = build_profile(x, sr)
    save_profile(a.output, prof)
    R = prof['Rnn']
    f = np.fft.rfftfreq(N_FFT, 1.0 / sr)
    print('画像已存: %s  (%d 帧，保留 %.0f%%，最小值跟踪偏差 %.1f dB)'
          % (a.output, prof['frames'], 100 * prof['keep'], 10 * np.log10(float(prof['minbias']))))
    print('噪声谱 (ch0):')
    for lo, hi in [(80, 250), (250, 600), (600, 1200), (1200, 2400), (2400, 4000), (4000, 8000)]:
        m = (f >= lo) & (f < hi)
        p = float(np.real(R[m, 0, 0]).mean())
        coh = float((np.abs(R[m, 0, 1]) ** 2 /
                     (np.real(R[m, 0, 0]) * np.real(R[m, 1, 1]) + 1e-20)).mean()) if R.shape[1] > 1 else 0
        print('  %5d-%5d Hz  %6.1f dB   双麦相干度 %.2f' % (lo, hi, 10 * np.log10(p + 1e-20), coh))


def cmd_run(a):
    import sounddevice as sd
    got = pick_device(sd, a.device)
    if not got:
        raise SystemExit('找不到名字含 "%s" 的输入设备（dongle 插上了吗？）。先跑 --list 看看。' % a.device)
    idx, dev, api = got
    R_nn, minbias = load_profile(a.profile, SR, N_FFT, HOP, CH)
    eng = StreamDenoiser(R_nn, minbias, floor_db=a.floor_db, over_sub=a.over_sub,
                         beam=not a.no_beam, adaptive_steer=a.adaptive_steer,
                         agc=not a.no_agc, agc_db=a.agc_db)

    print('设备   : [%d] %s (%s)' % (idx, dev['name'], api))
    print('格式   : %d Hz / %d ch / int16，块 %d 帧 (%.0f ms)' % (SR, CH, a.block, 1000.0 * a.block / SR))
    print('算法时延: %.0f ms（窗 %.0f ms - 帧移 %.0f ms）+ 采集块'
          % (eng.latency_ms, 1000.0 * N_FFT / SR, 1000.0 * HOP / SR))
    print('输出   : %s' % (a.output or '(不落盘)'))
    print('Ctrl+C 停止\n')

    q = queue.Queue(maxsize=64)
    stats = {'xrun': 0, 'drop': 0}

    def cb(indata, frames, tinfo, status):
        if status:
            stats['xrun'] += 1
        try:
            q.put_nowait(indata.copy())
        except queue.Full:
            stats['drop'] += 1

    wr = WavWriter(a.output, SR, 2 if a.ab else 1) if a.output else None
    raw_q = np.zeros(0, np.float32)          # A/B 用：原始 ch0 的延迟对齐缓冲
    stop = threading.Event()
    t_start = time.time()
    n_out = 0
    try:
        with sd.InputStream(device=idx, samplerate=SR, channels=CH, dtype='int16',
                            blocksize=a.block, callback=cb):
            last = time.time()
            while not stop.is_set():
                try:
                    blk = q.get(timeout=1.0)
                except queue.Empty:
                    print('  [!] 1 秒没收到音频 —— 检查 dongle / 蓝灯状态')
                    continue
                x = blk.astype(np.float32) / 32768.0
                y = eng.process(x)
                if wr and a.ab:
                    raw_q = np.concatenate([raw_q, x[:, 0]])
                    m = min(len(y), len(raw_q))
                    if m:
                        wr.write(np.stack([raw_q[:m], y[:m]], 1))
                        raw_q = raw_q[m:]
                elif wr and len(y):
                    wr.write(y)
                n_out += len(y)
                now = time.time()
                if now - last > 1.0:
                    el = now - t_start
                    rtf = eng.dsp_time / max(el, 1e-9)
                    lvl = 20 * np.log10(max(float(np.sqrt(np.mean(y ** 2))) if len(y) else 1e-9, 1e-9))
                    print('\r  %5.1fs  SNR %5.1f dB  输出 %6.1f dBFS  AGC %4.1fx  '
                          'CPU %4.1f%%  语音帧 %3.0f%%  xrun %d 丢块 %d'
                          % (el, eng.snr_db, lvl, eng.agc_gain, 100 * rtf,
                             100.0 * eng.n_speech / max(eng.n_frames, 1),
                             stats['xrun'], stats['drop']), end='')
                    sys.stdout.flush()
                    last = now
    except KeyboardInterrupt:
        pass
    finally:
        if wr:
            wr.close()
        el = time.time() - t_start
        print('\n\n收 %.1f s，出 %.1f s 音频，CPU 占用 %.1f%%，xrun %d，丢块 %d'
              % (el, n_out / float(SR), 100 * eng.dsp_time / max(el, 1e-9),
                 stats['xrun'], stats['drop']))
        if a.output:
            print('已写入 %s' % a.output)


def cmd_file(a):
    """用完全相同的流式引擎跑一个 wav —— 验证/回归用，结果应与实时一致。"""
    x, sr = wav_read(a.input)
    if sr != SR:
        raise SystemExit('采样率必须是 %d Hz，当前 %d Hz' % (SR, sr))
    R_nn, minbias = load_profile(a.profile, SR, N_FFT, HOP, x.shape[1])
    eng = StreamDenoiser(R_nn, minbias, ch=x.shape[1], floor_db=a.floor_db,
                         over_sub=a.over_sub, beam=not a.no_beam,
                         adaptive_steer=a.adaptive_steer, agc=not a.no_agc, agc_db=a.agc_db)
    blk = a.block
    outs = []
    t0 = time.time()
    for i in range(0, len(x), blk):                              # 按块喂，模拟实时
        outs.append(eng.process(x[i:i + blk]))
    y = np.concatenate(outs)
    el = time.time() - t0
    print('输入 %.1f s，输出 %.1f s' % (len(x) / float(sr), len(y) / float(sr)))
    print('算法时延 %.0f ms，处理耗时 %.1f s (实时率 %.2fx，CPU 占用 %.1f%%)'
          % (eng.latency_ms, el, len(x) / float(sr) / max(el, 1e-9), 100.0 * el * sr / len(x)))
    print('语音帧占比 %.0f%%，噪声电平校正 %.2fx' % (
        100.0 * eng.n_speech / max(eng.n_frames, 1), eng.scale))
    if a.ab:
        n = min(len(x), len(y))
        wr = WavWriter(a.output, SR, 2)
        wr.write(np.stack([x[:n, 0], y[:n]], 1))
        wr.close()
    else:
        wr = WavWriter(a.output, SR, 1)
        wr.write(y)
        wr.close()
    print('-> %s' % a.output)


def main():
    ap = argparse.ArgumentParser(
        description='ESP-NOW Wireless Mic 实时双麦降噪（边收边处理）',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('--list', action='store_true', help='列出输入设备后退出')
    sub = ap.add_subparsers(dest='cmd')

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--floor-db', type=float, default=-18.0, help='最大衰减量（越负越狠）')
    common.add_argument('--over-sub', type=float, default=1.4, help='噪声过减因子')
    common.add_argument('--no-beam', action='store_true', help='关掉 MVDR，只做单通道降噪')
    common.add_argument('--adaptive-steer', action='store_true',
                        help='在线重估导向矢量（默认固定正前方；说话人明显偏轴时可试）')
    common.add_argument('--no-agc', action='store_true', help='关掉自动增益')
    common.add_argument('--agc-db', type=float, default=-20.0, help='AGC 目标电平 dBFS')
    common.add_argument('--block', type=int, default=160, help='采集块大小（帧），160=10ms')
    common.add_argument('--ab', action='store_true', help='输出左=原始 右=增强 的双声道对比')

    p = sub.add_parser('profile', help='建噪声画像')
    p.add_argument('-o', '--output', default='noise_profile.npz')
    p.add_argument('-i', '--input', help='从 wav 建（不给就现场录）')
    p.add_argument('-s', '--seconds', type=int, default=10)
    p.add_argument('-d', '--device', default=DEV_NAME)

    p = sub.add_parser('run', parents=[common], help='实时降噪')
    p.add_argument('-p', '--profile', default='noise_profile.npz')
    p.add_argument('-o', '--output', help='增强音频落盘路径')
    p.add_argument('-d', '--device', default=DEV_NAME)

    p = sub.add_parser('file', parents=[common], help='用同一引擎处理 wav（验证用）')
    p.add_argument('-p', '--profile', default='noise_profile.npz')
    p.add_argument('-i', '--input', required=True)
    p.add_argument('-o', '--output', required=True)

    a = ap.parse_args()
    if a.list:
        import sounddevice as sd
        list_devices(sd)
        return
    if not a.cmd:
        ap.print_help()
        return
    {'profile': cmd_profile, 'run': cmd_run, 'file': cmd_file}[a.cmd](a)


if __name__ == '__main__':
    main()
