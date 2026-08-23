# ESP32-S3-Dongle —— ESP-NOW 无线麦克风接收端 → USB

把 NodeMCU-32S（`lua-esp32/espnow_duo`）通过 ESP-NOW 发来的麦克风音频收下来，
原样通过 USB 送到电脑。

```
INMP441 x2 ──I2S──> NodeMCU-32S ──ESP-NOW 信道1──> S3 Dongle ──USB CDC──> 电脑
                    (espnow_duo)                  (本工程)          recv_audio.py
```

音频格式：**16 kHz / 双声道交织 / int16 小端**，每包 320 帧 × 2 路，50 包/秒，512 kbps。

---

## 一、硬件对照（依据原理图）

| 项目 | 连线 | 说明 |
|---|---|---|
| **蓝灯 D3** | `GPIO1 → R3(1K) → 阳极`，阴极接 GND | 高电平点亮，用来指示链路状态 |
| **按键 S1** | `GPIO0 → S1 → GND` | BOOT 键，兼作状态查询 |
| **USB** | `GPIO19(D-) / GPIO20(D+)` 直连 USB-A | **板上没有串口芯片**，走 S3 内置 USB Serial/JTAG |
| **复位** | 无复位键 | 复位 = 拔插 USB |
| **芯片** | ESP32-S3FN8，内置 8MB Flash，无 PSRAM | 分区表给了 app 3MB（默认 1MB 不够，固件已 720KB） |

---

## 二、和发送端对齐的协议（改一处就要两边一起重烧）

这些常量在 [main/link_rx.c](main/link_rx.c) 顶部，全部取自
`lua-esp32/espnow_duo/main/link.c`：

| 项目 | 值 | 对端出处 |
|---|---|---|
| WiFi 信道 | 1 | `CONFIG_LINK_CHANNEL=1` |
| 状态包魔数 / 版本 | `0xE5` / v3 | `PKT_MAGIC` / `PKT_VER` |
| 音频包魔数 / 版本 | `0xE6` / v1 | `PKT_MAGIC_AUD` / `PKT_VER_AUD` |
| 状态包长度 | 11 字节 | `link_pkt_t` |
| 音频包长度 | 8 + 640×2 = **1288 字节** | `audio_hdr_t` + PCM |
| 通道标志 | `AUD_FLAG_2CH = 0x01` | 双声道交织 |
| 心跳周期 | 60 ms | `TX_EVERY=3 × TICK_MS=20` |
| 失联判定 | 1.2 秒 | `LOST_US` |

**两个必须知道的点：**

1. **本端不发心跳，对端就不会发音频。** 发送端的逻辑是
   `if (!paired || lost) continue;` —— 它要先收到我们的心跳、确认链路在线，
   才肯把 PCM 发出来。所以固件里那个 60ms 的心跳不是可选项。

2. **1288 字节超过了 ESP-NOW v1 的 250 字节上限**，两端都必须是
   ESP-NOW v2（IDF ≥ 5.4）。本工程用的 IDF 5.5.5 满足，开机 banner 会把
   实际协商到的版本打出来，是 v1 的话会显著告警。

---

## 三、USB 上的数据怎么走

同一条 CDC 链路既要走人看的中文日志、又要走机器读的二进制 PCM，做法是：

- **默认只打日志，不推流。** `idf.py monitor` 当普通串口用，看状态一切正常。
- 电脑端工具连上后发一个字节 `'S'`，固件才开始推流，同时把日志级别压成 `NONE`，
  链路上就只剩纯净的二进制帧。发 `'X'` 停流并恢复日志。
- 每帧带同步字 + 长度 + **CRC16-CCITT**，电脑端可以从任意字节重新对齐。

帧格式（[main/usb_stream.h](main/usb_stream.h) 与
[tools/recv_audio.py](tools/recv_audio.py) 必须一致）：

```
偏移  长度  含义
0     2     同步字 0xA5 0x5A
2     1     帧类型  0x01=PCM  0x10=INFO(开流第一帧，含采样率/声道数)
3     1     通道数
4     2     seq，小端 —— 直接沿用空口音频包号，电脑端据此算端到端丢包
6     2     payload 字节数，小端
8     2     payload 的 CRC16-CCITT，小端
10    N     payload
```

> 为什么不能只靠同步字：中文日志是 UTF-8，续字节里可能偶然出现 `0xA5`。
> 所以电脑端必须连长度和 CRC 一起复核才认帧。

---

## 四、烧录

### 1. 编译

VSCode 里点状态栏 🔨，或者命令行：

```powershell
cd d:\AI-Voice-recognition-hardware-main\s3FN8
.\idf.ps1 build
```

> `idf.ps1` 是本工程自带的助手。你系统 PATH 上有个 Python 3.8.6，官方
> `export.ps1` 会抓到它并报 `ESP-IDF supports Python 3.9 or newer` 直接失败，
> 这个脚本绕开了它。VSCode 扩展用的是自带的 3.11，不受影响。

### 2. 插板 + 选口

插上 dongle，设备管理器里会多出一个 **`USB 串行设备 (COMx)`**（VID `303A`，PID `1001`），
Windows 11 自带驱动。VSCode 状态栏点 🔌 选它。

> 别选 COM8/COM9 那种「蓝牙链接上的标准串行」，那是蓝牙虚拟口。

### 3. 烧录

VSCode 点 🔥（Build + Flash + Monitor），或 `Ctrl+E` 再按 `D`。命令行：

```powershell
.\idf.ps1 -p COM5 flash monitor      # COM5 换成实际的口
```

**分区表变了，第一次烧必须整片烧**（`flash` 命令本来就是三个 bin 一起写，
包含 partition-table.bin，所以正常操作即可）。若之前烧过自检程序，
保险起见可以先擦干净：

```powershell
.\idf.ps1 -p COM5 erase-flash
.\idf.ps1 -p COM5 flash monitor
```

### 4. 连不上时（本板没有复位键）

1. **按住 S1 不放**
2. 插进 USB 口
3. 过 1 秒松开
4. 芯片停在 ROM 下载模式，重新点烧录

烧完**拔下来再插上**（这板子只能这样复位）。

> USB Serial/JTAG 固化在 ROM 里，Flash 空的或程序写坏了 COM 口照样出现，
> 基本变不了砖。

### 5. 退出监视器

`Ctrl+]`

---

## 五、电脑端取流

```powershell
pip install pyserial

cd d:\AI-Voice-recognition-hardware-main\s3FN8\tools
python recv_audio.py                    # 自动找口，录成 rec_时间戳.wav
python recv_audio.py -o test.wav -t 10  # 录 10 秒
python recv_audio.py --list             # 只列串口
python recv_audio.py -p COM5            # 手动指定口
python recv_audio.py --play             # 边收边放(需 pip install sounddevice)
```

直接管道给别的程序（裸 PCM 到 stdout，状态行走 stderr）：

```powershell
python recv_audio.py -o - | ffplay -f s16le -ar 16000 -ch_layout stereo -i -
```

运行时的状态行：

```
 录   12.3s | 包   615 丢    2 ( 99.7%) CRC错   0 重同步   0 | L[########------------] -38.2  R[########------------] -37.9 dBFS
```

- **丢** 是端到端丢包（空口 + USB 都算在内），靠 seq 差值推算
- 默认丢包处**补静音**，保证 WAV 时长和真实时间对得上；`--no-fill` 可关掉
- **L/R 两条电平条**应该跟得很紧（1~3dB 以内）。长期差 10dB 以上说明对端
  有一只麦被挡住或虚焊

`Ctrl+C` 停止，脚本会发 `'X'` 让固件恢复日志，并把 WAV 收尾写好。

---

## 六、怎么判断跑通了

### 蓝灯

| 现象 | 含义 |
|---|---|
| 500ms 慢闪 | 还没找到麦克风端（对端没上电 / 信道不一致） |
| **常亮** | 已配对，链路在线，但电脑端没在取流 |
| 快闪 | 正在往电脑推音频 |

### 串口（`idf.py monitor`，未取流时）

```
>>> 配对成功! 麦克风端 XX:XX:XX:XX:XX:XX  RSSI -42 dBm
>>> 对端马上开始发音频

 音频 收2451 空口丢3 非法0 | USB 发0 满丢0 | RSSI -42 心跳100.0%  远端 全频-45dB 带内-52dB   [电脑端未取流]
```

- **空口丢** 涨得快 → RSSI 太低或信道拥挤，换 6/11 信道试试（两边都要改）
- **非法** 非 0 → 包结构对不上，多半是对端固件版本不一致
- **USB 满丢** 涨 → 电脑端读得太慢，缓冲被灌满
- **削顶 x%** 出现 → 对端增益给多了，调高它的 `PCM_GAIN_SHIFT`

---

## 七、常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 一直「正在广播寻找麦克风端」 | 对端没上电；或两边信道不一致（本端 `LINK_CHANNEL`，对端 `CONFIG_LINK_CHANNEL`，都要是 1） |
| 报「对端协议 vN，本机 v3」 | `espnow_duo` 那块板烧的是旧固件，一起重烧 |
| 报「收到 N 字节状态包，本机要 11 字节」 | 同上，包结构改过 |
| 报「对端发的是单声道音频」 | 对端 `MIC_CHANNELS` 不是 2，检查它的 `mic.h` |
| banner 里 ESP-NOW 是 v1 | IDF 版本太老，1288 字节的包会被丢。本工程用 5.5.5 不会 |
| 脚本说找不到 303A:1001 | dongle 没插好，或换根能传数据的 USB 线（有些线只有电） |
| 脚本状态行一直 0 包 | 蓝灯是不是常亮？不常亮说明空口那段就没通，先解决配对 |
| CRC错 / 重同步 一直涨 | USB 线质量差，或电脑端处理太慢导致固件端写超时 |
| 录出来左右声道对调 | 对端两只 INMP441 的 L/R 脚接反了（ch0 应接 GND，ch1 接 3V3） |
| 改了 `sdkconfig.defaults` 不生效 | 删掉工程里的 `sdkconfig` 再重新编译 |
| `main.c` 里 `#include` 全是红波浪线 | 先 Build 一次生成 `build/compile_commands.json`，再 `Ctrl+Shift+P → C/C++: Reset IntelliSense Database` |

---

## 八、工程结构

```
s3FN8/
├── CMakeLists.txt                  工程入口
├── partitions.csv                  自定义分区表(app 3MB，默认 1MB 装不下)
├── sdkconfig.defaults              板级默认配置
├── idf.ps1                         命令行编译烧录助手(绕过 Python 3.8 冲突)
├── .vscode/
│   ├── settings.json               ESP-IDF 扩展配置
│   ├── c_cpp_properties.json       IntelliSense 配置
│   └── extensions.json             推荐扩展
├── main/
│   ├── main.c                      启动流程
│   ├── link_rx.c / .h              ESP-NOW 接收 + 配对 + 心跳 + LED 指示
│   └── usb_stream.c / .h           USB 帧封装 + 推流任务 + 命令解析
└── tools/
    └── recv_audio.py               电脑端取流/存 WAV/电平监视
```
