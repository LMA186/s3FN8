# ESP32-S3-Dongle —— ESP-NOW 无线麦克风 → 免驱 USB 声卡

把 NodeMCU-32S（`lua-esp32/espnow_duo`）通过 ESP-NOW 发来的麦克风音频收下来，
**把自己伪装成一只标准 USB 麦克风**交给电脑。插上就能在 Windows 录音设备列表里
看到，不用装驱动、不用跑任何脚本。

```
INMP441 x2 ──I2S──> NodeMCU-32S ──ESP-NOW 信道1──> S3 Dongle ──USB UAC1.0──> 电脑
                    (espnow_duo)                   (本工程)      「ESP-NOW Wireless Mic」
```

音频格式：**16 kHz / 双声道 / int16**，空口每包 320 帧 × 2 路，50 包/秒，512 kbps。

---

## 一、硬件对照（依据原理图）

| 项目 | 连线 | 说明 |
|---|---|---|
| **蓝灯 D3** | `GPIO1 → R3(1K) → 阳极`，阴极接 GND | 高电平点亮。**纯 UAC 方案下这是唯一的现场状态指示** |
| **按键 S1** | `GPIO0 → S1 → GND` | BOOT 键，兼作状态查询（打一行到 UART0） |
| **USB** | `GPIO19(D-) / GPIO20(D+)` 直连 USB-A | 正是芯片内部 USB PHY 的引出脚，硬件无需任何改动 |
| **UART0** | `GPIO43(TX) / GPIO44(RX)` → J1 排针 | 日志出口。不接也不影响功能 |
| **复位** | 无复位键 | 复位 = 拔插 USB |
| **芯片** | ESP32-S3FN8，内置 8MB Flash，无 PSRAM | 分区表给 app 3MB，当前固件 726KB |

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

## 三、为什么能做成免驱麦克风

ESP32-S3 内部只有**一套** USB PHY，两个控制器抢它（IDF `hal/usb_phy_types.h`）：

```c
typedef enum {
    USB_PHY_CTRL_OTG,          /* 跑 TinyUSB，能做任意 USB 设备 */
    USB_PHY_CTRL_SERIAL_JTAG,  /* 固定就是个 CDC 串口 */
} usb_phy_controller_t;
```

而这套 PHY 的引出脚就是原理图上那两根：

```c
#define USBPHY_DP_NUM 20   /* D+ */
#define USBPHY_DM_NUM 19   /* D- */
```

所以硬件一点不用改，只要把 PHY 交给 OTG、descriptor 声明成 **UAC 1.0**，
Windows 就用系统自带的 `usbaudio.sys` 认它，全程免驱。

用的组件是 [`espressif/usb_device_uac`](https://components.espressif.com/components/espressif/usb_device_uac)
v0.2.0（内部封装 TinyUSB）。

> 它发的是 **UAC 2.0** 描述符（`bFunctionProtocol=0x20`），Windows 挂的是
> `usbaudio2.sys`，同样免驱（Win10 1703+ 起内置），不是 UAC 1.0 的 `usbaudio.sys`。

### 组件在多声道下有 bug，已 vendor 到本地修好

**没有**直接用仓库版本，而是拷进了 [components/usb_device_uac/](components/usb_device_uac/)。

原因：0.2.0 把麦克风的 `bmChannelConfig` 硬编码成 `AUDIO_CHANNEL_CONFIG_FRONT_CENTER`
（`0x00000004`，只有 1 个 bit），而 `bNrChannels` 用的是 `MIC_CHANNEL_NUM`。
只有 `MIC_CHANNEL_NUM==1` 时两者才自洽。

设成 2 之后描述符就变成**「声明 2 个声道，但声道位图只标了 1 个」**，
Windows 的 `usbaudio2.sys` 对这个一致性检查很严，直接拒绝启动：

```
Status       : Error
FriendlyName : usb uac
ProblemCode  : 10   (CM_PROB_FAILED_START)
Service      : usbaudio2
```

现象是设备管理器里一个带感叹号的「usb uac」，而**录音设备列表里什么都不出现**。
（上游在扬声器那条路径上用的是 `NON_PREDEFINED`，正因为声道数可变；麦克风这条漏了。）

修改在 [components/usb_device_uac/tusb/uac_config.h](components/usb_device_uac/tusb/uac_config.h)
里标了「本地修改 #1」的那段，按声道数给出自洽的位图：

| 声道数 | bmChannelConfig |
|---|---|
| 1 | `FRONT_CENTER` （保持上游行为）|
| 2 | `FRONT_LEFT｜FRONT_RIGHT` = `0x03`（正经立体声，Windows 会标 L/R）|
| 其它 | `NON_PREDEFINED` = `0x00`（对任意 bNrChannels 都合法）|

组件的 `CMakeLists.txt` 把 `tusb/` 硬编码进了 TinyUSB 的 include 路径，
没办法从工程外面覆盖，所以只能整个 vendor 进来。

### 本地修改 #2：麦克风的 Feature Unit 控制请求根本没实现

改完 #1 之后**症状一模一样**（还是代码 10），但这次描述符是完全合规的
——145 字节逐字段核对过，`wTotalLength`、CS_AC 的 `wTotalLength=64`
(9+8+17+12+18)、端点 `wMaxPacketSize=0x44`、`bInterval=1` 全都对。
问题在控制端点上：

```c
// usb_device_uac.c，上游原样
static bool tud_audio_feature_unit_get_request(...)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);   // 只认扬声器 0x02
    ...
}

bool tud_audio_get_req_entity_cb(...)
{
    if (request->bEntityID == UAC2_ENTITY_CLOCK)            { ... }  // 0x04
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) { ... }  // 0x02
    else { /* 返回 false */ }
}
```

而描述符里麦克风的 Feature Unit 是 **`UAC2_ENTITY_MIC_FEATURE_TERMINAL` = 0x12**，
并且 `MIC_CTRL` 把静音和音量都声明成了「主机可编程」(`0x0000000F`)。于是：

```
Windows 向实体 0x12 查音量
  → 掉进 else，返回 false
  → TinyUSB STALL 掉这个控制请求
  → usbaudio2.sys 启动失败，CM_PROB_FAILED_START
```

**声明了控制项却不实现**——这个 bug 和声道数无关，纯麦克风配置下必然触发。

顺带修掉一个越界：`mute[]` / `volume[]` 原来按 `N_CHANNELS_TX`（扬声器声道数）定长，
关掉扬声器之后只有 1 个元素，而麦克风有 master+2 共 3 个通道，
按 `bChannelNumber` 索引会写到结构体后面的字段上去。

修改点在 [components/usb_device_uac/usb_device_uac.c](components/usb_device_uac/usb_device_uac.c)
里标了「本地修改 #2」的 5 处：两个分发器、两个 `TU_ASSERT`、数组定长改成
`UAC_MAX_FU_CHANNELS = max(TX, RX)`。

> 目前音量/静音是**收下但不生效**的（Windows 的滑块会动，但不改变实际增益），
> 增益在 NodeMCU 那一侧调。要让滑块真正起作用的话，把 `set_volume_cb`
> 接到 [main/uac_mic.c](main/uac_mic.c) 里在 `input_cb` 出口乘一个系数即可。

### 排查这类问题的通用方法

代码 10 只说明「驱动起不来」，不区分是描述符非法还是控制请求被拒。分辨方法：

- **描述符问题** → 从 ELF 里挖出来逐字段核对（见上）
- **控制请求问题** → 描述符核对无误却仍然失败，就去看固件里所有
  `tud_audio_*_cb` 的返回路径，任何一条返回 `false` 都会变成 STALL

改完可以直接从 ELF 里挖描述符自查：

```bash
objdump -t build/espnow_usb_mic.elf | grep desc_configuration
objdump -s -j .flash.rodata --start-address=0x<上面的地址> --stop-address=0x<+0x91> build/espnow_usb_mic.elf
```

Input Terminal(4.7.2.4) 应该长这样 —— `bNrChannels=02` 后面紧跟 `bmChannelConfig=03 00 00 00`：

```
11 24 02 11 01 02 13 04 | 02 | 03 00 00 00 | 00 04 00 00
```

### 代价

| 失去的 | 补偿 |
|---|---|
| USB Serial/JTAG 那个 COM 口 | 日志改走 **UART0**（GPIO43/44，J1 排针），接个 USB-TTL 就能看 |
| `idf.py monitor` 走 USB | 同上；不接串口就看蓝灯 |
| 之前的 `tools/recv_audio.py` | 作废了，已删除。需要的话 `git checkout e758070 -- tools/` 找回 |

---

## 四、时钟漂移怎么处理（[main/uac_mic.c](main/uac_mic.c)）

采样时钟在 NodeMCU 的 I2S 上，播放时钟在电脑的 USB 主机上，两边晶振各
±10~20ppm，跑久了必然一边撑满一边抽干。三条水位线兜住：

| 水位 | 值 | 作用 |
|---|---|---|
| `PREBUF_MS` | 100 ms | 攒够才开始交付，给网络抖动留垫子 |
| `HIGH_WATER_MS` | 250 ms | 超过就丢一整块追延迟（发送端偏快） |
| 欠载 | — | 没数据就交静音并计数（发送端偏慢或丢包） |
| `JITTER_MS` | 400 ms | 缓冲总容量 |

**和 `voice_assistant` 那版不同的一点**：那边的消费方是 AFE，`feed()` 比实时快得多，
会把缓冲一路抽干，所以必须自己维护绝对时间轴。这里的消费方是 USB 等时端点，
主机每 10ms 雷打不动只要 10ms 的量，**天然就是实时节拍**，在回调里阻塞等数据
就等于被主机定速了，不需要额外的时间轴。

⚠️ 组件的 `usb_mic_task` 是个**没有 delay 的紧循环**，节拍完全由 `input_cb` 自己把握。
所以 [uac_mic.c](main/uac_mic.c) 里每条返回路径都保证要么阻塞在
`xStreamBufferReceive`、要么显式 `vTaskDelay` —— 无条件立刻返回会让那个任务空转吃满一个核。

---

## 五、烧录（**方式变了，仔细看**）

固件跑起来之后 PHY 归 TinyUSB，**COM 口消失**，电脑上看到的是一只麦克风。
但 ROM 下载模式用的是 USB Serial/JTAG 控制器，所以还有路走：

1. **按住板上的 S1 键不放**
2. 把 dongle 插进 USB 口
3. 过 1 秒松手 —— 此时芯片停在 ROM 下载模式，COM 口重新出现
4. 烧录：

```powershell
cd d:\AI-Voice-recognition-hardware-main\s3FN8
.\idf.ps1 -p COMx flash          # COMx 换成重新出现的那个口
```

5. 烧完**拔下来再插上**（这块板没有复位键），程序开始跑，
   电脑上就多出一只麦克风了。

> `idf.ps1` 绕开了系统 PATH 上那个 Python 3.8.6（它会让官方 `export.ps1` 直接报错退出）。

VSCode 里用 ESP-IDF 扩展也一样：先手动进下载模式，再点 ⚡ Flash。

---

## 六、电脑端怎么用

**不需要任何脚本。** 插上之后：

- 设备管理器 → 声音、视频和游戏控制器 → 多出 **`ESP-NOW Wireless Mic`**
- 设置 → 系统 → 声音 → 输入 → 选它
- 任何录音软件（录音机 / Audacity / 浏览器 / 微信 / ASR 程序）直接选这个输入设备

VID:PID = `303A:8001`。**故意和之前 CDC 方案的 `303A:1001` 错开** ——
Windows 按 VID:PID 缓存设备描述符，同一个 PID 从 CDC 变成音频类，
大概率变成「无法识别的设备」，还得手工清缓存。

格式是 **16 kHz 立体声**。两路来自间距 5cm 的双 INMP441 阵列，
ch0 = 左槽（L/R→GND），ch1 = 右槽（L/R→3V3）。要做波束成形/降噪的话素材是全的；
只想要单声道就在录音软件里降混。

---

## 七、蓝灯含义（没接串口时唯一的状态来源）

| 现象 | 含义 | 该查什么 |
|---|---|---|
| **1Hz 慢闪** | 还没找到麦克风端 | NodeMCU 是不是没上电；两边信道是否都是 1 |
| **常亮** | 已配对、音频在收，但电脑没在录音 | 正常。在电脑上打开录音软件即可 |
| **快闪** | 电脑正在录音，整条链路都通了 | 正常工作状态 |
| **不亮** | 固件没跑起来 | 按住 S1 重插进下载模式，重新烧录 |

---

## 八、看日志（可选）

接个 USB-TTL 到 J1 排针的 `TXD0` / `GND`，115200 8N1：

```
=================================================
  ESP32-S3-Dongle  ESP-NOW 无线麦克风 -> USB 声卡
=================================================
  本机 MAC   : XX:XX:XX:XX:XX:XX
  WiFi 信道  : 1  (必须和 espnow_duo 的 LINK_CHANNEL 一致)
  ESP-NOW    : v2
  空口格式   : 16000 Hz / 2 声道交织 / int16 小端 / 每包 320 帧
  USB 身份   : ESP-NOW Wireless Mic  (UAC 1.0, Windows 免驱)
=================================================

>>> 配对成功! 麦克风端 XX:XX:XX:XX:XX:XX  RSSI -42 dBm
 音频 收 615 空口丢 2 非法 0 | 缓冲 118ms 欠载 0 追帧 0 满丢 0 | RSSI -42 心跳 99.7%  远端 音量 34 全频-38dB 带内-41dB
```

判读：

| 字段 | 正常 | 异常说明 |
|---|---|---|
| `缓冲` | 100~250ms 之间浮动 | 长期贴 250 = 发送端偏快；长期接近 0 = 偏慢/丢包 |
| `欠载` | 不涨 | 持续涨 = 空口丢包严重，或对端 I2S 供数不足 |
| `追帧` | 偶尔 +1 | 频繁涨 = 两边时钟差得多，属正常补偿 |
| `满丢` | 0 | 非 0 = 电脑没在取数但空口还在灌 |
| `心跳` | >95% | 偏低 = 信号差，换信道或缩短距离 |
| `削顶` | 不出现 | 出现就去调对端的 `PCM_GAIN_SHIFT` |

---

## 九、工程结构

```
s3FN8/
├── CMakeLists.txt              工程入口
├── partitions.csv              app 给到 3MB（默认 1MB 不够）
├── sdkconfig.defaults          板级 + UAC + 控制台改 UART0
├── idf.ps1                     命令行助手（绕过 Python 3.8 冲突）
├── .vscode/                    ESP-IDF 扩展 + IntelliSense 配置
├── components/
│   └── usb_device_uac/         vendor 自 espressif/usb_device_uac 0.2.0
│                               改了多声道的 bmChannelConfig，见第三节
└── main/
    ├── idf_component.yml       只声明 IDF 版本（UAC 组件已 vendor 到上面）
    ├── main.c                  启动顺序
    ├── link_rx.c/.h            ESP-NOW 收音频 + 心跳 + 配对 + LED
    └── uac_mic.c/.h            抖动缓冲 + UAC 取数回调
```

---

## 十、常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 插上去没有麦克风，也没有 COM 口 | 固件没跑。按住 S1 重插进下载模式重烧 |
| 显示「无法识别的 USB 设备」 | Windows 缓存了旧描述符。换个 USB 口，或设备管理器里卸载后重插 |
| 设备管理器里有带感叹号的「usb uac」，代码 10 | 描述符不合法，**或**某个控制请求被 STALL 了。见第三节的两处本地修改和排查方法 |
| 麦克风有了，但录出来全是静音 | 蓝灯是慢闪 = 没配对。检查 NodeMCU 上电、两边信道都是 1 |
| 录音断断续续 | 看 UART 日志的 `欠载`；空口丢包严重就换信道或缩短距离 |
| 声音有明显延迟 | 正常，约 130~150ms（20ms 空口包 + 100ms 起播 + USB）。调低 `PREBUF_MS` 可减少，代价是抗抖动变差 |
| 改了 `sdkconfig.defaults` 不生效 | 删掉 `sdkconfig` 再重新编译 |
| 想回到之前的 CDC 推流方案 | `git checkout e758070` |
