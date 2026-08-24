/*
 * 把 ESP-NOW 收到的 PCM 变成一只标准 USB 麦克风
 *
 * ============================ 为什么是 UAC ============================
 *
 * ESP32-S3 内部只有一套 USB PHY，两个控制器抢它（见 IDF hal/usb_phy_types.h）：
 *
 *     USB_PHY_CTRL_OTG           跑 TinyUSB，能做任意 USB 设备
 *     USB_PHY_CTRL_SERIAL_JTAG   固定是个 CDC 串口
 *
 * 之前走的是后者，PCM 靠自定义帧协议 + 电脑端 Python 脚本取流。改成前者之后，
 * 描述符声明成 UAC 1.0，Windows 用系统自带的 usbaudio.sys 直接认成麦克风，
 * 录音机/微信/浏览器/任何 ASR 软件都能直接选到，不需要任何脚本。
 *
 * 代价是 USB Serial/JTAG 没了：日志改走 UART0(GPIO43/44，在 J1 排针上)，
 * 烧录要按住 S1 再插 USB 进 ROM 下载模式。
 *
 * ============================ 时钟漂移 ============================
 *
 * 采样时钟在 NodeMCU 那块板的 I2S 上，播放时钟在电脑的 USB 主机上，
 * 两边晶振各 ±10~20ppm，跑久了必然一边撑满一边抽干。这里用三条水位线兜住：
 *
 *   PREBUF_MS      攒够这么多才开始交付，给网络抖动留垫子
 *   HIGH_WATER_MS  超过就丢一块，把攒下来的延迟追回来（发送端偏快）
 *   欠载           没数据就交静音并计数（发送端偏慢，或者真丢包了）
 *
 * 和 voice_assistant 那版不同的是：这里不需要自己维护绝对时间轴。
 * 那边的消费方是 AFE，feed() 比实时快得多，会把缓冲一路抽干；
 * 而这里的消费方是 USB 等时端点，主机每 10ms 雷打不动只要 10ms 的量，
 * 天然就是实时节拍，我们在回调里阻塞等数据就等于被主机定速了。
 * =====================================================================
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 USB-OTG + TinyUSB，把自己注册成 UAC 1.0 麦克风
 *
 * 采样率/声道数/每次取数间隔由 menuconfig 里的 UAC_* 决定，
 * 本工程在 sdkconfig.defaults 里固定成 16000Hz / 2 声道 / 10ms。
 */
esp_err_t uac_mic_init(void);

/**
 * @brief 把一包空口来的 PCM 塞进抖动缓冲
 *
 * 可以直接在 ESP-NOW 接收回调里调用，内部只做一次非阻塞写。
 *
 * @return true 已入队；false 缓冲满，整包丢弃
 */
bool uac_mic_push(const void *pcm, size_t bytes);

/**
 * @brief 链路断了的时候调用：丢掉缓冲里的陈音频并重新进入预缓冲
 *
 * 不清的话，恢复之后那几百毫秒的旧声音会一直顶在前面，延迟再也降不下来。
 */
void uac_mic_flush(void);

/** 电脑是否正在录音（主机在持续取数）。LED 和串口状态行用 */
bool uac_mic_host_active(void);

/* ---- 统计，串口状态行用 ---- */
uint32_t uac_mic_underruns(void);   /* 缓冲空，交了静音的次数 */
uint32_t uac_mic_drops(void);       /* 缓冲满，整包丢弃的次数 */
uint32_t uac_mic_catchups(void);    /* 超高水位，主动丢块追延迟的次数 */
int      uac_mic_depth_ms(void);    /* 当前缓冲深度，毫秒 */

#ifdef __cplusplus
}
#endif
