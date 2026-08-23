/*
 * 把 ESP-NOW 收到的 PCM 通过 USB 送给电脑
 *
 * 走的是 ESP32-S3 内置的 USB Serial/JTAG（板上没有串口芯片，只有这一条路）。
 * 同一条 CDC 链路上既要走人看的中文日志、又要走机器读的二进制 PCM，
 * 所以做了两件事：
 *
 *   1. PCM 一律封成带同步字 + 长度 + CRC16 的帧，电脑端可以从任意字节重新对齐；
 *   2. 默认不发 PCM，只打日志。电脑端工具连上后发一个 'S' 才开始推流，
 *      推流期间固件把日志级别压成 NONE，链路上就只剩纯净的帧。
 *
 * 这样 idf.py monitor 仍然能当普通串口用来看状态，而 recv_audio.py 拿到的
 * 是干净的数据流，两者不用二选一。
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 帧格式（电脑端 tools/recv_audio.py 必须一致）----------------
 *
 *   偏移  长度  含义
 *   0     2     同步字 0xA5 0x5A
 *   2     1     帧类型
 *   3     1     通道数（PCM 帧）
 *   4     2     seq，小端。直接沿用空口音频包的序号，电脑端据此算丢包
 *   6     2     payload 字节数，小端
 *   8     2     payload 的 CRC16-CCITT，小端
 *   10    N     payload
 *
 * 10 字节头配 1280 字节负载，开销 0.8%。
 * 同步字有可能在中文日志的 UTF-8 续字节里偶然出现，所以电脑端必须连
 * 长度和 CRC 一起校验才认帧 —— 只认同步字会误判。
 */
#define USB_FRAME_SYNC0   0xA5
#define USB_FRAME_SYNC1   0x5A

#define USB_FRAME_PCM     0x01   /* payload = int16 小端交织 PCM */
#define USB_FRAME_INFO    0x10   /* payload = usb_stream_info_t，开流时先发一帧 */

typedef struct __attribute__((packed)) {
    uint8_t  sync0;
    uint8_t  sync1;
    uint8_t  type;
    uint8_t  ch;
    uint16_t seq;
    uint16_t len;
    uint16_t crc;
} usb_frame_hdr_t;

/* 开流第一帧，让电脑端不用把采样率写死在脚本里 */
typedef struct __attribute__((packed)) {
    uint32_t sample_rate;     /* 16000 */
    uint8_t  channels;        /* 2 */
    uint8_t  bits;            /* 16 */
    uint16_t frames_per_pkt;  /* 320 */
} usb_stream_info_t;

/* 电脑端 tools/recv_audio.py 里的 HDR_FMT="<BBBBHHH" 和这里必须同尺寸，
 * 差一个字节整条流就对不齐。钉在编译期 */
_Static_assert(sizeof(usb_frame_hdr_t) == 10, "帧头必须 10 字节，见 tools/recv_audio.py");
_Static_assert(sizeof(usb_stream_info_t) == 8, "INFO 负载必须 8 字节，见 tools/recv_audio.py");

/**
 * @brief 安装 USB Serial/JTAG 驱动，启动推流任务和命令任务
 *
 * @param sample_rate     采样率，写进 INFO 帧
 * @param channels        通道数
 * @param frames_per_pkt  每包每通道的采样点数
 */
esp_err_t usb_stream_init(uint32_t sample_rate, uint8_t channels, uint16_t frames_per_pkt);

/** 电脑端是否已经要求推流。ESP-NOW 回调里据此决定要不要往下送 */
bool usb_stream_is_streaming(void);

/**
 * @brief 把一包 PCM 交给 USB 推流任务
 *
 * 可以在 ESP-NOW 接收回调里直接调用：内部只做一次非阻塞的环形缓冲写入，
 * 真正的 USB 写在独立任务里做，不会拖住 WiFi 任务。
 *
 * @return true 已入队；false 缓冲满，整包丢弃（电脑端读得太慢或没在读）
 */
bool usb_stream_send_pcm(uint16_t seq, const void *pcm, size_t bytes);

/**
 * @brief 给人看的串口输出，推流期间自动丢弃
 *
 * 必须用它代替 printf：CDC 上只有一条流，如果一边在写二进制帧、另一边在写
 * 文字，两者会按字节交织，帧就废了。这个函数和帧写入用同一把锁，并且在锁内
 * 复核推流状态，从根上避免这种交织。
 */
void usb_stream_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** 因缓冲满而丢弃的包数 */
uint32_t usb_stream_drops(void);

/** 已经成功写进 USB 的帧数 */
uint32_t usb_stream_frames(void);

#ifdef __cplusplus
}
#endif
