/*
 * 把对端（espnow_duo 那块 NodeMCU）的唤醒键变成电脑上的一次空格键
 *
 * 链路：GPIO4 按下 -> 状态包里的 wake 计数 +1 -> 本端发现计数变了
 *       -> tud_hid_keyboard_report(空格) -> 电脑收到一次按键
 *
 * 为什么是空格：它在电脑上本来就有现成的语义 —— 播放器/浏览器的播放暂停、
 * PPT 翻页、录音软件的开始停止，不用在电脑上装任何东西就能看到现象。
 *
 * 关掉的办法：menuconfig -> USB Device UAC Configuration -> 把「HID 空格键」关掉。
 * 关掉之后 USB 描述符退回纯 UAC，和加这个功能之前一模一样 —— 排查 Windows
 * 枚举问题时这是第一步（见 s3FN8/README.md 第三节记的那两次描述符踩坑）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 HID 发送任务
 *
 * 必须在 usb_stream_init()（也就是 TinyUSB 起来）之后调用。
 * 关掉 CONFIG_UAC_HID_KEY 时这个函数是空实现，返回 ESP_OK。
 */
esp_err_t hid_key_start(void);

/**
 * @brief 请求敲一次空格键
 *
 * 可以在任何任务、也可以在 ESP-NOW 回调里调用：它只往队列里塞一个请求就返回，
 * 真正的 USB 操作在自己的任务里做（要按下 -> 等 20ms -> 松开，不能在回调里阻塞）。
 *
 * 电脑没插、没在枚举、或者主机挂起时会被丢弃并计数，不会阻塞调用方。
 */
void hid_key_tap_space(void);

/** 成功发出去的按键次数 */
uint32_t hid_key_sent(void);

/** 因为 USB 没就绪而丢弃的次数。一直涨就说明线没插好或者电脑没认到这个键盘 */
uint32_t hid_key_dropped(void);

/** HID 接口当前能不能发（电脑已枚举且端点空闲） */
bool hid_key_ready(void);

#ifdef __cplusplus
}
#endif
