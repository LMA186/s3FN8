/*
 * 把对端（espnow_duo 那块 NodeMCU）的唤醒键变成电脑上的一次按键，当前是【回车】
 *
 * 链路：GPIO4 按下 -> 状态包里的 wake 计数 +1 -> 本端发现计数变了
 *       -> tud_hid_keyboard_report(回车) -> 电脑收到一次按键
 *
 * 为什么用回车：它在电脑上本来就有现成的语义 —— 聊天软件发送消息、命令行执行、
 * PPT 翻下一页、对话框确认，不用在电脑上装任何东西就能看到现象。
 *
 * ⚠ 也正因为这样，按之前要知道电脑上哪个窗口是焦点：焦点在聊天窗口时，
 * 按一下就把输入框里的内容发出去了。
 *
 * 换成别的键：改 hid_key.c 顶部的 TAP_KEYCODE / TAP_KEYNAME，只有那一处。
 *
 * 关掉的办法：menuconfig -> USB Device UAC Configuration -> 把「HID 按键」关掉。
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
 * @brief 请求敲一次按键（键码见 hid_key.c 的 TAP_KEYCODE）
 *
 * 可以在任何任务、也可以在 ESP-NOW 回调里调用：它只往队列里塞一个请求就返回，
 * 真正的 USB 操作在自己的任务里做（要按下 -> 等 20ms -> 松开，不能在回调里阻塞）。
 *
 * 电脑没插、没在枚举、或者主机挂起时会被丢弃并计数，不会阻塞调用方。
 */
void hid_key_tap(void);

/** 成功发出去的按键次数 */
uint32_t hid_key_sent(void);

/** 因为 USB 没就绪而丢弃的次数。一直涨就说明线没插好或者电脑没认到这个键盘 */
uint32_t hid_key_dropped(void);

/** HID 接口当前能不能发（电脑已枚举且端点空闲） */
bool hid_key_ready(void);

#ifdef __cplusplus
}
#endif
