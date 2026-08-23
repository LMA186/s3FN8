/*
 * ESP-NOW 接收端：从 NodeMCU-32S(espnow_duo) 收音频，转手交给 USB
 *
 * 协议、信道、包结构全部对齐发送端 lua-esp32/espnow_duo/main/link.c，
 * 任何一处改了都要两边一起改、一起重烧。
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 这三个值电脑端也要用到，放在头文件里给 main.c 传给 usb_stream_init() */
#define LINK_SAMPLE_RATE   16000
#define LINK_MIC_CHANNELS  2
#define LINK_FRAMES_PKT    320

/**
 * @brief 初始化 WiFi + ESP-NOW，启动接收/心跳任务
 *
 * 调用前 nvs_flash_init() 和 usb_stream_init() 必须已经成功。
 */
esp_err_t link_rx_start(void);

#ifdef __cplusplus
}
#endif
