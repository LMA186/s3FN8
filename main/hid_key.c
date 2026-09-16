/*
 * HID 键盘：把对端唤醒键变成电脑上的一次空格
 *
 * 为什么要单独一个任务，而不是收到事件就地调 tud_hid_keyboard_report：
 *   一次完整的按键是「按下 -> 保持一小会 -> 松开」两份报文。中间必须有间隔，
 *   否则电脑那头两份报文挨在一起，可能被合并成"什么都没按"。而这个间隔只能
 *   靠 delay，绝不能在 ESP-NOW 回调或 20ms 的主循环里 delay。
 *
 *   更要紧的是那份「松开」报文不能省：不发的话电脑认为这个键一直按着，
 *   系统级的按键重复会开始工作，一秒钟几十个空格灌进当前窗口。
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "hid_key.h"

#if CONFIG_UAC_HID_KEY

#include "tusb.h"

static const char *TAG = "hid";

/* 按下到松开之间的保持时间。太短电脑可能识别不到，太长会触发按键重复。
 * 20ms 约等于人手最快的一次点按，各系统都稳 */
#define KEY_HOLD_MS     20

/* 队列深度 1：连按时不排队。空格是个动作触发，积压几十个补发出去
 * 只会让电脑那头莫名其妙连翻几页，不如丢掉 */
#define KEY_QUEUE_LEN   1

/* 主机挂起后唤醒它要走一次总线恢复，给的等待上限。超时就放弃这一次 */
#define RESUME_WAIT_MS  500

static QueueHandle_t s_q;
static uint32_t      s_sent;
static uint32_t      s_dropped;

/* 等 HID 端点空闲。刚发完一份报文时端点是忙的，要等主机把它取走 */
static bool wait_hid_ready(int timeout_ms)
{
    for (int waited = 0; waited < timeout_ms; waited += 2) {
        if (tud_hid_ready()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return tud_hid_ready();
}

static void hid_key_task(void *arg)
{
    (void)arg;
    uint8_t req;

    while (1) {
        if (xQueueReceive(s_q, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 电脑睡着了就先把它叫醒。没声明远程唤醒能力的主机会直接忽略，
         * 所以这里不判返回值，只看它有没有真的醒过来 */
        if (tud_suspended()) {
            tud_remote_wakeup();
            for (int waited = 0; waited < RESUME_WAIT_MS && tud_suspended(); waited += 10) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        if (!tud_mounted() || !wait_hid_ready(50)) {
            s_dropped++;
            ESP_LOGW(TAG, "USB 没就绪，丢弃一次空格（累计 %" PRIu32 " 次）", s_dropped);
            continue;
        }

        /* 按下。keycode 是个 6 键数组（USB 键盘天生支持同时按 6 个键），
         * 我们只用第一个，其余填 0 表示"没有别的键被按着" */
        uint8_t keycode[6] = { HID_KEY_SPACE, 0, 0, 0, 0, 0 };
        tud_hid_keyboard_report(0, 0, keycode);

        vTaskDelay(pdMS_TO_TICKS(KEY_HOLD_MS));

        /* 松开。这一份必须发出去，宁可多等一会 —— 漏了就是满屏空格 */
        if (!wait_hid_ready(200)) {
            ESP_LOGE(TAG, "松开报文发不出去，电脑那头可能会连续输入空格");
        }
        tud_hid_keyboard_report(0, 0, NULL);

        s_sent++;
    }
}

esp_err_t hid_key_start(void)
{
    s_q = xQueueCreate(KEY_QUEUE_LEN, sizeof(uint8_t));
    if (s_q == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 优先级 4：比音频那条通路低。按键晚几毫秒没人察觉，
     * 音频晚了电脑那头就是一声爆音 */
    if (xTaskCreate(hid_key_task, "hid_key", 3072, NULL, 4, NULL) != pdPASS) {
        vQueueDelete(s_q);
        s_q = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void hid_key_tap_space(void)
{
    if (s_q == NULL) {
        return;
    }
    uint8_t req = 1;
    /* 超时 0：队列满了直接丢。见 KEY_QUEUE_LEN 的注释 */
    if (xQueueSend(s_q, &req, 0) != pdTRUE) {
        s_dropped++;
    }
}

uint32_t hid_key_sent(void)    { return s_sent; }
uint32_t hid_key_dropped(void) { return s_dropped; }
bool     hid_key_ready(void)   { return tud_mounted() && tud_hid_ready(); }

#else  /* !CONFIG_UAC_HID_KEY */

/* 关掉 HID 时的空实现。link_rx.c 里的调用是无条件编译的，
 * 少一个就是链接错误 */
esp_err_t hid_key_start(void)  { return ESP_OK; }
void     hid_key_tap_space(void) { }
uint32_t hid_key_sent(void)    { return 0; }
uint32_t hid_key_dropped(void) { return 0; }
bool     hid_key_ready(void)   { return false; }

#endif
