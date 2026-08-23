/*
 * ESP32-S3-Dongle v1.0g —— ESP-NOW 无线麦克风接收端 → USB
 *
 * 干的事：收 NodeMCU-32S(lua-esp32/espnow_duo) 通过 ESP-NOW 发来的麦克风音频，
 *         原样通过 USB 送给电脑。
 *
 * 板子相关（依据原理图，和自检程序一致）：
 *   蓝灯 D3 = GPIO1，高电平点亮，用来指示链路状态
 *   按键 S1 = GPIO0，按下为低
 *   USB     = GPIO19/GPIO20 直连，板上没有串口芯片，走内置 USB Serial/JTAG
 *
 * 数据链路：
 *   INMP441 x2 ──I2S──> NodeMCU-32S ──ESP-NOW 信道1──> 本板 ──USB CDC──> 电脑
 *   16kHz / 双声道交织 / int16，每包 320 帧 x 2 路，50 包/秒，512 kbps
 *
 * 电脑端配套脚本：tools/recv_audio.py
 */

#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "link_rx.h"
#include "usb_stream.h"

static const char *TAG = "main";

void app_main(void)
{
    /* WiFi 要用 NVS 存射频标定数据，必须先初始化。
     * 分区被写满或格式变了就擦掉重来 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* USB 要先起来：link_rx 的接收回调一开始工作就会往里塞 PCM。
     * 顺带它会接管 stdout，后面的 printf 才不会和二进制帧打架 */
    ESP_ERROR_CHECK(usb_stream_init(LINK_SAMPLE_RATE, LINK_MIC_CHANNELS, LINK_FRAMES_PKT));

    /* USB CDC 要等主机枚举完才收得到字符，早打的几行会丢在空里 */
    vTaskDelay(pdMS_TO_TICKS(1500));

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    if (chip.model != CHIP_ESP32S3) {
        ESP_LOGE(TAG, "编译目标不是 ESP32-S3! 执行 idf.py set-target esp32s3 后重编");
    }

    ESP_ERROR_CHECK(link_rx_start());

    /* 现象都在 LED 和 link 任务的串口输出里。主任务只定期报内存，
     * 用来确认长时间跑下来没有内存泄漏 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        if (!usb_stream_is_streaming()) {
            ESP_LOGI(TAG, "运行中, 空闲内存 %" PRIu32 " 字节 (历史最低 %" PRIu32 ")",
                     esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
        }
    }
}
