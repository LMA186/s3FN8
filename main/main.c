/*
 * ESP32-S3-Dongle v1.0g —— ESP-NOW 无线麦克风 → 免驱 USB 声卡
 *
 * 干的事：收 NodeMCU-32S(lua-esp32/espnow_duo) 通过 ESP-NOW 发来的麦克风音频，
 *         把自己伪装成一只标准 USB 麦克风交给电脑。插上就能在 Windows 的
 *         录音设备列表里看到，不需要装驱动、也不需要跑任何脚本。
 *
 * 数据链路：
 *   INMP441 x2 ──I2S──> NodeMCU-32S ──ESP-NOW 信道1──> 本板 ──USB UAC1.0──> 电脑
 *   16kHz / 双声道交织 / int16，空口每包 320 帧 x 2 路，50 包/秒，512 kbps
 *
 * 除了麦克风，本板在电脑上还是一只【键盘】：对端那块板的唤醒键(GPIO4)按一下，
 * 电脑就收到一次空格键（播放暂停 / PPT 翻页都能直接用）。见 hid_key.c。
 * 不想要的话 menuconfig 里关掉 CONFIG_UAC_HID_KEY，描述符就退回纯 UAC。
 *
 * 板子相关（依据原理图）：
 *   蓝灯 D3 = GPIO1，高电平点亮，指示链路状态（详见 link_rx.c 的 led_update）
 *   按键 S1 = GPIO0，按下为低。既是 BOOT 键，也用来在串口上打一行状态
 *   USB     = GPIO19(D-)/GPIO20(D+) 直连，芯片内部 PHY 现在归 USB-OTG 用
 *
 * ============================ 注意烧录方式变了 ============================
 * 芯片只有一套 USB PHY。跑起来之后 PHY 归 TinyUSB(USB-OTG) 所有，
 * USB Serial/JTAG 那个 COM 口就没有了，电脑上看到的是一只麦克风。
 *
 *   烧录：按住 S1 → 插 USB → 松手。芯片停在 ROM 下载模式，
 *         此时 PHY 归 USB Serial/JTAG，COM 口重新出现，正常烧即可。
 *   日志：改走 UART0(GPIO43=TX / GPIO44=RX)，在 J1 排针上，
 *         接个 USB-TTL 就能看。不接也不影响功能，状态看蓝灯。
 * =========================================================================
 */

#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "hid_key.h"
#include "link_rx.h"
#include "uac_mic.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    if (chip.model != CHIP_ESP32S3) {
        ESP_LOGE(TAG, "编译目标不是 ESP32-S3! 执行 idf.py set-target esp32s3 后重编");
    }

    /* WiFi 要用 NVS 存射频标定数据，必须先初始化。
     * 分区被写满或格式变了就擦掉重来 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* UAC 要先起来：link_rx 的接收回调一开始工作就会往抖动缓冲里塞 PCM。
     * 这一步会把 USB PHY 从 Serial/JTAG 切给 OTG，之后 COM 口就消失了 */
    ESP_ERROR_CHECK(uac_mic_init());

    /* HID 键盘任务。必须排在 uac_mic_init() 之后（那一步才把 TinyUSB 拉起来），
     * 也必须排在 link_rx_start() 之前 —— 链路一通就可能收到唤醒事件，
     * 那时队列得已经存在，否则头几次按键会被静默丢掉 */
    ESP_ERROR_CHECK(hid_key_start());

    ESP_ERROR_CHECK(link_rx_start());

    /* 状态都在蓝灯和 link 任务的 UART 输出里。主任务只定期报内存，
     * 用来确认长时间跑下来没有内存泄漏 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "运行中, 空闲内存 %" PRIu32 " 字节 (历史最低 %" PRIu32 ")",
                 esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    }
}
