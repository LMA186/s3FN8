#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "link_rx.h"
#include "usb_stream.h"

static const char *TAG = "link";

/* ============ 下面这一段必须和发送端 espnow_duo/main/link.c 完全一致 ============
 * 对不上的表现不是报错，而是「永远配不上对」或者「收到一堆乱码 PCM」，
 * 所以每个常量后面都标了对应的来源 */
#define LINK_CHANNEL  1         /* 对端 CONFIG_LINK_CHANNEL=1 */

#define PKT_MAGIC     0xE5      /* 状态包(心跳) */
#define PKT_VER       3         /* 对端 PKT_VER=3，版本不同双方都会直接拒收 */
#define PKT_FLAG_MIC  0x01      /* 对端置位表示它接了麦克风 */

#define PKT_MAGIC_AUD 0xE6      /* 音频包 */
#define PKT_VER_AUD   1
#define AUD_FLAG_2CH  0x01      /* 置位表示包体是双声道交织的 */

#define TICK_MS       20        /* 主循环节拍 */
#define TX_EVERY      3         /* 每 60ms 发一个心跳，和对端同频 */
#define PRINT_EVERY   50        /* 每 1 秒打一行 */
#define LOST_US       (1200 * 1000LL)

#define LED_GPIO      GPIO_NUM_1   /* 板载蓝灯 D3，高电平点亮 */
#define BTN_GPIO      GPIO_NUM_0   /* 板载 BOOT 键 S1，按下为低 */

/* 状态包，11 字节。字段含义见对端 link.c 的注释 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  ver;
    uint16_t seq;
    uint8_t  level;       /* 对端音量 0~100 */
    uint8_t  event;       /* 对端 BOOT 键计数 */
    uint8_t  flags;
    uint8_t  rsv;         /* 对端最近 1 秒的削顶占比 0~100 */
    int8_t   dbfs_full;   /* 高通之前、全频段 RMS */
    int8_t   dbfs_band;   /* 高通之后、语音带内 RMS */
    uint8_t  wake;        /* 对端外接唤醒键的按下计数 */
} link_pkt_t;

/* 音频包：8 字节头 + samples 个 int16 小端交织 PCM。
 * 320 帧 x 2 路 = 640 个 int16 = 1288 字节/包、50 包/秒 —— 超过了 ESP-NOW v1
 * 的 250 字节上限，两端都必须是 v2 (IDF >= 5.4)，开机 banner 会打印实际版本 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  ver;
    uint16_t seq;
    uint16_t samples;   /* 本包 int16 总数 = 帧数 x 通道数 */
    uint8_t  flags;
    uint8_t  rsv;
} audio_hdr_t;

/* 结构体尺寸是这条链路唯一的硬约束：对端按这个长度发、本端按这个长度收，
 * 差一个字节收到的就是一堆错位的噪声，而且不会有任何报错。
 * 所以钉死在编译期 —— 以后谁不小心动了字段，编译直接过不去 */
_Static_assert(sizeof(link_pkt_t) == 11, "状态包必须 11 字节，见 espnow_duo/main/link.c");
_Static_assert(sizeof(audio_hdr_t) == 8, "音频包头必须 8 字节，见 espnow_duo/main/link.c");
/* ============================================================================ */

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t s_self_mac[6];
static uint8_t s_peer_mac[6];

/* 回调跑在 WiFi 任务里，和主循环并发。回调里只做拷贝和计数，
 * add_peer、打印这些活都甩给主循环 */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static bool     s_paired;
static bool     s_pending_pair;
static uint8_t  s_pending_mac[6];

static int64_t  s_last_rx_us;
static uint8_t  s_remote_level;
static uint8_t  s_remote_clip;
static int8_t   s_remote_full = -120;
static int8_t   s_remote_band = -120;
static bool     s_remote_has_mic;
static int8_t   s_rssi;
static uint16_t s_rx_last_seq;
static bool     s_rx_seq_valid;
static uint32_t s_rx_ok, s_rx_lost;

static uint16_t s_tx_seq;

/* 音频统计。只在回调里写、主循环里读，都是 32 位对齐的标量，
 * 读到旧值最多让某一行打印差一点，不值得为它加锁 */
static uint32_t s_aud_rx;        /* 收到的音频包数 */
static uint32_t s_aud_lost;      /* 按序号差算出来的空口丢包 */
static uint32_t s_aud_bad;       /* 长度/通道数不合法而丢弃 */
static uint16_t s_aud_last_seq;
static bool     s_aud_seq_valid;

/* ---------------- ESP-NOW 回调 ---------------- */

static void on_recv_status(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)len;
    link_pkt_t p;
    memcpy(&p, data, sizeof(p));
    if (p.ver != PKT_VER) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            ESP_LOGE(TAG, "对端协议 v%u，本机 v%d —— espnow_duo 那块板要一起重烧",
                     p.ver, PKT_VER);
        }
        return;
    }

    int8_t rssi = (info->rx_ctrl != NULL) ? info->rx_ctrl->rssi : 0;

    portENTER_CRITICAL(&s_mux);

    if (!s_paired) {
        memcpy(s_pending_mac, info->src_addr, 6);
        s_pending_pair = true;
    } else if (memcmp(info->src_addr, s_peer_mac, 6) != 0) {
        portEXIT_CRITICAL(&s_mux);
        return;    /* 附近还有第三块板，只认第一个配上的 */
    }

    /* 丢包统计：序号差多少就是中间丢了多少。
     * 差值为负说明对端复位了（seq 从 0 重来），统计清零重新开始 */
    if (s_rx_seq_valid) {
        int16_t d = (int16_t)(p.seq - s_rx_last_seq);
        if (d > 0) {
            s_rx_lost += (uint32_t)(d - 1);
            s_rx_ok++;
            s_rx_last_seq = p.seq;
        } else if (d < -8) {
            s_rx_ok       = 1;
            s_rx_lost     = 0;
            s_rx_last_seq = p.seq;
        }
    } else {
        s_rx_seq_valid = true;
        s_rx_last_seq  = p.seq;
        s_rx_ok        = 1;
        s_rx_lost      = 0;
    }

    s_remote_level   = (p.level > 100) ? 100 : p.level;
    s_remote_clip    = (p.rsv > 100) ? 100 : p.rsv;
    s_remote_full    = p.dbfs_full;
    s_remote_band    = p.dbfs_band;
    s_remote_has_mic = (p.flags & PKT_FLAG_MIC) != 0;
    s_rssi           = rssi;
    s_last_rx_us     = esp_timer_get_time();

    portEXIT_CRITICAL(&s_mux);
}

static void on_recv_audio(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    audio_hdr_t hdr;
    memcpy(&hdr, data, sizeof(hdr));
    if (hdr.ver != PKT_VER_AUD) {
        return;
    }

    /* 长度自洽性检查。空气里的包不可信，头里写的 samples 必须和实际长度对得上，
     * 否则下面按 samples 去读就会越界；samples 还必须是通道数的整数倍，
     * 否则写出去的 PCM 会错开半帧，电脑端从此左右声道对调 */
    size_t need = sizeof(audio_hdr_t) + (size_t)hdr.samples * sizeof(int16_t);
    if (hdr.samples == 0 || need != (size_t)len ||
        (hdr.samples % LINK_MIC_CHANNELS) != 0) {
        s_aud_bad++;
        return;
    }
    if ((hdr.flags & AUD_FLAG_2CH) == 0) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            ESP_LOGE(TAG, "对端发的是单声道音频，本端按双麦配置 —— 对端要重烧");
        }
        s_aud_bad++;
        return;
    }

    /* 配对之前不收音频：还没确认对端是谁，收了也不知道该不该信 */
    bool ok;
    portENTER_CRITICAL(&s_mux);
    ok = s_paired && (memcmp(info->src_addr, s_peer_mac, 6) == 0);
    portEXIT_CRITICAL(&s_mux);
    if (!ok) {
        return;
    }

    if (s_aud_seq_valid) {
        int16_t d = (int16_t)(hdr.seq - s_aud_last_seq);
        if (d > 0) {
            s_aud_lost += (uint32_t)(d - 1);
            s_aud_last_seq = hdr.seq;
        } else if (d < -8) {          /* 对端复位了 */
            s_aud_lost     = 0;
            s_aud_last_seq = hdr.seq;
        } else {
            return;                   /* 乱序/重复包，丢掉，否则电脑端时间轴会错位 */
        }
    } else {
        s_aud_seq_valid = true;
        s_aud_last_seq  = hdr.seq;
    }

    s_aud_rx++;

    /* 空口序号原样带给电脑端：这样电脑端算出来的丢包是「端到端」的，
     * 包含了 USB 这一段，而不只是空口那一段 */
    usb_stream_send_pcm(hdr.seq, data + sizeof(audio_hdr_t),
                        (size_t)hdr.samples * sizeof(int16_t));
}

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < 2 || memcmp(info->src_addr, s_self_mac, 6) == 0) {
        return;
    }
    /* 空气里可能有别人的 ESP-NOW 包，先拿魔数挡一道 */
    if (data[0] == PKT_MAGIC && len == (int)sizeof(link_pkt_t)) {
        on_recv_status(info, data, len);
    } else if (data[0] == PKT_MAGIC) {
        /* 魔数对上了、长度不对：协议升过版本，对端还烧着旧固件。
         * 不报的话现象只是「一直配不上对」，很难联想到是版本问题 */
        static bool warned = false;
        if (!warned) {
            warned = true;
            ESP_LOGE(TAG, "收到 %d 字节状态包，本机要 %d 字节 —— 对端要一起重烧",
                     len, (int)sizeof(link_pkt_t));
        }
    } else if (data[0] == PKT_MAGIC_AUD && len > (int)sizeof(audio_hdr_t)) {
        on_recv_audio(info, data, len);
    }
}

static void on_send(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;   /* 本端只发几字节心跳，成功率由对端统计，这里不重复 */
}

/* ---------------- 初始化 ---------------- */

static esp_err_t wifi_init(void)
{
    /* 纯 ESP-NOW 不连路由器，但 esp_wifi_init 内部要注册事件处理器，
     * 所以 netif 和默认事件循环还是得先建起来 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));  /* 不连 AP，不用往 flash 里存配置 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 省电模式下 STA 会周期性关射频，收包时断时续。
     * 对心跳只是慢一拍，对连续音频流就是直接听感断续，必须关掉 */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* 不连 AP，信道自己定死，必须和对端 CONFIG_LINK_CHANNEL 一致 */
    ESP_ERROR_CHECK(esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE));
    return ESP_OK;
}

static esp_err_t espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));

    /* 广播也要先 add_peer，否则 esp_now_send 到广播地址会返回 NOT_FOUND */
    esp_now_peer_info_t peer = {
        .channel = LINK_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST, 6);
    return esp_now_add_peer(&peer);
}

static esp_err_t add_unicast_peer(const uint8_t *mac)
{
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }
    esp_now_peer_info_t peer = {
        .channel = LINK_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac, 6);
    return esp_now_add_peer(&peer);
}

/* ---------------- 板载 LED / 按键 ---------------- */

static void io_init(void)
{
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(LED_GPIO, 0);

    /* S1 只把 GPIO0 接到 GND，必须开内部上拉才读得到确定的电平 */
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BTN_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
}

/* 蓝灯三种含义，隔着壳子也能看出链路状态：
 *   慢闪(1Hz)  还没找到麦克风端
 *   常亮       已配对，但电脑端没在取流
 *   快闪       正在往电脑推音频（每 10 包翻转一次，约 2.5Hz）
 */
static void led_update(uint32_t tick, bool paired, bool lost)
{
    int level;
    if (!paired || lost) {
        level = (int)((tick / 25) & 1);          /* 25 x 20ms = 500ms */
    } else if (!usb_stream_is_streaming()) {
        level = 1;
    } else {
        level = (int)((s_aud_rx / 10) & 1);
    }
    gpio_set_level(LED_GPIO, level);
}

/* ---------------- 开机横幅 ---------------- */

static void print_banner(void)
{
    uint32_t nowver = 0;
    esp_now_get_version(&nowver);

    usb_stream_printf("\n");
    usb_stream_printf("=================================================\n");
    usb_stream_printf("  ESP32-S3-Dongle  ESP-NOW 无线麦克风接收端\n");
    usb_stream_printf("=================================================\n");
    usb_stream_printf("  本机 MAC   : %02X:%02X:%02X:%02X:%02X:%02X\n",
           s_self_mac[0], s_self_mac[1], s_self_mac[2],
           s_self_mac[3], s_self_mac[4], s_self_mac[5]);
    usb_stream_printf("  WiFi 信道  : %d  (必须和 espnow_duo 的 LINK_CHANNEL 一致)\n", LINK_CHANNEL);
    usb_stream_printf("  ESP-NOW    : v%" PRIu32 "%s\n", nowver,
           (nowver >= 2) ? "" : "  << 只有 v1! 1288 字节的音频包会被丢，需要 IDF>=5.4");
    usb_stream_printf("  音频格式   : %d Hz / %d 声道交织 / int16 小端 / 每包 %d 帧\n",
           LINK_SAMPLE_RATE, LINK_MIC_CHANNELS, LINK_FRAMES_PKT);
    usb_stream_printf("-------------------------------------------------\n");
    usb_stream_printf("  蓝灯: 慢闪=找不到发送端  常亮=已配对  快闪=正在推流\n");
    usb_stream_printf("  电脑端取流: python tools/recv_audio.py\n");
    usb_stream_printf("             (工具连上后会自动发 S 命令开始推流)\n");
    usb_stream_printf("=================================================\n\n");
}

/* ---------------- 主循环 ---------------- */

static void link_task(void *arg)
{
    (void)arg;
    print_banner();

    uint32_t tick     = 0;
    bool     was_lost = true;
    bool     btn_down = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        tick++;

        bool     pending_pair, paired;
        uint8_t  pending_mac[6];
        int64_t  last_rx_us;
        int      remote_level, remote_clip, remote_full, remote_band, rssi;
        bool     remote_has_mic;
        uint32_t rx_ok, rx_lost;

        portENTER_CRITICAL(&s_mux);
        pending_pair = s_pending_pair; s_pending_pair = false;
        memcpy(pending_mac, s_pending_mac, 6);
        paired         = s_paired;
        last_rx_us     = s_last_rx_us;
        remote_level   = s_remote_level;
        remote_clip    = s_remote_clip;
        remote_full    = s_remote_full;
        remote_band    = s_remote_band;
        remote_has_mic = s_remote_has_mic;
        rssi           = s_rssi;
        rx_ok          = s_rx_ok;
        rx_lost        = s_rx_lost;
        portEXIT_CRITICAL(&s_mux);

        bool quiet = usb_stream_is_streaming();   /* 推流期间串口上不能有文字 */

        /* ---- 配对 ---- */
        if (pending_pair && !paired) {
            esp_err_t err = add_unicast_peer(pending_mac);
            if (err == ESP_OK) {
                memcpy(s_peer_mac, pending_mac, 6);
                portENTER_CRITICAL(&s_mux);
                s_paired = true;
                portEXIT_CRITICAL(&s_mux);
                paired = true;
                if (!quiet) {
                    usb_stream_printf("\n>>> 配对成功! 麦克风端 %02X:%02X:%02X:%02X:%02X:%02X  RSSI %d dBm\n",
                           s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
                           s_peer_mac[3], s_peer_mac[4], s_peer_mac[5], rssi);
                    usb_stream_printf(">>> 对端马上开始发音频\n\n");
                }
                /* 配对本身就是「从无到有」，别让下面的失联判定再报一次「链路恢复」 */
                was_lost = false;
            } else {
                ESP_LOGE(TAG, "esp_now_add_peer 失败: %s", esp_err_to_name(err));
            }
        }

        int64_t now  = esp_timer_get_time();
        bool    lost = !paired || (now - last_rx_us > LOST_US);

        if (lost && !was_lost) {
            if (!quiet) {
                usb_stream_printf("\n!!! 麦克风端失联\n\n");
            }
            s_aud_seq_valid = false;   /* 重连后序号会跳，别把这一跳算成丢包 */
        } else if (!lost && was_lost && paired && !quiet) {
            usb_stream_printf("\n>>> 链路恢复\n\n");
        }
        was_lost = lost;

        /* ---- 心跳 ----
         * 这不只是保活：对端要收到它才认为链路在线、才肯开始发音频。
         * 停发超过 1.2 秒对端就停流了 */
        if (tick % TX_EVERY == 0) {
            link_pkt_t p = {
                .magic     = PKT_MAGIC,
                .ver       = PKT_VER,
                .seq       = s_tx_seq++,
                .level     = 0,        /* 本端不采声音 */
                .event     = 0,
                .flags     = 0,        /* 不置 PKT_FLAG_MIC */
                .rsv       = 0,
                .dbfs_full = -120,     /* 本端没麦克风，填静音底值而不是 0 —— */
                .dbfs_band = -120,     /* 0 dBFS 是满量程，会被误读成「震耳欲聋」 */
                .wake      = 0,
            };
            /* 配上了走单播（有 MAC 层 ACK）；没配上或失联了广播找人 */
            const uint8_t *dst = (paired && !lost) ? s_peer_mac : BROADCAST;
            esp_now_send(dst, (const uint8_t *)&p, sizeof(p));
        }

        /* ---- BOOT 键：不接电脑时也能确认固件活着 ---- */
        bool down = (gpio_get_level(BTN_GPIO) == 0);
        if (down && !btn_down && !quiet) {
            usb_stream_printf("\n[按键] 当前 %s，推流由电脑端工具控制\n\n",
                   usb_stream_is_streaming() ? "推流中" : "空闲");
        }
        btn_down = down;

        led_update(tick, paired, lost);

        /* ---- 串口状态行 ---- */
        if (!quiet && tick % PRINT_EVERY == 0) {
            if (!paired) {
                usb_stream_printf(" 正在广播寻找麦克风端... (信道 %d) —— 给那块 NodeMCU 上电，"
                       "或检查两边信道是否一致\n", LINK_CHANNEL);
            } else if (lost) {
                usb_stream_printf(" !! 已 %.1f 秒收不到麦克风端 (最后 RSSI %d dBm)，广播重连中...\n",
                       (double)(now - last_rx_us) / 1e6, rssi);
            } else {
                uint32_t total   = rx_ok + rx_lost;
                double   hb_rate = total ? (100.0 * rx_ok / total) : 0.0;

                /* 整行先拼进缓冲再一次性输出。分成多次调用的话，每次都要单独
                 * 抢一次输出锁，中间可能被别的任务插进来，一行就断成好几截 */
                char   line[256];
                size_t n = 0;
                n += snprintf(line + n, sizeof(line) - n,
                              " 音频 收%" PRIu32 " 空口丢%" PRIu32 " 非法%" PRIu32
                              " | USB 发%" PRIu32 " 满丢%" PRIu32
                              " | RSSI%4d 心跳%5.1f%%",
                              s_aud_rx, s_aud_lost, s_aud_bad,
                              usb_stream_frames(), usb_stream_drops(),
                              rssi, hb_rate);
                if (remote_has_mic && n < sizeof(line)) {
                    n += snprintf(line + n, sizeof(line) - n,
                                  "  远端 音量%d 全频%ddB 带内%ddB",
                                  remote_level, remote_full, remote_band);
                    if (remote_clip > 0 && n < sizeof(line)) {
                        n += snprintf(line + n, sizeof(line) - n,
                                      "  << 削顶%d%%! 降对端 PCM_GAIN_SHIFT", remote_clip);
                    }
                } else if (n < sizeof(line)) {
                    n += snprintf(line + n, sizeof(line) - n, "  << 远端没有麦克风!");
                }
                if (!usb_stream_is_streaming() && n < sizeof(line)) {
                    snprintf(line + n, sizeof(line) - n, "   [电脑端未取流]");
                }
                usb_stream_printf("%s\n", line);
            }
        }
    }
}

esp_err_t link_rx_start(void)
{
    ESP_ERROR_CHECK(esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA));

    io_init();

    esp_err_t err = wifi_init();
    if (err != ESP_OK) {
        return err;
    }
    err = espnow_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 优先级 4：低于 USB 推流任务(5)。钉在 core1，core0 留给 WiFi */
    if (xTaskCreatePinnedToCore(link_task, "link", 4096, NULL, 4, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
