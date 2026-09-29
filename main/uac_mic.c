#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"
#include "usb_device_uac.h"

#include "uac_mic.h"

static const char *TAG = "uac";

/* 这三个值由 menuconfig 决定，组件内部也用同一套宏算 bytes_require，
 * 所以这里直接引用，不另写一份常量，免得两处对不上 */
#define SAMPLE_RATE   CONFIG_UAC_SAMPLE_RATE
#define CHANNELS      CONFIG_UAC_MIC_CHANNEL_NUM
#define INTERVAL_MS   CONFIG_UAC_MIC_INTERVAL_MS

#define BYTES_PER_MS  (SAMPLE_RATE * CHANNELS * (int)sizeof(int16_t) / 1000)
#define CHUNK_BYTES   (INTERVAL_MS * BYTES_PER_MS)   /* 主机每次要走的量 */

/* 水位线。缓冲总容量要明显大于高水位，否则刚追完延迟又立刻满 */
#define JITTER_MS     400
#define PREBUF_MS     100
#define HIGH_WATER_MS 250

/* 等一块数据的超时。给到 INTERVAL 的两倍多一点：
 * 太短会把正常的网络抖动误判成欠载，太长则链路断了之后 USB 那头会卡顿 */
#define WAIT_MS       (INTERVAL_MS * 2 + 5)

/* 两次取数间隔超过这么久，就认为电脑是「重新开始录音」。
 *
 * 正常录音时 10ms 取一次；停止录音后取数任务整个停下来，间隔是几秒到几小时。
 * 取 200ms 而不是更短：CPU 偶尔卡一下、USB 偶尔慢一帧都可能让间隔到几十毫秒，
 * 门限太低会在正常录音中途误清缓冲，凭空插进一段 100ms 的起播空白 */
#define SESSION_GAP_MS 200

static StreamBufferHandle_t s_pcm = NULL;

/* 下面两个标志都只由取数任务（唯一的读者）来执行清空。
 * s_prebuffering 只在取数任务里读写；s_reset_req 由别的任务置位、取数任务清零 */
static volatile bool        s_prebuffering = true;
static volatile bool        s_reset_req;

static volatile uint32_t s_underruns;
static volatile uint32_t s_drops;
static volatile uint32_t s_catchups;

/* 主机有没有在录音。组件没把 mic_active 暴露出来，所以用"最近一次被要数据
 * 是什么时候"来推断 —— 主机一停止录音，input_cb 就不再被调用了。
 * 初值 0 让第一次取数也被判成「新会话」，插上电脑后的第一次录音同样要清旧数据 */
static volatile int64_t s_last_pull_us;

/* 读空缓冲并重新进入预缓冲。【只能在取数任务里调】。
 *
 * FreeRTOS 流缓冲只保证「一个读者 + 一个写者」安全，它的读指针更新不是原子的。
 * 以前 uac_mic_flush() 在链路任务里直接读，和取数任务就成了两个读者。
 * 现在别的任务只置 s_reset_req，真正的读空都收到这里做。
 *
 * 128 字节一读不会破坏采样对齐：循环一直读到返回 0 为止，读完缓冲是空的，
 * 下一包写进来又是从整包边界开始 */
static void drain_and_rebuffer(void)
{
    uint8_t junk[128];
    while (xStreamBufferReceive(s_pcm, junk, sizeof(junk), 0) > 0) {
        /* 空转到读干净为止 */
    }
    s_prebuffering = true;
}

/* ---------------- UAC 取数回调 ---------------- */

/*
 * 组件的 usb_mic_task 是个没有 delay 的紧循环，节拍完全由这个回调自己把握：
 * 一旦无条件立刻返回，那个任务就会空转吃满一个核。所以下面每条路径都保证
 * 要么阻塞在 xStreamBufferReceive 上，要么显式 vTaskDelay。
 */
static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *ctx)
{
    (void)ctx;

    int64_t now     = esp_timer_get_time();
    bool    new_ses = (now - s_last_pull_us) > SESSION_GAP_MS * 1000LL;
    s_last_pull_us  = now;

    /* ---- 电脑刚(重新)开始录音，或者链路任务请求清空 ----
     *
     * 电脑停止录音后没人取数，ESP-NOW 却还在往里写：缓冲零点几秒就满，之后新包
     * 整包丢弃，留在里面的是「停止录音那一刻」的旧音频。不清的话，下一次开始录音
     * （可能是几小时后）开头交给电脑的就是这段旧声音 —— 对每说一句就开关一次
     * 麦克风的语音识别软件，等于每一句开头都拼上上一句的尾巴。
     *
     * 先清标志再读空：清完之后才到的请求，对应的数据也会被这次读空一并带走 */
    if (new_ses || s_reset_req) {
        s_reset_req = false;
        drain_and_rebuffer();
    }

    /* ---- 预缓冲：只等，不消费 ---- */
    if (s_prebuffering) {
        if (xStreamBufferBytesAvailable(s_pcm) >= (size_t)(PREBUF_MS * BYTES_PER_MS)) {
            s_prebuffering = false;
            ESP_LOGI(TAG, "缓冲已攒够 %d ms，开始向电脑交付音频", PREBUF_MS);
        } else {
            vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));
            memset(buf, 0, len);
            *bytes_read = len;
            return ESP_OK;
        }
    }

    /* ---- 高水位：发送端偏快或刚从卡顿里恢复，丢一块把延迟追回来 ----
     * 丢的是最老的一块。整块丢而不是丢几个字节：错开半个采样点会让
     * 左右声道从此对调，比丢 10ms 严重得多 */
    if (xStreamBufferBytesAvailable(s_pcm) > (size_t)(HIGH_WATER_MS * BYTES_PER_MS)) {
        uint8_t junk[CHUNK_BYTES];
        xStreamBufferReceive(s_pcm, junk, sizeof(junk), 0);
        s_catchups++;
    }

    /* ---- 正常取数。触发水位已设成整块，所以这里要么等到一整块，要么超时 ---- */
    size_t got = xStreamBufferReceive(s_pcm, buf, len, pdMS_TO_TICKS(WAIT_MS));

    if (got < len) {
        /* 欠载：补静音而不是让流水线停住。USB 主机的时间轴不会为我们暂停，
         * 少交的这一段在录音里就是一个空洞，补零至少能保持时长对齐 */
        memset(buf + got, 0, len - got);
        s_underruns++;

        /* 并且重新攒够 PREBUF_MS 再继续交付。
         *
         * 缓冲见底说明抗抖动的垫子已经用完了。如果只补这 10ms 就接着交，缓冲会
         * 一直贴着 0 运行，之后 ESP-NOW 每晚到一次都直接变成一次断续，而且永远
         * 恢复不过来。重新起播的代价是这一次多空 100ms，换来之后又有完整的垫子。
         *
         * 丢包已经在 link_rx.c 里按序号补了静音，不会再慢慢吃掉垫子；
         * 所以正常情况下这里很少触发，触发了多半是两边时钟漂移积累到头，
         * 或者空口断流超过了 200ms 的补位上限 */
        s_prebuffering = true;
    }

    *bytes_read = len;
    return ESP_OK;
}

/* ---------------- 对外接口 ---------------- */

bool uac_mic_push(const void *pcm, size_t bytes)
{
    if (s_pcm == NULL) {
        return false;
    }
    /* 要么整包写进去，要么整包丢掉。xStreamBufferSend 在空间不够时会写入
     * "部分"字节，那会让取数端从此错开半个采样点，整条音频变成噪声 */
    if (xStreamBufferSpacesAvailable(s_pcm) < bytes) {
        s_drops++;
        return false;
    }
    xStreamBufferSend(s_pcm, pcm, bytes, 0);
    return true;
}

void uac_mic_flush(void)
{
    /* 这里只置标志，不直接读：调用方是链路任务 / WiFi 任务，而取数任务同时也在读
     * 这个流缓冲，两个读者会把读指针写乱。交给取数任务下次进来时自己清。
     *
     * 电脑没在录音时取数任务不跑，标志会一直留着 —— 没关系，等电脑开始录音时
     * 本来就会按「新会话」清一遍 */
    s_reset_req = true;
}

bool uac_mic_host_active(void)
{
    /* 主机停止录音后 input_cb 不再被调用，超过 3 个周期没被要数据就算停了 */
    return (esp_timer_get_time() - s_last_pull_us) < (INTERVAL_MS * 3 * 1000LL);
}

uint32_t uac_mic_underruns(void) { return s_underruns; }
uint32_t uac_mic_drops(void)     { return s_drops; }
uint32_t uac_mic_catchups(void)  { return s_catchups; }

int uac_mic_depth_ms(void)
{
    if (s_pcm == NULL) {
        return 0;
    }
    return (int)(xStreamBufferBytesAvailable(s_pcm) / BYTES_PER_MS);
}

esp_err_t uac_mic_init(void)
{
    s_pcm = xStreamBufferCreate((size_t)JITTER_MS * BYTES_PER_MS, CHUNK_BYTES);
    if (s_pcm == NULL) {
        ESP_LOGE(TAG, "抖动缓冲分配失败");
        return ESP_ERR_NO_MEM;
    }
    /* 触发水位 = 一整块。默认是 1，那样 xStreamBufferReceive 只要有 1 个字节
     * 就会立刻返回，欠载计数会被没攒够的半块刷爆 */
    xStreamBufferSetTriggerLevel(s_pcm, CHUNK_BYTES);

    uac_device_config_t cfg = {
        .skip_tinyusb_init = false,   /* 让组件自己把 PHY 从 Serial/JTAG 切到 OTG */
        .output_cb         = NULL,    /* 不做扬声器，只做麦克风 */
        .input_cb          = uac_input_cb,
        .set_mute_cb       = NULL,
        .set_volume_cb     = NULL,
        .cb_ctx            = NULL,
    };
    esp_err_t err = uac_device_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UAC 设备初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UAC 麦克风就绪: %d Hz / %d 声道 / 16bit，主机每次取 %d ms (%d 字节)",
             SAMPLE_RATE, CHANNELS, INTERVAL_MS, CHUNK_BYTES);
    ESP_LOGI(TAG, "抖动缓冲 %d ms，起播水位 %d ms，高水位 %d ms",
             JITTER_MS, PREBUF_MS, HIGH_WATER_MS);
    return ESP_OK;
}
