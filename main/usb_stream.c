#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"

#include "usb_stream.h"

static const char *TAG = "usb";

/* 环形缓冲存放待发的音频包。50 包/秒、每包 1288 字节 → 32KB 约等于 500ms 的垫子。
 * 电脑端偶尔卡一下（比如脚本在写磁盘）能吸收掉，长时间不读就开始丢包并计数 */
#define RB_BYTES        (32 * 1024)

/* USB 驱动自己的发送缓冲。给到 8KB，单帧 1290 字节能一次塞进去不阻塞 */
#define USB_TX_BUF      8192
#define USB_RX_BUF      512

/* 单帧最长的等待时间。超过就说明电脑端真的不读了，认栽丢帧，
 * 而不是把推流任务一直堵在这儿 */
#define WRITE_TMO_MS    200

/* 入队时带上的小抬头，让推流任务知道这包的序号和长度。
 * 不直接复用 USB 帧头是因为 CRC 要在推流任务里算 —— 那是个纯计算，
 * 不该占用 WiFi 任务的时间 */
typedef struct {
    uint16_t seq;
    uint16_t bytes;
} rb_item_hdr_t;

static RingbufHandle_t s_rb;
static volatile bool   s_streaming;
static volatile bool   s_desync;      /* 上一帧只写出去一半，电脑端需要重新对齐 */
static uint32_t        s_drops;
static uint32_t        s_frames;

/* CDC 上只有一条字节流，谁都不能写到一半被别人插进来。
 * 竞争者有三个：推流任务写 PCM 帧、命令任务写 INFO 帧、link 任务打状态行。
 * 没有这把锁的话，开流那一瞬间 INFO 帧和第一个 PCM 帧就会交织在一起 */
static SemaphoreHandle_t s_wr_mux;

static uint32_t s_rate;
static uint8_t  s_ch;
static uint16_t s_frames_per_pkt;

/* CRC16-CCITT (0x1021)，初值 0xFFFF，不反射不异或。
 * 不建表：1280 字节算下来几十微秒，比多占 512 字节 IRAM 划算 */
static uint16_t crc16_ccitt(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 把 n 个字节整块写出去。
 *
 * usb_serial_jtag_write_bytes 在缓冲不够时会只写一部分，直接用它的返回值当
 * "写完了"会让帧从中间断开，电脑端收到的就是错位的 PCM。所以这里循环补写，
 * 补不完就置 s_desync，让下一帧之前先补发同步字。 */
static bool write_all(const uint8_t *p, size_t n)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(WRITE_TMO_MS);
    while (n > 0) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            return false;
        }
        int w = usb_serial_jtag_write_bytes(p, n, deadline - now);
        if (w <= 0) {
            return false;
        }
        p += w;
        n -= (size_t)w;
    }
    return true;
}

static bool send_frame(uint8_t type, uint8_t ch, uint16_t seq,
                       const void *payload, size_t len)
{
    usb_frame_hdr_t h = {
        .sync0 = USB_FRAME_SYNC0,
        .sync1 = USB_FRAME_SYNC1,
        .type  = type,
        .ch    = ch,
        .seq   = seq,
        .len   = (uint16_t)len,
        .crc   = crc16_ccitt(payload, len),
    };

    xSemaphoreTake(s_wr_mux, portMAX_DELAY);

    /* 上一帧写残了。先甩一串同步字过去，电脑端的重同步会快一点，
     * 不然它要一直啃到下一帧头才对得上 */
    if (s_desync) {
        s_desync = false;
        static const uint8_t pad[8] = {
            USB_FRAME_SYNC0, USB_FRAME_SYNC0, USB_FRAME_SYNC0, USB_FRAME_SYNC0,
            USB_FRAME_SYNC0, USB_FRAME_SYNC0, USB_FRAME_SYNC0, USB_FRAME_SYNC0,
        };
        write_all(pad, sizeof(pad));
    }

    bool ok = write_all((const uint8_t *)&h, sizeof(h)) && write_all(payload, len);
    if (ok) {
        s_frames++;
    } else {
        s_desync = true;
    }

    xSemaphoreGive(s_wr_mux);
    return ok;
}

void usb_stream_printf(const char *fmt, ...)
{
    if (s_streaming) {
        return;     /* 快速路径，绝大多数调用在推流时直接返回 */
    }
    xSemaphoreTake(s_wr_mux, portMAX_DELAY);
    /* 锁内复核：拿锁期间推流可能刚被打开，这时候一个字都不能再往外写 */
    if (!s_streaming) {
        va_list ap;
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
        fflush(stdout);
    }
    xSemaphoreGive(s_wr_mux);
}

/* ---------------- 推流任务 ---------------- */

static void usb_tx_task(void *arg)
{
    (void)arg;
    while (1) {
        size_t   got  = 0;
        uint8_t *item = xRingbufferReceive(s_rb, &got, portMAX_DELAY);
        if (item == NULL) {
            continue;
        }

        if (s_streaming && got > sizeof(rb_item_hdr_t)) {
            rb_item_hdr_t hdr;
            memcpy(&hdr, item, sizeof(hdr));
            send_frame(USB_FRAME_PCM, s_ch, hdr.seq,
                       item + sizeof(rb_item_hdr_t), hdr.bytes);
        }

        vRingbufferReturnItem(s_rb, item);
    }
}

/* ---------------- 命令任务 ---------------- */

static void start_stream(void)
{
    if (s_streaming) {
        return;
    }
    /* 日志和二进制帧混在同一条 CDC 上会互相打断，推流期间干脆闭嘴。
     * 停流时再放回来 */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* 缓冲里可能积着开流之前的陈数据，倒干净再开始，否则电脑端一上来
     * 就先听到一段几百毫秒前的旧声音 */
    size_t   got;
    uint8_t *item;
    while ((item = xRingbufferReceive(s_rb, &got, 0)) != NULL) {
        vRingbufferReturnItem(s_rb, item);
    }

    s_desync = false;

    /* INFO 必须是流上的第一帧，所以先发它、后置 s_streaming ——
     * 反过来的话推流任务可能抢先把一包 PCM 发出去，电脑端就要靠默认值猜格式 */
    usb_stream_info_t info = {
        .sample_rate    = s_rate,
        .channels       = s_ch,
        .bits           = 16,
        .frames_per_pkt = s_frames_per_pkt,
    };
    send_frame(USB_FRAME_INFO, 0, 0, &info, sizeof(info));

    s_streaming = true;
}

static void stop_stream(void)
{
    if (!s_streaming) {
        return;
    }
    s_streaming = false;
    esp_log_level_set("*", ESP_LOG_INFO);
    usb_stream_printf("\n>>> 推流已停止，恢复日志输出\n");
}

static void usb_cmd_task(void *arg)
{
    (void)arg;
    uint8_t c;
    while (1) {
        int n = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(500));
        if (n != 1) {
            /* 拔线了就把流关掉。否则重新插上时固件还以为在推流、
             * 日志一直是关的，看上去像死机 */
            if (s_streaming && !usb_serial_jtag_is_connected()) {
                s_streaming = false;
                esp_log_level_set("*", ESP_LOG_INFO);
            }
            continue;
        }
        switch (c) {
        case 'S': case 's': start_stream(); break;
        case 'X': case 'x': stop_stream();  break;
        case '?':
            usb_stream_printf("\n[状态] 推流=停止  已发帧=%" PRIu32
                              "  缓冲满丢包=%" PRIu32 "\n", s_frames, s_drops);
            break;
        default: break;   /* 别的字节忽略，避免误触发 */
        }
    }
}

/* ---------------- 对外接口 ---------------- */

bool usb_stream_is_streaming(void) { return s_streaming; }
uint32_t usb_stream_drops(void)    { return s_drops; }
uint32_t usb_stream_frames(void)   { return s_frames; }

bool usb_stream_send_pcm(uint16_t seq, const void *pcm, size_t bytes)
{
    if (!s_streaming) {
        return true;    /* 没在推流不算丢包 */
    }
    rb_item_hdr_t hdr = { .seq = seq, .bytes = (uint16_t)bytes };

    /* 一次性把抬头和数据放进同一个 item，保证边界不会被拆开。
     * NOSPLIT 类型的环形缓冲要求整块写入，正好符合"要么整包要么不要"的语义 */
    void *slot = NULL;
    if (xRingbufferSendAcquire(s_rb, &slot, sizeof(hdr) + bytes, 0) != pdTRUE) {
        s_drops++;
        return false;
    }
    memcpy(slot, &hdr, sizeof(hdr));
    memcpy((uint8_t *)slot + sizeof(hdr), pcm, bytes);
    xRingbufferSendComplete(s_rb, slot);
    return true;
}

esp_err_t usb_stream_init(uint32_t sample_rate, uint8_t channels, uint16_t frames_per_pkt)
{
    s_rate           = sample_rate;
    s_ch             = channels;
    s_frames_per_pkt = frames_per_pkt;

    s_wr_mux = xSemaphoreCreateMutex();
    if (s_wr_mux == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_rb = xRingbufferCreate(RB_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_rb == NULL) {
        ESP_LOGE(TAG, "环形缓冲分配失败");
        return ESP_ERR_NO_MEM;
    }

    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = USB_TX_BUF,
        .rx_buffer_size = USB_RX_BUF,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial/JTAG 驱动安装失败: %s", esp_err_to_name(err));
        return err;
    }
    /* 让 printf / ESP_LOG 也走这个驱动。不换的话它们直接怼寄存器，
     * 会插进正在写的帧中间 */
    usb_serial_jtag_vfs_use_driver();

    /* 优先级 5：要比 link 任务(4)高，USB 这一端不能成为瓶颈。
     * 钉在 core1，把 core0 完整留给 WiFi */
    if (xTaskCreatePinnedToCore(usb_tx_task, "usb_tx", 4096, NULL, 5, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(usb_cmd_task, "usb_cmd", 3072, NULL, 3, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
