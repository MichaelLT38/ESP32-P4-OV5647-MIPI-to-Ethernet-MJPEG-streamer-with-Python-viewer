/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Camera pipeline: MIPI CSI (RAW10) → ISP (RAW10 → RGB565) → hardware JPEG
 *                  (RGB565 in, 4:2:0 out) → Ethernet UDP (MJPEG)
 * No DSI display used. Target: 1920x1080 @ 30fps.
 *
 * Why RGB565 into the JPEG encoder (not YUV422 directly):
 *   The ESP32-P4 ISP emits YUV422 in UYVY byte order, while the JPEG encoder's
 *   YUV422 input expects YVYU — a byte-order mismatch whose reorder path is
 *   chip-revision dependent. ISP→RGB565→JPEG is the configuration Espressif's
 *   official OV5647 test verifies, and the JPEG OUTPUT is still 4:2:0 YUV, so
 *   compression and quality are unchanged. To switch to a YUV422 encoder input
 *   later, change ISP output to ISP_COLOR_YUV422 and the JPEG src_type to
 *   JPEG_ENCODE_IN_FORMAT_YUV422 (and verify colors on hardware).
 *
 * Architecture:
 *   - Two PSRAM DMA buffers (s_dbl.buf[0/1]): camera alternates between them
 *     (callback mode — the ISR re-arms the next buffer autonomously).
 *   - One PSRAM snapshot buffer (s_enc_src): copy of the latest completed frame,
 *     used as the JPEG encoder input so the camera can keep reusing its DMA
 *     buffers while we encode.
 *   - One PSRAM JPEG output buffer (s_jpeg_buf): holds the compressed bitstream.
 *   - Capture task (main): snapshots a completed frame into s_enc_src when the
 *     encode/send task is idle, drops the frame otherwise.
 *   - Encode/send task: JPEG-encodes s_enc_src, then streams the compressed
 *     bytes over UDP in 1400-byte chunks.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "driver/jpeg_encode.h"
#include "esp_ldo_regulator.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "example_sensor_init.h"
#include "example_config.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "cam_eth";

#define UDP_DEST_IP     "192.168.0.94"
#define UDP_DEST_PORT   12345
#define UDP_CHUNK_SIZE  1400   /* one Ethernet frame — avoids IP fragmentation */

#define JPEG_QUALITY    CONFIG_EXAMPLE_JPEG_QUALITY

/* Header prepended to every UDP datagram. frame_bytes is the size of the whole
 * compressed JPEG for this frame; the receiver reassembles total_chunks chunks
 * then decodes the JPEG. */
typedef struct __attribute__((packed)) {
    uint32_t frame_num;
    uint16_t chunk_idx;
    uint16_t total_chunks;
    uint32_t frame_bytes;
    uint32_t chunk_bytes;
} frame_chunk_hdr_t;

/* ------------------------------------------------------------------ */
/* Triple-buffer state — shared between ISR and capture task          */
/* ------------------------------------------------------------------ */
/* Three DMA buffers (instead of two) give the encoder a 2-frame safety
 * margin: by the time the camera round-robins back to a buffer, the JPEG
 * encode that read it has long finished. This eliminates the mid-frame
 * "tear" that two buffers produced once per-frame work exceeded one frame
 * period, and lets us encode straight from the DMA buffer (zero-copy). */
#define NUM_FRAME_BUFFERS 3
static struct {
    void   *buf[NUM_FRAME_BUFFERS]; /* PSRAM DMA buffers (RGB565 frames) */
    size_t  buflen;
    int     next_idx; /* which buf[] the ISR hands to the camera next */
} s_dbl;

/* ------------------------------------------------------------------ */
/* Encode/send-task shared state                                       */
/* ------------------------------------------------------------------ */
static void           *s_enc_ptr;      /* DMA buffer to encode (zero-copy)  */
static size_t          s_enc_len;      /* valid bytes in s_enc_ptr          */
static uint32_t        s_send_frame;
static jpeg_encoder_handle_t s_jpeg;   /* hardware JPEG encoder            */
static uint8_t        *s_jpeg_buf;     /* PSRAM compressed-output buffer   */
static size_t          s_jpeg_buf_cap; /* capacity of s_jpeg_buf           */
static SemaphoreHandle_t s_rdy;        /* capture task → encode/send task  */
static SemaphoreHandle_t s_done;       /* encode/send task → capture task  */
/* ISR → capture task: pointer to the latest completed DMA buffer    */
static QueueHandle_t      s_frame_q;

/* ------------------------------------------------------------------ */
/* Ethernet event group                                                */
/* ------------------------------------------------------------------ */
static EventGroupHandle_t s_eth_eg;
#define ETH_GOT_IP_BIT BIT0

/* ------------------------------------------------------------------ */
/* ISR callbacks (IRAM)                                                */
/* ------------------------------------------------------------------ */

/* Called by the ISR to get the buffer for the NEXT DMA transfer.
 * Round-robins through the three buffers so a buffer handed to the
 * encoder is not reused by the camera until two more frames have
 * elapsed (~44 ms @ 45fps) — far longer than a JPEG encode. */
static bool IRAM_ATTR s_camera_get_new_vb(esp_cam_ctlr_handle_t handle,
                                           esp_cam_ctlr_trans_t *trans,
                                           void *user_data)
{
    trans->buffer = s_dbl.buf[s_dbl.next_idx];
    trans->buflen = s_dbl.buflen;
    s_dbl.next_idx = (s_dbl.next_idx + 1) % NUM_FRAME_BUFFERS;
    return false;
}

/* Called by the ISR after the PREVIOUS DMA transfer completed.
 * trans->buffer is the buffer that just finished — safe to read.
 * The next DMA into a different buffer has already been armed above. */
static bool IRAM_ATTR s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle,
                                                   esp_cam_ctlr_trans_t *trans,
                                                   void *user_data)
{
    BaseType_t woken = pdFALSE;
    xQueueOverwriteFromISR(s_frame_q, &trans->buffer, &woken);
    return woken == pdTRUE;
}

/* ------------------------------------------------------------------ */
/* Ethernet                                                            */
/* ------------------------------------------------------------------ */
static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "Ethernet got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
    xEventGroupSetBits(s_eth_eg, ETH_GOT_IP_BIT);
}

static void init_ethernet(void)
{
    s_eth_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
    if (!eth_netif) { ESP_LOGE(TAG, "netif create failed"); abort(); }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               on_got_ip, NULL));

    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_mac_config_t        mac_cfg  = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t        phy_cfg  = ETH_PHY_DEFAULT_CONFIG();

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    /* Verify PHY for your Waveshare board — swap if needed:
     *   RTL8201 → esp_eth_phy_new_rtl8201()
     *   LAN8720 → esp_eth_phy_new_lan87xx()    */
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);
    if (!mac || !phy) { ESP_LOGE(TAG, "MAC/PHY create failed"); abort(); }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "Ethernet init — waiting for DHCP...");
    xEventGroupWaitBits(s_eth_eg, ETH_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Network ready");
}

/* ------------------------------------------------------------------ */
/* Encode + send task — runs at lower priority than capture loop      */
/* ------------------------------------------------------------------ */
static void encode_send_task(void *arg)
{
    struct sockaddr_in *dest = (struct sockaddr_in *)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "encode_send_task: socket failed (errno=%d)", errno);
        vTaskDelete(NULL);
        return;
    }

    /* Packet staging buffer in internal SRAM (lwIP can't DMA from PSRAM) */
    uint8_t *pkt = heap_caps_malloc(sizeof(frame_chunk_hdr_t) + UDP_CHUNK_SIZE,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!pkt) {
        ESP_LOGE(TAG, "encode_send_task: pkt alloc failed");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "encode_send_task ready, pkt buf @ %p", pkt);

    /* Signal capture task that we are ready */
    xSemaphoreGive(s_done);

    for (;;) {
        /* Wait for capture task to hand us a frame snapshot */
        xSemaphoreTake(s_rdy, portMAX_DELAY);

        uint32_t fn = s_send_frame;

        /* --- Hardware JPEG encode: RGB565 in → 4:2:0 JPEG out --- */
        jpeg_encode_cfg_t enc_cfg = {
            .src_type      = JPEG_ENCODE_IN_FORMAT_RGB565,
            .sub_sample    = JPEG_DOWN_SAMPLING_YUV420,
            .image_quality = JPEG_QUALITY,
            .width         = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES,
            .height        = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES,
        };
        uint32_t jpeg_size = 0;
        esp_err_t err = jpeg_encoder_process(s_jpeg, &enc_cfg,
                                             (const uint8_t *)s_enc_ptr, s_enc_len,
                                             s_jpeg_buf, s_jpeg_buf_cap, &jpeg_size);
        if (err != ESP_OK || jpeg_size == 0) {
            ESP_LOGW(TAG, "JPEG encode failed (frame %" PRIu32 ", err=%d)", fn, err);
            xSemaphoreGive(s_done);
            continue;
        }

        /* --- Stream the compressed JPEG over UDP in chunks --- */
        const uint8_t *src       = s_jpeg_buf;
        size_t         remaining = jpeg_size;
        uint16_t       total     = (uint16_t)((remaining + UDP_CHUNK_SIZE - 1) / UDP_CHUNK_SIZE);
        uint16_t       idx       = 0;

        while (remaining > 0) {
            size_t cb = remaining < UDP_CHUNK_SIZE ? remaining : UDP_CHUNK_SIZE;

            frame_chunk_hdr_t hdr = {
                .frame_num    = fn,
                .chunk_idx    = idx,
                .total_chunks = total,
                .frame_bytes  = jpeg_size,
                .chunk_bytes  = (uint32_t)cb,
            };
            memcpy(pkt,              &hdr, sizeof(hdr));
            memcpy(pkt + sizeof(hdr), src, cb);

            int sent = sendto(sock, pkt, sizeof(hdr) + cb, 0,
                              (const struct sockaddr *)dest, sizeof(*dest));
            if (sent < 0) {
                ESP_LOGW(TAG, "sendto errno=%d (frame %" PRIu32 " chunk %" PRIu16 ")",
                         errno, fn, idx);
            }

            if ((idx & 63) == 63) vTaskDelay(1);  /* yield to lwIP TX queue */
            src       += cb;
            remaining -= cb;
            idx++;
        }

        ESP_LOGI(TAG, "Frame %" PRIu32 ": JPEG %" PRIu32 " B (%d chunks)", fn, jpeg_size, total);
        xSemaphoreGive(s_done);   /* tell capture task we're done */
    }
}

/* ------------------------------------------------------------------ */
/* app_main — capture loop                                             */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    esp_err_t ret;

    init_ethernet();

    /* Frame buffers hold RGB565 (ISP output) = 2 bytes/pixel.
     * Three for triple-buffered CSI DMA; the JPEG encoder reads one of them
     * directly (zero-copy) — no separate snapshot buffer / memcpy needed. */
    size_t fb_size = (size_t)CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES
                   * (size_t)CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES
                   * 2;  /* RGB565 = 2 B/px */

    /* --- Hardware JPEG encoder engine (created before its input buffers) --- */
    jpeg_encode_engine_cfg_t jpeg_eng_cfg = {
        .intr_priority = 0,
        .timeout_ms    = 100,   /* ample for one 1080p frame at 30 fps */
    };
    ret = jpeg_new_encoder_engine(&jpeg_eng_cfg, &s_jpeg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encoder engine init failed [%d]", ret);
        return;
    }

    /* JPEG output buffer — half the raw size is far more than q80 1080p needs
     * (~200-400 KB), but generous keeps us safe against busy/noisy scenes. */
    jpeg_encode_memory_alloc_cfg_t enc_out_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t jpeg_out_alloc = 0;
    s_jpeg_buf     = jpeg_alloc_encoder_mem(fb_size / 2, &enc_out_cfg, &jpeg_out_alloc);
    s_jpeg_buf_cap = jpeg_out_alloc;

    bool dma_ok = true;
    for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
        s_dbl.buf[i] = heap_caps_aligned_alloc(64, fb_size, MALLOC_CAP_SPIRAM);
        if (!s_dbl.buf[i]) dma_ok = false;
    }
    if (!dma_ok || !s_jpeg_buf) {
        ESP_LOGE(TAG, "Frame buffer alloc failed (need %d × %zu B DMA + encoder mem)",
                 NUM_FRAME_BUFFERS, fb_size);
        return;
    }
    s_dbl.buflen   = fb_size;
    s_dbl.next_idx = 0;   /* on_get_new_vb gives buf[0] first, then round-robins */

    ESP_LOGI(TAG, "Buffers: DMA[0]=%p DMA[1]=%p DMA[2]=%p jpeg=%p  %zu B/frame  %dx%d RGB565",
             s_dbl.buf[0], s_dbl.buf[1], s_dbl.buf[2], s_jpeg_buf, fb_size,
             CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES, CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES);

    /* UDP destination */
    int sock_arp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    static struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_DEST_PORT),
        .sin_addr.s_addr = 0,
    };
    dest.sin_addr.s_addr = inet_addr(UDP_DEST_IP);

    /* ARP warm-up */
    sendto(sock_arp, "ping", 4, 0, (struct sockaddr *)&dest, sizeof(dest));
    close(sock_arp);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "ARP warm-up done");

    /* Semaphores + queue for capture ↔ send handshake */
    s_rdy     = xSemaphoreCreateBinary();
    s_done    = xSemaphoreCreateBinary();
    s_frame_q = xQueueCreate(1, sizeof(void *));  /* overwrite: always latest frame */
    if (!s_rdy || !s_done || !s_frame_q) {
        ESP_LOGE(TAG, "FreeRTOS object alloc failed");
        return;
    }

    /* Spawn encode+send task at lower priority (3 < 5 = main task prio) */
    xTaskCreate(encode_send_task, "enc_send", 4096, &dest, 3, NULL);

    /* MIPI PHY power: the CSI D-PHY is powered by an internal LDO channel.
     * Without this the sensor still answers on I2C (SCCB) but NO data ever
     * arrives over the CSI lanes — the DMA done-ISR never fires and every
     * frame times out. Channel 3 @ 2500 mV per the Espressif OV5647 example. */
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id    = 3,
        .voltage_mv = 2500,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy));
    ESP_LOGI(TAG, "MIPI PHY LDO acquired (chan 3, 2500 mV)");

    /* Camera sensor + SCCB */
    example_sensor_handle_t sensor_handle = { .sccb_handle = NULL, .i2c_bus_handle = NULL };
    example_sensor_config_t cam_sensor_config = {
        .i2c_port_num   = I2C_NUM_0,
        .i2c_sda_io_num = EXAMPLE_MIPI_CSI_CAM_SCCB_SDA_IO,
        .i2c_scl_io_num = EXAMPLE_MIPI_CSI_CAM_SCCB_SCL_IO,
        .port           = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name    = EXAMPLE_CAM_FORMAT,
    };
    example_sensor_init(&cam_sensor_config, &sensor_handle);

    /* ISP — must be created BEFORE CSI controller, enabled AFTER.
     * Demosaics the sensor RAW10 Bayer into packed RGB565 for the encoder. */
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = ISP_COLOR_RAW10,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet  = false,
        .has_line_end_packet    = false,
        .h_res                  = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES,
        .v_res                  = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));

    /* CSI controller — RAW10 passthrough (input == output).
     * The CSI bridge's own color conversion only works when src != dst AND on
     * ESP32-P4 chip rev >= v3.0; this chip is older, so the CSI must bypass and
     * let the ISP do the RAW10 -> RGB565 demosaic. The frame written to memory
     * is therefore RGB565 (matches the proven OV5647 test pattern, scaled from
     * its RAW8 case). */
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id                = 0,
        .h_res                  = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES,
        .v_res                  = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES,
        .lane_bit_rate_mbps     = EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW10,
        .output_data_color_type = CAM_CTLR_COLOR_RAW10,
        .data_lane_num          = 2,
        .byte_swap_en           = false,
        .queue_items            = 2,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "CSI init failed [%d]", ret); return; }

    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    /* Callback mode:
     *   on_get_new_trans  — ISR calls this to get the buffer for the NEXT DMA.
     *                       Alternates buf[0]/buf[1]; never touches trans_que.
     *   on_trans_finished — ISR calls this after the PREVIOUS DMA completes.
     *                       Pushes the finished buffer ptr to s_frame_q.
     * Do NOT call esp_cam_ctlr_receive() — that pushes to trans_que which the
     * ISR ignores in callback mode, filling it until error 263. */
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Callback register failed"); return;
    }

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

    /* start() calls on_get_new_trans to get the first buffer (buf[0]).
     * The ISR will call it again for buf[1] on the first DMA completion,
     * then alternate forever. No receive() needed. */
    if (esp_cam_ctlr_start(cam_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Camera start failed"); return;
    }
    ESP_LOGI(TAG, "Camera started — streaming");

    /* Capture loop — callback mode.
     * s_frame_q is length 1 (xQueueOverwrite): always holds the latest frame.
     * We wait for a notification, snapshot the finished buffer, then loop.
     * The ISR keeps DMA running continuously via on_get_new_trans. */
    uint32_t frame_num  = 0;
    uint32_t sent_count = 0;

    for (;;) {
        void *frame_buf = NULL;
        if (xQueueReceive(s_frame_q, &frame_buf, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGW(TAG, "No frame in 1 s — camera stalled?");
            continue;
        }

        if (xSemaphoreTake(s_done, 0) == pdTRUE) {
            /* Zero-copy: hand the finished DMA buffer straight to the encoder.
             * Safe because the camera round-robins through 3 buffers, so this
             * one is not reused until two more frames elapse — long after the
             * JPEG encode has read it. */
            s_enc_ptr     = frame_buf;
            s_enc_len     = s_dbl.buflen;
            s_send_frame  = frame_num;
            xSemaphoreGive(s_rdy);
            sent_count++;
            ESP_LOGI(TAG, "Frame %" PRIu32 " queued (sent so far: %" PRIu32 ")",
                     frame_num, sent_count);
        } else {
            ESP_LOGD(TAG, "Frame %" PRIu32 " dropped (send busy)", frame_num);
        }

        frame_num++;
    }
}
