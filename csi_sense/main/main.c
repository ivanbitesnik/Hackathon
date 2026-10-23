/*
 * Wi-Fi CSI sensing, milestone 0+1.
 *
 * Same firmware on both boards; the role is picked at boot:
 * the role is saved in flash, and holding KEY_4 while resetting switches it:
 *   - TRANSMITTER: broadcasts ESP-NOW packets at TX_RATE_HZ
 *   - RECEIVER (default): measures CSI of those packets and streams
 *                                   one line per packet over USB:
 *                                   CSI,<time_ms>,<rssi>,<amp0> <amp1> ... <ampN>
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "display.h"

static const char *TAG = "csi";

#define WIFI_CHANNEL     6
#define TX_RATE_HZ       100
#define PIN_KEY_ROLE     GPIO_NUM_48  /* KEY_4 */
#define MAX_SUBCARRIERS  64
#define FRAME_QUEUE_LEN  64

/* The transmitter uses a fixed MAC so the receiver can ignore every other packet on the channel */
static const uint8_t TX_MAC[6] = {0x1a, 0x00, 0x00, 0xc5, 0x1c, 0x01};
static const uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

typedef struct {
    uint32_t time_ms;
    int8_t rssi;
    uint8_t count;
    uint8_t amplitude[MAX_SUBCARRIERS];
} csi_frame_t;

static QueueHandle_t s_frames;
static volatile uint32_t s_dropped;
static volatile uint32_t s_csi_total;    /* CSI callbacks from any sender */
static volatile uint32_t s_csi_matched;  /* ... of which came from the transmitter */
static uint8_t s_last_mac[6];             /* sender of the latest CSI callback, for debugging */

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/* The role is stored in flash so resets (including the one idf.py monitor does) keep it.
 * Holding KEY_4 at boot flips it. */
static bool load_role_is_transmitter(void)
{
    gpio_config_t key = {
        .pin_bit_mask = BIT64(PIN_KEY_ROLE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&key));
    vTaskDelay(pdMS_TO_TICKS(10));  /* let the pull-up settle */
    bool flip = gpio_get_level(PIN_KEY_ROLE) == 0;

    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("csi", NVS_READWRITE, &nvs));
    uint8_t transmitter = 0;
    nvs_get_u8(nvs, "transmitter", &transmitter);  /* missing key -> receiver */
    if (flip) {
        transmitter = !transmitter;
        ESP_ERROR_CHECK(nvs_set_u8(nvs, "transmitter", transmitter));
        ESP_ERROR_CHECK(nvs_commit(nvs));
    }
    nvs_close(nvs);
    return transmitter;
}

static void wifi_init(bool transmitter)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (transmitter) {
        ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, TX_MAC));
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Power save would make the radio sleep between beacons and drop packets */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void run_transmitter(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {
        .channel = WIFI_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* CSI is measured on the OFDM preamble, so send 802.11n frames instead of the 802.11b default */
    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_HT20,
        .rate = WIFI_PHY_RATE_MCS0_LGI,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(BROADCAST_MAC, &rate));

    ESP_LOGI(TAG, "TRANSMITTER: %d packets/s on channel %d", TX_RATE_HZ, WIFI_CHANNEL);

    display_clear();
    display_text(10, 8, "TRANSMITTER", false);
    display_flush();

    uint32_t seq = 0;
    uint32_t failed = 0;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        /* A failed send is just one missing CSI sample; only count it */
        if (esp_now_send(BROADCAST_MAC, (const uint8_t *)&seq, sizeof(seq)) != ESP_OK) {
            failed++;
        }
        seq++;
        if (seq % TX_RATE_HZ == 0) {
            ESP_LOGI(TAG, "TRANSMITTER status: %" PRIu32 " sent, %" PRIu32 " failed", seq, failed);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / TX_RATE_HZ));
    }
}

/* Runs in the Wi-Fi task: convert to amplitudes and hand off, never block here */
static void csi_callback(void *ctx, wifi_csi_info_t *info)
{
    s_csi_total++;
    memcpy(s_last_mac, info->mac, sizeof(s_last_mac));
    if (memcmp(info->mac, TX_MAC, sizeof(TX_MAC)) != 0) {
        return;
    }
    s_csi_matched++;

    csi_frame_t frame = {
        .time_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .rssi = info->rx_ctrl.rssi,
    };
    /* buf holds one (imaginary, real) int8 pair per subcarrier */
    size_t count = info->len / 2;
    if (count > MAX_SUBCARRIERS) {
        count = MAX_SUBCARRIERS;
    }
    for (size_t i = 0; i < count; i++) {
        int im = info->buf[2 * i];
        int re = info->buf[2 * i + 1];
        frame.amplitude[i] = (uint8_t)sqrtf((float)(im * im + re * re));
    }
    frame.count = count;

    if (xQueueSend(s_frames, &frame, 0) != pdTRUE) {
        s_dropped++;
    }
}

static void run_receiver(void)
{
    s_frames = xQueueCreate(FRAME_QUEUE_LEN, sizeof(csi_frame_t));
    assert(s_frames);

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    wifi_csi_config_t csi_cfg = {
        .lltf_en = true,
        .htltf_en = false,
        .stbc_htltf2_en = false,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "RECEIVER: listening on channel %d", WIFI_CHANNEL);

    display_clear();
    display_text(10, 8, "RECEIVER", false);
    display_flush();

    csi_frame_t frame;
    char line[32 + MAX_SUBCARRIERS * 4 + 2];
    int64_t next_status_us = 0;
    while (true) {
        if (esp_timer_get_time() >= next_status_us) {
            next_status_us = esp_timer_get_time() + 1000000;
            ESP_LOGI(TAG, "RECEIVER status: %" PRIu32 " CSI callbacks, %" PRIu32 " from transmitter, %" PRIu32
                     " dropped (serial too slow), last sender " MACSTR,
                     s_csi_total, s_csi_matched, s_dropped, MAC2STR(s_last_mac));
        }
        if (xQueueReceive(s_frames, &frame, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        int len = snprintf(line, sizeof(line), "CSI,%" PRIu32 ",%d,", frame.time_ms, frame.rssi);
        for (int i = 0; i < frame.count; i++) {
            len += snprintf(line + len, sizeof(line) - len, i ? " %u" : "%u", frame.amplitude[i]);
        }
        line[len++] = '\n';
        fwrite(line, 1, len, stdout);
    }
}

void app_main(void)
{
    nvs_init();
    bool transmitter = load_role_is_transmitter();
    ESP_LOGI(TAG, "ROLE: %s (hold KEY_4 while resetting to switch)", transmitter ? "TRANSMITTER" : "RECEIVER");

    bool display_ok = display_init() == ESP_OK;
    if (!display_ok) {
        ESP_LOGE(TAG, "display init failed - continuing without screen");
    }

    if (display_ok) {
        display_clear();
        display_text(10, 8, "Buscando pword", false);
        display_text(10, 24, "Buscando pword", false);
        display_text(10, 40, "Buscando pword", false);
        display_flush();
        sleep(5);
    }

    wifi_init(transmitter);
    if (transmitter) {
        run_transmitter();
    } else {
        run_receiver();
    }
}
