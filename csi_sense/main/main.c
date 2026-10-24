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
#define CALIBRATION_DURATION_MS 10000

/* The transmitter uses a fixed MAC so the receiver can ignore every other packet on the channel */
static const uint8_t TX_MAC[6] = {0x1a, 0x00, 0x00, 0xc5, 0x1c, 0x01};
static const uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

typedef struct {
    uint32_t time_ms;
    int8_t rssi;    // Signal strength indicator
    uint8_t count;
    uint8_t amplitude[MAX_SUBCARRIERS];
} csi_frame_t;

static QueueHandle_t s_frames;
static volatile uint32_t s_dropped = 0;
static volatile uint32_t s_csi_total = 0;    /* CSI callbacks from any sender */
static volatile uint32_t s_csi_matched = 0;  /* ... of which came from the transmitter */
static uint8_t s_last_mac[6];             /* sender of the latest CSI callback, for debugging */

typedef enum {
    CALIBRATING_IDLE,
    CALIBRATING_WALK,
    MONITORING,
} receiver_state_t;

static receiver_state_t s_receiver_state = CALIBRATING_IDLE;
static uint32_t s_calibration_start_ms;
static uint32_t s_previous_packet_time_ms;
static uint32_t s_idle_samples;
static uint32_t s_walk_samples;
static double s_average_idle_interval_ms;
static double s_average_idle_rssi;
static double s_average_walk_interval_ms;
static double s_average_walk_rssi;
static double s_average_idle_amplitude[MAX_SUBCARRIERS];
static double s_average_walk_amplitude[MAX_SUBCARRIERS];
static uint32_t s_idle_amplitude_samples[MAX_SUBCARRIERS];
static uint32_t s_walk_amplitude_samples[MAX_SUBCARRIERS];

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

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        /* A failed send is just one missing CSI sample; only count it */
        if (esp_now_send(BROADCAST_MAC, (const uint8_t *)&s_csi_total, sizeof(s_csi_total)) != ESP_OK) {
            s_dropped++;
        }
        s_csi_total++;
        if (s_csi_total % TX_RATE_HZ == 0) {
            ESP_LOGI(TAG, "TRANSMITTER status: %" PRIu32 " sent, %" PRIu32 " failed", s_csi_total, s_dropped);
        }

        display_clear();
        display_text(30, 5, "TRANSMITTER", false);
        display_text(5, 18, "Check receiver for instructions!", false);

        char dropped_str[10];
        snprintf(dropped_str, 30, "Dropped packets: %ld", s_dropped);
        display_text(5, 30, dropped_str, false);
        char total_str[10];
        snprintf(total_str, 30, "Total packets: %ld", s_dropped);
        display_text(5, 42, total_str, false);
        display_flush();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / TX_RATE_HZ));  // Send signal every 10ms
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

static void update_average_amplitudes(const csi_frame_t *frame, double *averages,
                                      uint32_t *sample_counts)
{
    for (size_t i = 0; i < frame->count; i++) {
        sample_counts[i]++;
        averages[i] += (frame->amplitude[i] - averages[i]) / sample_counts[i];
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

    csi_frame_t frame;
    char line[64 + MAX_SUBCARRIERS * 16];
    int64_t next_status_us = 0;
    bool have_previous_packet = false;
    bool calibration_started = false;
    while (true) {
        int64_t time_microseconds = esp_timer_get_time();
        if (time_microseconds >= next_status_us) {
            next_status_us = time_microseconds + 1000000;
            ESP_LOGI(TAG, "RECEIVER status: %" PRIu32 " CSI callbacks, %" PRIu32 " from transmitter, %" PRIu32
                     " dropped (serial too slow), last sender " MACSTR,
                     s_csi_total, s_csi_matched, s_dropped, MAC2STR(s_last_mac));
        }

        if (xQueueReceive(s_frames, &frame, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        if (!calibration_started) {
            s_calibration_start_ms = frame.time_ms;
            calibration_started = true;
        }

        bool has_interval = have_previous_packet;
        uint32_t packet_interval_ms = has_interval ? frame.time_ms - s_previous_packet_time_ms : 0;
        s_previous_packet_time_ms = frame.time_ms;
        have_previous_packet = true;

        uint32_t calibration_elapsed_ms = frame.time_ms - s_calibration_start_ms;
        if (s_receiver_state == CALIBRATING_IDLE &&
            calibration_elapsed_ms >= CALIBRATION_DURATION_MS)
        {
            s_receiver_state = CALIBRATING_WALK;
            s_calibration_start_ms = frame.time_ms;
            calibration_elapsed_ms = 0;
        }
        else if (s_receiver_state == CALIBRATING_WALK &&
                   calibration_elapsed_ms >= CALIBRATION_DURATION_MS)
        {
            s_receiver_state = MONITORING;
        }

        display_clear();
        display_text(5, 5, "RECEIVER", false);
        if (s_receiver_state == CALIBRATING_IDLE ||
            s_receiver_state == CALIBRATING_WALK) {
            uint32_t remaining_ms = CALIBRATION_DURATION_MS - calibration_elapsed_ms;
            char status[32];
            if (s_receiver_state == CALIBRATING_IDLE)
            {
                display_text(5, 18, "Please keep still for baseline", false);
                s_idle_samples++;
                s_average_idle_interval_ms += (packet_interval_ms - s_average_idle_interval_ms) / s_idle_samples;
                s_average_idle_rssi += (frame.rssi - s_average_idle_rssi) / s_idle_samples;
                update_average_amplitudes(&frame, s_average_idle_amplitude,
                                          s_idle_amplitude_samples);
            }
            else
            {
                display_text(5, 18, "Please walk around for motion baseline", false);
                s_walk_samples++;
                s_average_walk_interval_ms += (packet_interval_ms - s_average_walk_interval_ms) / s_walk_samples;
                s_average_walk_rssi += (frame.rssi - s_average_walk_rssi) / s_walk_samples;
                update_average_amplitudes(&frame, s_average_walk_amplitude,
                                          s_walk_amplitude_samples);
            }
            snprintf(status, sizeof(status), "Remaining: %" PRIu32 " s", (remaining_ms + 999) / 1000);
            display_text(5, 30, status, false);
        }
        else
        {
            int len = snprintf(line, sizeof(line), "CSI,%" PRIu32 ",%d,", frame.time_ms, frame.rssi);
            for (int i = 0; i < frame.count; i++) {
                int written = snprintf(line + len, sizeof(line) - (size_t)len,
                                       i ? " %u" : "%u", frame.amplitude[i]);
                if (written < 0 || (size_t)written >= sizeof(line) - (size_t)len) {
                    len = -1;
                    break;
                }
                len += written;
            }
            for (int i = 0; len >= 0 && i < frame.count; i++) {
                int written = snprintf(line + len, sizeof(line) - (size_t)len,
                                       i ? " %.1f" : ",%.1f",
                                       s_average_idle_amplitude[i]);
                if (written < 0 || (size_t)written >= sizeof(line) - (size_t)len) {
                    len = -1;
                    break;
                }
                len += written;
            }
            for (int i = 0; len >= 0 && i < frame.count; i++) {
                int written = snprintf(line + len, sizeof(line) - (size_t)len,
                                       i ? " %.1f" : ",%.1f",
                                       s_average_walk_amplitude[i]);
                if (written < 0 || (size_t)written >= sizeof(line) - (size_t)len) {
                    len = -1;
                    break;
                }
                len += written;
            }
            if (len < 0 || (size_t)len + 1 >= sizeof(line)) {
                ESP_LOGE(TAG, "CSI output line exceeds buffer");
                continue;
            }
            line[len++] = '\n';
            if (fwrite(line, 1, (size_t)len, stdout) != (size_t)len) {
                ESP_LOGE(TAG, "Failed to write CSI frame to stdout");
            }
        }
        
        display_flush();
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
        display_text(10, 8, "Awaiting connection...", false);
        display_flush();
    }

    wifi_init(transmitter);
    if (transmitter) {
        run_transmitter();
    } else {
        run_receiver();
    }
}
