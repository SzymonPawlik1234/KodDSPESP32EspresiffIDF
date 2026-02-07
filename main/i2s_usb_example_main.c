/*
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_system.h"
#include "esp_log.h"
#include "usb_device_uac.h"

// =========================
// Konfiguracja I2S i audio
// =========================
#define I2S_BCK_PIN           5
#define I2S_WS_PIN            6
#define I2S_DO_PIN            8
#define I2S_MCLK_PIN          10


// Audio format
#define I2S_SAMPLE_RATE       48000   // 48 kHz – standard USB Audio
#define I2S_BITS_PER_SAMPLE   16      // 16-bit
#define I2S_CHANNELS          2       // Stereo

// Buforowanie DMA
#define I2S_DMA_DESC_NUM      16
#define I2S_DMA_FRAME_NUM     256

// Kolejka audio
#define AUDIO_QUEUE_LENGTH    16
#define AUDIO_PACKET_SIZE     192    

static const char *TAG = "usb_i2s_bridge";

static i2s_chan_handle_t i2s_tx_handle = NULL;
static QueueHandle_t audio_queue = NULL;

/* ---------- I2S init ---------- */
static void i2s_driver_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_BITS_PER_SAMPLE, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,         
            .bclk = I2S_BCK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DO_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_handle));
}

/* ---------- USB callback ---------- */
static esp_err_t usb_uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    if (xQueueSend(audio_queue, buf, 0) != pdTRUE) {
        // kolejka pełna -> drop
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ---------- Audio task ---------- */
static void audio_task(void *arg)
{
    uint8_t packet[AUDIO_PACKET_SIZE];
    size_t written = 0;

    while (1) {
        if (xQueueReceive(audio_queue, packet, portMAX_DELAY) == pdTRUE) {
            i2s_channel_write(i2s_tx_handle, packet, AUDIO_PACKET_SIZE, &written, portMAX_DELAY);
        }
    }
}

/* ---------- USB init ---------- */
static void usb_uac_device_init(void)
{
    uac_device_config_t config = {
        .output_cb = usb_uac_device_output_cb,
        .input_cb = NULL,
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uac_device_init(&config));
}

/* ---------- MAIN ---------- */
void app_main(void)
{
    i2s_driver_init();
    usb_uac_device_init();

    audio_queue = xQueueCreate(AUDIO_QUEUE_LENGTH, AUDIO_PACKET_SIZE);
    xTaskCreatePinnedToCore(audio_task, "audio_task", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "USB -> I2S bridge (PCM5102AAA) started");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
