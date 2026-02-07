/*
 * SPDX-FileCopyrightText: 2026
 * Ultra-Low Jitter USB-I2S Bridge + DSP UART Control
 * Professional Grade - Sync Fix & Real-time EQ
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "usb_device_uac.h"
#include "esp_attr.h"

// --- KONFIGURACJA PINÓW ---
#define I2S_BCK_PIN           5
#define I2S_WS_PIN            6
#define I2S_DO_PIN            8
#define I2S_MCLK_PIN          10

#define UART_PORT_NUM         UART_NUM_2
#define UART_TX_PIN           1
#define UART_RX_PIN           2
#define UART_BAUD_RATE        115200

// --- AUDIO CONFIG ---
#define SAMPLE_RATE           48000
#define RING_BUFFER_SIZE      (64 * 2048) 
#define DMA_BUFFER_SIZE       2048        // Rozmiar bloku przetwarzania (bajty)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Funkcja pomocnicza clamp
static inline int clamp_int(int v, int min, int max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

// --- STRUKTURY DSP ---
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1L, x2L, y1L, y2L;
    float x1R, x2R, y1R, y2R;
} Biquad;

typedef struct {
    float F, G, Q;
} FilterState;

static const char *TAG = "AUDIO_DSP_PRO";
static RingbufHandle_t audio_ring_buf = NULL;
static i2s_chan_handle_t tx_handle = NULL;
static bool sync_established = false;

// Zmienne DSP
static Biquad eq[10];
static FilterState currentFilters[10];
static volatile bool isDSPOn = false;
static volatile float volume_factor = 1.0f;
static SemaphoreHandle_t xDspMutex = NULL;

// --- IMPLEMENTACJA DSP ---
static void biquadInit(Biquad *f, float freq, float Q, float gainDB, float fs) {
    if (fs == 0) return;
    double A = pow(10.0, (double)gainDB / 40.0);
    double w0 = 2.0 * M_PI * (double)freq / (double)fs;
    double sinw0 = sin(w0);
    double cosw0 = cos(w0);
    double alpha = sinw0 / (2.0 * (double)Q);

    double b0 = 1.0 + alpha * A;
    double b1 = -2.0 * cosw0;
    double b2 = 1.0 - alpha * A;
    double a0 = 1.0 + alpha / A;
    double a1 = -2.0 * cosw0;
    double a2 = 1.0 - alpha / A;

    f->b0 = (float)(b0 / a0); f->b1 = (float)(b1 / a0); f->b2 = (float)(b2 / a0);
    f->a1 = (float)(a1 / a0); f->a2 = (float)(a2 / a0);

    f->x1L = f->x2L = f->y1L = f->y2L = 0.0f;
    f->x1R = f->x2R = f->y1R = f->y2R = 0.0f;
}

static inline float IRAM_ATTR biquadProcessSampleL(Biquad *f, float in) {
    float out = f->b0 * in + f->b1 * f->x1L + f->b2 * f->x2L - f->a1 * f->y1L - f->a2 * f->y2L;
    if (fabsf(out) < 1e-15f) out = 0.0f;
    f->x2L = f->x1L; f->x1L = in; f->y2L = f->y1L; f->y1L = out;
    return out;
}

static inline float IRAM_ATTR biquadProcessSampleR(Biquad *f, float in) {
    float out = f->b0 * in + f->b1 * f->x1R + f->b2 * f->x2R - f->a1 * f->y1R - f->a2 * f->y2R;
    if (fabsf(out) < 1e-15f) out = 0.0f;
    f->x2R = f->x1R; f->x1R = in; f->y2R = f->y1R; f->y1R = out;
    return out;
}

// --- RESET SYNCHRONIZACJI ---
static void IRAM_ATTR reset_audio_sync() {
    if (tx_handle) i2s_channel_disable(tx_handle);
    size_t dummy;
    if (audio_ring_buf) xRingbufferReceiveUpTo(audio_ring_buf, &dummy, 0, RING_BUFFER_SIZE);
    sync_established = false;
    if (tx_handle) i2s_channel_enable(tx_handle);
}

// --- TASK AUDIO (Rdzeń 1) ---
static void IRAM_ATTR audio_processor_task(void *arg) {
    size_t item_size;
    size_t written;

    while (1) {
        if (!sync_established) {
            size_t free_size;
            vRingbufferGetInfo(audio_ring_buf, NULL, NULL, NULL, NULL, &free_size);
            if ((RING_BUFFER_SIZE - free_size) > (RING_BUFFER_SIZE / 2)) {
                sync_established = true;
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
        }

        int16_t *data = (int16_t *)xRingbufferReceiveUpTo(audio_ring_buf, &item_size, pdMS_TO_TICKS(10), DMA_BUFFER_SIZE);
        
        if (data != NULL) {
            size_t sample_count = item_size / sizeof(int16_t);
            float vol = volume_factor;
            bool dsp_on = isDSPOn;

            for (size_t i = 0; i < sample_count; i += 2) {
                float l = (float)data[i];
                float r = (float)data[i+1];

                if (dsp_on) {
                    for (int f = 0; f < 10; f++) {
                        if (currentFilters[f].F > 0) {
                            l = biquadProcessSampleL(&eq[f], l);
                            r = biquadProcessSampleR(&eq[f], r);
                        }
                    }
                }

                l *= vol; r *= vol;

                // Clipping i konwersja
                data[i]   = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, l));
                data[i+1] = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, r));
            }

            esp_err_t err = i2s_channel_write(tx_handle, data, item_size, &written, pdMS_TO_TICKS(10));
            if (err != ESP_OK) reset_audio_sync();
            
            vRingbufferReturnItem(audio_ring_buf, data);
        } else {
            reset_audio_sync();
        }
    }
}

// --- MODYFIKACJA OBSŁUGI UART ---
static void parse_uart_command(char *data) {
    if (strncmp(data, "VOL", 3) == 0) {
        int vol = atoi(data + 3);
        volume_factor = (float)clamp_int(vol, 0, 100) / 100.0f;
        ESP_LOGI(TAG, "Volume set: %.2f", volume_factor);
    } 
    else if (strncmp(data, "DSPON", 5) == 0) {
        if (xSemaphoreTake(xDspMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            char *ptr = data + 5;
            
            // 1. Najpierw oznaczamy wszystkie filtry jako nieaktywne (F=0)
            // Dzięki temu filtry, których nie ma w nowej ramce, zostaną wyłączone
            for (int i = 0; i < 10; i++) {
                currentFilters[i].F = 0.0f; 
            }

            // 2. Szukamy filtrów w otrzymanym ciągu
            for (int i = 0; i < 10; i++) {
                char keyF[8], keyG[8], keyQ[8];
                sprintf(keyF, "F%d", i); 
                sprintf(keyG, "G%d", i); 
                sprintf(keyQ, "Q%d", i);
                
                char *pF = strstr(ptr, keyF);
                if (pF) {
                    // Wyciągamy wartości tylko jeśli klucz F(x) istnieje
                    float freq = strtof(pF + strlen(keyF), NULL);
                    float gain = 0.0f;
                    float q_factor = 0.707f; // domyślny Q

                    char *pG = strstr(ptr, keyG);
                    if (pG) gain = strtof(pG + strlen(keyG), NULL);
                    
                    char *pQ = strstr(ptr, keyQ);
                    if (pQ) q_factor = strtof(pQ + strlen(keyQ), NULL);

                    // Zapisujemy i inicjujemy filtr
                    currentFilters[i].F = freq;
                    currentFilters[i].G = gain;
                    currentFilters[i].Q = q_factor;
                    
                    if (freq > 0) {
                        biquadInit(&eq[i], freq, q_factor, gain, (float)SAMPLE_RATE);
                    }
                }
            }
            isDSPOn = true;
            xSemaphoreGive(xDspMutex);
            ESP_LOGI(TAG, "DSP Updated: Active filters applied, missing ones disabled.");
        }
    } 
    else if (strncmp(data, "DSPOFF", 6) == 0) {
        isDSPOn = false;
        // Opcjonalnie czyścimy stany filtrów przy wyłączeniu
        for (int i = 0; i < 10; i++) currentFilters[i].F = 0;
        ESP_LOGI(TAG, "DSP OFF");
    }
}

static void uart_task(void *arg) {
    uart_config_t cfg = { .baud_rate = UART_BAUD_RATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &cfg);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1);
    uint8_t *data_buf = malloc(1024);
    char line[256]; int pos = 0;
    while(1) {
        int len = uart_read_bytes(UART_PORT_NUM, data_buf, 1023, pdMS_TO_TICKS(20));
        if (len > 0) {
            for(int i=0; i<len; i++) {
                if (data_buf[i] == '\n' || data_buf[i] == '\r') {
                    if (pos > 0) { line[pos] = 0; parse_uart_command(line); pos = 0; }
                } else if (pos < 255) line[pos++] = data_buf[i];
            }
        }
    }
}

// --- USB CALLBACK ---
static esp_err_t IRAM_ATTR uac_output_cb(uint8_t *buf, size_t len, void *arg) {
    if (!audio_ring_buf) return ESP_OK;
    if (xRingbufferSend(audio_ring_buf, buf, len, 0) != pdTRUE) {
        // Overflow - resetujemy, by nie było metalicznego dźwięku
        sync_established = false; 
    }
    return ESP_OK;
}

void app_main(void) {
    xDspMutex = xSemaphoreCreateMutex();
    audio_ring_buf = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 32; 
    chan_cfg.dma_frame_num = 512; 
    chan_cfg.auto_clear = true;
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = I2S_MCLK_PIN, .bclk = I2S_BCK_PIN, .ws = I2S_WS_PIN, .dout = I2S_DO_PIN }
    };
    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    i2s_channel_enable(tx_handle);

    xTaskCreatePinnedToCore(audio_processor_task, "audio_srv", 8192, NULL, 24, NULL, 1);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, NULL, 5, NULL, 0);

    uac_device_config_t u_cfg = { .output_cb = uac_output_cb };
    uac_device_init(&u_cfg);
}