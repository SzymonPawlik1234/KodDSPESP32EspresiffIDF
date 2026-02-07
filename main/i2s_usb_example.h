#pragma once
#include "driver/gpio.h"

/* I2S DMA configuration */
#define EXAMPLE_I2S_DMA_DESC_NUM        16
#define EXAMPLE_I2S_DMA_FRAME_NUM       256

/* I2S port and GPIOs (STAŁE pod PCM5102A) */
#define EXAMPLE_I2S_NUM                 (0)
#define EXAMPLE_I2S_MCK_IO    10          GPIO_NUM_0
#define I2S_BCK_PIN           5    // Bit Clock (BCLK)
#define I2S_WS_PIN            6   // Word Select / LRC
#define I2S_DO_PIN            8    // Data Out (DIN dla DAC)
#define EXAMPLE_I2S_DI_IO               GPIO_NUM_NC

/* Audio configuration */
#define EXAMPLE_AUDIO_SAMPLE_RATE       (48000)
#define EXAMPLE_AUDIO_BIT_WIDTH         I2S_DATA_BIT_WIDTH_16BIT
