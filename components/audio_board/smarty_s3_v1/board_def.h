/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2021 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _AUDIO_BOARD_DEFINITION_H_
#define _AUDIO_BOARD_DEFINITION_H_

/*
 * Smarty S3 v1 - custom ESP32-S3 PCB (module ESP32-S3-WROOM-1-N16R8).
 *
 * Mirrors the ESP-LyraT-Mini V1.1 audio topology on S3 silicon:
 *   - ES8311 playback codec on I2S port 0  (I2C 8-bit addr 0x30, driver default)
 *   - ES7243L mic ADC     on I2S port 1  (I2C 8-bit addr 0x20, driver default;
 *                                          register-compatible with the es7243e driver)
 * All I2S/SDMMC/I2C signals route through the S3 GPIO matrix (free pin choice).
 * See docs/s3-custom-board-pinmap.md for the authoritative, netlist-traced map.
 */

/**
 * @brief SDCARD Function Definition
 *
 * S3 SDMMC pins are assigned from these defines via the GPIO matrix (1-line only).
 * SDCARD_PWR_CTRL is an active-LOW P-FET gate with a 10k pull-up: the card is OFF
 * at boot and audio_board_sdcard_init() drives the pin LOW to power it on.
 */
#define FUNC_SDCARD_EN            (1)
#define SDCARD_OPEN_FILE_NUM_MAX  5
#define SDCARD_INTR_GPIO          GPIO_NUM_6
#define SDCARD_PWR_CTRL           GPIO_NUM_5

#define ESP_SD_PIN_CLK            GPIO_NUM_15
#define ESP_SD_PIN_CMD            GPIO_NUM_7
#define ESP_SD_PIN_D0             GPIO_NUM_4
#define ESP_SD_PIN_D1             -1
#define ESP_SD_PIN_D2             -1
#define ESP_SD_PIN_D3             -1
#define ESP_SD_PIN_D4             -1
#define ESP_SD_PIN_D5             -1
#define ESP_SD_PIN_D6             -1
#define ESP_SD_PIN_D7             -1
#define ESP_SD_PIN_CD             -1
#define ESP_SD_PIN_WP             -1


/**
 * @brief LED Function Definition
 *
 * LED hardware is undecided on this rev (see docs §6); both are -1. The app-side
 * board_hal already stubs the status LED on S3.
 */
#define FUNC_SYS_LEN_EN           (1)
#define GREEN_LED_GPIO            -1
#define BLUE_LED_GPIO             -1


/**
 * @brief Audio Codec Chip Function Definition
 *
 * Capture contract: 16 kHz / 16-bit, mono on the RIGHT slot (ES7243L AINR).
 * The LEFT channel (ES7243L AINL) carries a hardware AEC echo reference:
 * ES8311 OUT -> 10k/1.6k resistive pad (~-17 dB) -> AINL, sampled in the same
 * frame as the mic so it is time-aligned. Firmware captures ONLY_RIGHT today and
 * discards the echo-ref; stereo capture (ref=L, mic=R fed to the AFE as MR) comes
 * with the Phase-2 AFE. No hardware change is needed for AEC.
 *
 * ES8311_MCLK_SOURCE=0 -> ES8311 clocks from the MCLK pad (GPIO16), the
 * Korvo-2-validated configuration.
 */
#define FUNC_AUDIO_CODEC_EN       (1)
#define HEADPHONE_DETECT          GPIO_NUM_47
#define PA_ENABLE_GPIO            GPIO_NUM_48   /* NS4150 CTRL; board has external 10k pull-down */
#define ES8311_MCLK_SOURCE        (0)  /* 0 From MCLK of esp32   1 From BCLK */
#define CODEC_ADC_I2S_PORT        (1)
#define CODEC_ADC_BITS_PER_SAMPLE (16) /* 16bit */
#define CODEC_ADC_SAMPLE_RATE     (16000)
#define RECORD_HARDWARE_AEC       (true)
#define BOARD_PA_GAIN             (20) /* Power amplifier gain defined by board (dB) */

extern audio_hal_func_t AUDIO_CODEC_ES8311_DEFAULT_HANDLE;
extern audio_hal_func_t AUDIO_CODEC_ES7243E_DEFAULT_HANDLE;
#define AUDIO_CODEC_DEFAULT_CONFIG(){                   \
        .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,        \
        .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,         \
        .codec_mode = AUDIO_HAL_CODEC_MODE_BOTH,        \
        .i2s_iface = {                                  \
            .mode = AUDIO_HAL_MODE_SLAVE,               \
            .fmt = AUDIO_HAL_I2S_NORMAL,                \
            .samples = AUDIO_HAL_48K_SAMPLES,           \
            .bits = AUDIO_HAL_BIT_LENGTH_16BITS,        \
        },                                              \
};


/**
 * @brief Button Function Definition
 *
 * Resistor ladder on GPIO1 = ADC1_CHANNEL_0 (set explicitly in board.c). Same
 * ladder as LyraT-Mini, so the thresholds (btn_array in board.c) are unchanged.
 * COLOR is the VOL+ & VOL- chord and occupies the lowest rung (act_id 0); every
 * other button shifts up by one vs. the stock 6-button map. The user IDs below
 * are copied from lyrat_mini_v1_1 so the app's button_control.c is unchanged.
 * (Battery VBAT divider lives on GPIO2/ADC1_CH1 - no board-package code.)
 */
#define FUNC_BUTTON_EN            (1)
#define ADC_DETECT_GPIO           GPIO_NUM_1
#define INPUT_KEY_NUM             7
#define BUTTON_VOLUP_ID           1
#define BUTTON_COLOR_ID           0
#define BUTTON_VOLDOWN_ID         2
#define BUTTON_SET_ID             3
#define BUTTON_PLAY_ID            4
#define BUTTON_MODE_ID            5
#define BUTTON_REC_ID             6
#define INPUT_KEY_DEFAULT_INFO() {                      \
     {                                                  \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_REC,               \
        .act_id = BUTTON_REC_ID,                        \
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_MODE,              \
        .act_id = BUTTON_MODE_ID,                       \
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_SET,               \
        .act_id = BUTTON_SET_ID,                        \
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_PLAY,              \
        .act_id = BUTTON_PLAY_ID,                       \
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_VOLUP,             \
        .act_id = BUTTON_VOLUP_ID,                      \
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_VOLDOWN,           \
        .act_id = BUTTON_VOLDOWN_ID,                    \
    },                                                  \
    {                                                   \
        .type = PERIPH_ID_ADC_BTN,                      \
        .user_id = INPUT_KEY_USER_ID_COLOR,             \
        .act_id = BUTTON_COLOR_ID,                      \
    }                                                   \
}

#endif
