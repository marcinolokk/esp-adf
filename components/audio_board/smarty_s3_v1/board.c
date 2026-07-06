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

#include "esp_log.h"
#include "driver/gpio.h"
#include "board.h"
#include "audio_mem.h"
#include "periph_sdcard.h"
#include "periph_adc_button.h"
#include "i2c_bus.h"

static const char *TAG = "AUDIO_BOARD";

/* ES7243x mic ADC lives on the shared I2C0 bus. 8-bit address 0x20 is the
 * es7243e driver default (7-bit 0x10, this board straps AD0=AD1=GND); the
 * fork's i2c_bus API takes 8-bit addresses. */
#define MIC_ADC_I2C_ADDR   (0x20)

static audio_board_handle_t board_handle = 0;

/*
 * Non-fatal chip-identity probe for the mic ADC. ES7243L and ES7243E share the
 * es7243e register map and I2C address; they differ only in the chip-ID regs
 * 0xFD/0xFE (0x72/0x43 = ES7243L, 0x7A = ES7243E). Log which silicon is fitted
 * so field units can be told apart. Runs AFTER es7243e init has opened I2C0;
 * i2c_bus_create is ref-counted, so this reuses the codecs' bus handle rather
 * than reinstalling the driver, and i2c_bus_delete just drops our extra ref.
 */
static void mic_adc_probe_chip_id(void)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    if (get_i2c_pins(I2C_NUM_0, &i2c_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "mic ADC chip-ID probe skipped: no I2C pins");
        return;
    }
    i2c_bus_handle_t i2c_handle = i2c_bus_create(I2C_NUM_0, &i2c_cfg);
    if (i2c_handle == NULL) {
        ESP_LOGW(TAG, "mic ADC chip-ID probe skipped: i2c_bus_create failed");
        return;
    }

    uint8_t reg = 0xFD, id_fd = 0, id_fe = 0;
    esp_err_t r1 = i2c_bus_read_bytes(i2c_handle, MIC_ADC_I2C_ADDR, &reg, sizeof(reg), &id_fd, sizeof(id_fd));
    reg = 0xFE;
    esp_err_t r2 = i2c_bus_read_bytes(i2c_handle, MIC_ADC_I2C_ADDR, &reg, sizeof(reg), &id_fe, sizeof(id_fe));

    if (r1 != ESP_OK || r2 != ESP_OK) {
        ESP_LOGW(TAG, "mic ADC chip-ID probe: I2C read failed (0xFD:%s 0xFE:%s)",
                 esp_err_to_name(r1), esp_err_to_name(r2));
    } else if (id_fd == 0x72 && id_fe == 0x43) {
        ESP_LOGI(TAG, "mic ADC chip ID 0x72: ES7243L (expected, es7243e-driver compatible)");
    } else if (id_fd == 0x7A) {
        ESP_LOGW(TAG, "mic ADC is ES7243E silicon (chip ID 0x%02X/0x%02X)", id_fd, id_fe);
    } else {
        ESP_LOGW(TAG, "mic ADC unknown chip ID 0x%02X/0x%02X (expected ES7243L 0x72/0x43)", id_fd, id_fe);
    }

    i2c_bus_delete(i2c_handle);
}

audio_board_handle_t audio_board_init(void)
{
    if (board_handle) {
        ESP_LOGW(TAG, "The board has already been initialized!");
        return board_handle;
    }
    board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);
    board_handle->audio_hal = audio_board_codec_init();
    board_handle->adc_hal = audio_board_adc_init();
    mic_adc_probe_chip_id();
    return board_handle;
}

audio_hal_handle_t audio_board_adc_init(void)
{
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_codec_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
    audio_hal_handle_t adc_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES7243E_DEFAULT_HANDLE);
    AUDIO_NULL_CHECK(TAG, adc_hal, return NULL);
    return adc_hal;
}

audio_hal_handle_t audio_board_codec_init(void)
{
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_hal_handle_t codec_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8311_DEFAULT_HANDLE);
    AUDIO_NULL_CHECK(TAG, codec_hal, return NULL);
    return codec_hal;
}

display_service_handle_t audio_board_led_init(void)
{
    // No status LED on this board rev (GREEN_LED_GPIO == -1); see docs §6.
    return NULL;
}

display_service_handle_t audio_board_blue_led_init(void)
{
    // No status LED on this board rev (BLUE_LED_GPIO == -1); see docs §6.
    return NULL;
}

esp_err_t audio_board_key_init(esp_periph_set_handle_t set)
{
    periph_adc_button_cfg_t adc_btn_cfg = PERIPH_ADC_BUTTON_DEFAULT_CONFIG();
    adc_arr_t adc_btn_tag = ADC_DEFAULT_ARR();
    adc_btn_tag.total_steps = 7;
    // Button ladder is on GPIO1 = ADC1_CHANNEL_0. Set the channel EXPLICITLY:
    // LyraT-Mini silently inherits the periph default channel, which is a trap on
    // a board whose ladder pin differs. (Korvo-2 sets its channel explicitly too.)
    adc_btn_tag.adc_ch = ADC1_CHANNEL_0;
    // Same ladder resistors as LyraT-Mini, so the same windows apply: COLOR is the
    // VOL+ & VOL- chord (~282 mV, rung 0), then VOL+ (~380), VOL- (~819) ... REC
    // (~2408). Must match the BUTTON_*_ID order in board_def.h (COLOR=0, ...).
    int btn_array[8] = {100, 300, 600, 1000, 1375, 1775, 2050, 2900};
    adc_btn_tag.adc_level_step = btn_array;
    adc_btn_cfg.arr = &adc_btn_tag;
    adc_btn_cfg.arr_size = 1;
    if (audio_mem_spiram_stack_is_enabled()) {
        adc_btn_cfg.task_cfg.ext_stack = true;
    }
    esp_periph_handle_t adc_btn_handle = periph_adc_button_init(&adc_btn_cfg);
    AUDIO_NULL_CHECK(TAG, adc_btn_handle, return ESP_ERR_ADF_MEMORY_LACK);
    return esp_periph_start(set, adc_btn_handle);
}

esp_err_t audio_board_sdcard_init(esp_periph_set_handle_t set, periph_sdcard_mode_t mode)
{
    if (mode != SD_MODE_1_LINE) {
        ESP_LOGE(TAG, "Current board only support 1-line SD mode!");
        return ESP_FAIL;
    }
    // SDCARD_PWR_CTRL is an active-LOW P-FET gate (10k pull-up -> card off at
    // boot). Drive it LOW to power the card, exactly as LyraT-Mini does.
    gpio_config_t sdcard_pwr_pin_cfg = {
        .pin_bit_mask = 1UL << SDCARD_PWR_CTRL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&sdcard_pwr_pin_cfg);
    gpio_set_level(SDCARD_PWR_CTRL, 0);

    periph_sdcard_cfg_t sdcard_cfg = {
        .root = "/sdcard",
        .card_detect_pin = get_sdcard_intr_gpio(),
        .mode = mode
    };
    esp_periph_handle_t sdcard_handle = periph_sdcard_init(&sdcard_cfg);
    esp_err_t ret = esp_periph_start(set, sdcard_handle);
    int retry_time = 5;
    bool mount_flag = false;
    while (retry_time --) {
        if (periph_sdcard_is_mounted(sdcard_handle)) {
            mount_flag = true;
            break;
        } else {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    if (mount_flag == false) {
        ESP_LOGE(TAG, "Sdcard mount failed");
        return ESP_FAIL;
    }
    return ret;
}

audio_board_handle_t audio_board_get_handle(void)
{
    return board_handle;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board)
{
    esp_err_t ret = ESP_OK;
    ret |= audio_hal_deinit(audio_board->audio_hal);
    ret |= audio_hal_deinit(audio_board->adc_hal);
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}
