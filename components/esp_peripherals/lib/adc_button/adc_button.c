/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
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

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "math.h"
#include "audio_mem.h"
#include "string.h"
#include "adc_button.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "audio_thread.h"
#include "audio_idf_version.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/*
 * ADC access (Smarty fork, 2026-11): ported from the legacy driver (driver/adc.h +
 * esp_adc_cal) to adc_oneshot + adc_cali, which may not coexist with the legacy driver in
 * one firmware. Each sample comes from the application's read_mv callback when one is
 * given (it then owns ADC1 and shares it, e.g. with a battery monitor), otherwise from a
 * private oneshot unit opened by the button task (stock boards and examples). Sampling
 * (10 samples, trimmed mean of the middle 8), attenuation (12 dB, the same setting the
 * legacy ADC_ATTEN_DB_11 alias selected), bitwidth and calibration precedence (ESP32 line
 * fitting: eFuse Two-Point -> eFuse Vref -> V_REF) match the legacy path, so the mV values
 * and therefore the step tables are unchanged.
 */
#define V_REF                           1100   /* ESP32 line fitting: Vref when no eFuse value */
#define ADC_BTN_ATTEN                   ADC_ATTEN_DB_12
#define ADC_BTN_READ_RETRIES            3      /* adc_oneshot_read() try-locks ADC1 */
#define ADC_BTN_MIN_GOOD_SAMPLES        3

#define ADC_SAMPLES_NUM                 10
#define ADC_SAMPLE_INTERVAL_TIME_MS     20
#define DIAL_VOL_INTERVAL_TIME_MS       150

#define ADC_BTN_INVALID_ID              -1
#define ADC_BTN_INVALID_ACT_ID          -2
#define ADC_BTN_DETECT_TIME_MS          20
#define ADC_BTN_DETECTED_CNT            2

#ifndef ENABLE_ADC_VOLUME
#define USER_KEY_MAX                    7
#endif

static char *TAG = "ADC_BTN";
static EventGroupHandle_t g_event_bit;

typedef struct {
    adc_button_callback btn_callback;
    adc_btn_list *head;
    void *user_data;
    audio_thread_t audio_thread;
} adc_btn_tag_t;

static const int default_step_level[USER_KEY_MAX] = {0, 683, 1193, 1631, 2090, 2578, 3103};
static const int DESTROY_BIT = BIT0;
static bool _task_flag;

static adc_btn_read_mv_fn        s_read_mv;    /* application reader, or NULL */
static void                     *s_read_ctx;
static adc_oneshot_unit_handle_t s_unit;       /* private unit, fallback path only */
static adc_cali_handle_t         s_cali[SOC_ADC_MAX_CHANNEL_NUM];
static bool                      s_cali_tried[SOC_ADC_MAX_CHANNEL_NUM];
static int                       s_last_mv = -1;
static uint32_t                  s_read_failures;

adc_btn_list *adc_btn_create_list(adc_arr_t *adc_conf, int channels)
{
    adc_btn_list *head = NULL;
    adc_btn_list *node = NULL;
    adc_btn_list *find = NULL;
    for (int i = 0; i < channels; i++) {
        node = (adc_btn_list *)audio_calloc(1, sizeof(adc_btn_list));
        if (NULL == node) {
            ESP_LOGE(TAG, "Memory allocation failed! Line: %d", __LINE__);
            return NULL;
        }
        memset(node, 0, sizeof(adc_btn_list));
        adc_arr_t *info = &(node->adc_info);
        memcpy(info, adc_conf + i, sizeof(adc_arr_t));
        info->adc_level_step = (int *)audio_calloc(1, (info->total_steps + 1) * sizeof(int));
        memset(info->adc_level_step, 0, (info->total_steps + 1) * sizeof(int));
        if (NULL == info->adc_level_step) {
            ESP_LOGE(TAG, "Memory allocation failed! Line: %d", __LINE__);
            audio_free(node);
            return NULL;
        }
        if (adc_conf[i].adc_level_step == NULL) {
            memcpy(info->adc_level_step, default_step_level, USER_KEY_MAX * sizeof(int));
        } else {
            memcpy(info->adc_level_step, adc_conf[i].adc_level_step, (adc_conf[i].total_steps + 1) * sizeof(int));
        }
        if (info->total_steps > USER_KEY_MAX) {
            ESP_LOGE(TAG, "The total_steps should be less than USER_KEY_MAX");
            audio_free(info->adc_level_step);
            audio_free(node);
        }
        node->btn_dscp = (btn_decription *)audio_calloc(1, sizeof(btn_decription) * (adc_conf[i].total_steps));
        if (NULL == node->btn_dscp) {
            ESP_LOGE(TAG, "Memory allocation failed! Line: %d", __LINE__);
            audio_free(info->adc_level_step);
            audio_free(node);
        }
        memset(node->btn_dscp, 0, sizeof(btn_decription) * (adc_conf[i].total_steps));
        node->next = NULL;
        if (NULL == head) {
            head = node;
            find = head;
        } else {
            find->next = node;
            find = node;
        }
    }
    return head;
}

esp_err_t adc_btn_destroy_list(adc_btn_list *head)
{
    if (head == NULL) {
        ESP_LOGD(TAG, "The head of list is null");
        return ESP_OK;
    }
    adc_btn_list *find = head;
    adc_btn_list *tmp = find;

    while (find) {
        adc_arr_t *info = &(find->adc_info);
        tmp = find->next;
        audio_free(find->btn_dscp);
        audio_free(info->adc_level_step);
        audio_free(find);
        find = tmp;
    }
    return ESP_OK;
}

/* ---- Fallback path: private ADC1 oneshot unit (used only when no read_mv is given) ---- */

static void fallback_cali_create(int channel)
{
    if (channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM || s_cali_tried[channel]) {
        return;
    }
    s_cali_tried[channel] = true;
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = ADC_BTN_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cfg, &s_cali[channel]);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_BTN_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
#if CONFIG_IDF_TARGET_ESP32
        .default_vref = V_REF,
#endif
    };
    ret = adc_cali_create_scheme_line_fitting(&cfg, &s_cali[channel]);
#endif
    if (ret != ESP_OK) {
        s_cali[channel] = NULL;
        ESP_LOGW(TAG, "ADC1 ch%d: no calibration (%s), using the uncalibrated approximation",
                 channel, esp_err_to_name(ret));
    }
}

static void fallback_cali_delete(int channel)
{
    if (s_cali[channel] == NULL) {
        return;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(s_cali[channel]);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(s_cali[channel]);
#endif
    s_cali[channel] = NULL;
}

static void fallback_channel_init(int channel)
{
    if (s_unit == NULL) {
        return;
    }
    adc_oneshot_chan_cfg_t cfg = {
        .atten = ADC_BTN_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_oneshot_config_channel(s_unit, channel, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 ch%d config failed: %s", channel, esp_err_to_name(ret));
        return;
    }
    fallback_cali_create(channel);
}

static void fallback_unit_open(void)
{
    adc_oneshot_unit_init_cfg_t cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&cfg, &s_unit);
    if (ret != ESP_OK) {
        s_unit = NULL;
        ESP_LOGE(TAG, "ADC1 open failed (%s): buttons are inactive. If the application "
                 "already owns ADC1, pass it in via periph_adc_button_cfg_t.read_mv",
                 esp_err_to_name(ret));
    }
}

static void fallback_unit_close(void)
{
    for (int ch = 0; ch < SOC_ADC_MAX_CHANNEL_NUM; ch++) {
        fallback_cali_delete(ch);
        s_cali_tried[ch] = false;
    }
    if (s_unit) {
        adc_oneshot_del_unit(s_unit);
        s_unit = NULL;
    }
}

static int fallback_read_mv(int channel)
{
    if (s_unit == NULL || channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM) {
        return -1;
    }
    int raw = 0;
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < ADC_BTN_READ_RETRIES; attempt++) {
        ret = adc_oneshot_read(s_unit, channel, &raw);
        if (ret != ESP_ERR_TIMEOUT) {
            break;
        }
        esp_rom_delay_us(20);  /* another task holds ADC1 for one conversion */
    }
    if (ret != ESP_OK) {
        return -1;
    }
    if (s_cali[channel]) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali[channel], raw, &mv) == ESP_OK) {
            return mv;
        }
        return -1;
    }
    return raw * 3100 / ((1 << SOC_ADC_RTC_MAX_BITWIDTH) - 1);
}

/* One reading = ADC_SAMPLES_NUM samples, sorted, mean of all but the lowest and highest. */
static int get_adc_voltage(int channel)
{
    uint32_t data[ADC_SAMPLES_NUM] = { 0 };
    uint32_t sum = 0;
    int tmp = 0;
    int n = 0;

    for (int i = 0; i < ADC_SAMPLES_NUM; ++i) {
        int mv = s_read_mv ? s_read_mv(channel, s_read_ctx) : fallback_read_mv(channel);
        if (mv >= 0) {
            data[n++] = (uint32_t)mv;
        }
    }
    if (n < ADC_BTN_MIN_GOOD_SAMPLES) {
        /* Keep the previous reading so a read glitch never looks like a key release. */
        if ((s_read_failures++ % 256) == 0) {
            ESP_LOGW(TAG, "ADC1 ch%d: only %d/%d samples read (failures: %" PRIu32 ")",
                     channel, n, ADC_SAMPLES_NUM, s_read_failures);
        }
        return s_last_mv;
    }
    for (int j = 0; j < n - 1; j++) {
        for (int i = 0; i < n - j - 1; i++) {
            if (data[i] > data[i + 1]) {
                tmp = data[i];
                data[i] = data[i + 1];
                data[i + 1] = tmp;
            }
        }
    }
    for (int num = 1; num < n - 1; num++)
        sum += data[num];
    s_last_mv = sum / (n - 2);
    return s_last_mv;
}

static int get_button_id(adc_btn_list *node, int adc)
{
    int m = ADC_BTN_INVALID_ID;
    adc_arr_t *info = &(node->adc_info);
    for (int i = 0; i < info->total_steps; i++) {
        ESP_LOGV(TAG, "max:%d, adc:%d, i:%d, %d, %d", info->total_steps, adc, i, info->adc_level_step[i], info->adc_level_step[i + 1]);
        if ((adc > info->adc_level_step[i])
            && (adc <= info->adc_level_step[i + 1])) {
            m = i;
            break;
        }
    }
    return m;
}

static void reset_btn(btn_decription *btn_dscp, int btn_num)
{
    memset(btn_dscp, 0, sizeof(btn_decription) * btn_num);
    for (int i = 0; i < btn_num; ++i) {
        btn_dscp[i].active_id = ADC_BTN_INVALID_ID;
    }
}

static adc_btn_state_t get_adc_btn_state(int adc_value, int act_id, adc_btn_list *node)
{
    adc_btn_state_t st = ADC_BTN_STATE_IDLE;
    adc_arr_t *info = &(node->adc_info);
    btn_decription *btn_dscp = node->btn_dscp;
    int id = get_button_id(node, adc_value);
    if (id == ADC_BTN_INVALID_ID) {
        if (act_id == ADC_BTN_INVALID_ACT_ID) {
            // No old act id and new act id.
            return ADC_BTN_STATE_IDLE;
        }
        if (btn_dscp[act_id].click_cnt <= 1) {
            return ADC_BTN_STATE_IDLE;
        }
        // Have old act ID, new id is invalid
        // Need to send release event
        if (btn_dscp[act_id].click_cnt < (info->press_judge_time / ADC_BTN_DETECT_TIME_MS)) {
            ESP_LOGD(TAG, "pressed: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
            st = ADC_BTN_STATE_RELEASE;
        } else {
            ESP_LOGD(TAG, "long press release: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
            st = ADC_BTN_STATE_LONG_RELEASE;
        }
        btn_dscp[act_id].active_id = -1;
        btn_dscp[act_id].long_click = 0;
        btn_dscp[act_id].click_cnt = 0;
        return st;
    }
    // 1.ID is valid and act ID is invalid.
    if (act_id == ADC_BTN_INVALID_ACT_ID) {
        // First new act id
        btn_dscp[id].active_id = id;
        return ADC_BTN_STATE_IDLE;
    }
    // 2.ID and act ID are valid, but not equal.
    if (id != act_id) {
        ESP_LOGW(TAG, "Old ID:%d, New ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
        // Invalid the act ID
        btn_dscp[act_id].active_id = -1;
        btn_dscp[act_id].long_click = 0;
        // Set the new id act ID
        btn_dscp[id].active_id = id;
        // Maybe need to check release long pressed.
        if (btn_dscp[act_id].click_cnt < ADC_BTN_DETECTED_CNT) {
            btn_dscp[act_id].click_cnt = 0;
            return ADC_BTN_STATE_IDLE;
        }
        btn_dscp[act_id].click_cnt = 0;
        // Have old act ID, new id is invalid
        // Need to send release event
        if (btn_dscp[act_id].click_cnt < (info->press_judge_time / ADC_BTN_DETECT_TIME_MS)) {
            ESP_LOGD(TAG, "pressed: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
            return ADC_BTN_STATE_RELEASE;
        } else {
            ESP_LOGD(TAG, "long press release: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
            return ADC_BTN_STATE_LONG_RELEASE;
        }
    }
    // 3.ID and act ID are valid, and equal.
    btn_dscp[act_id].click_cnt++;
    if (btn_dscp[act_id].click_cnt == ADC_BTN_DETECTED_CNT) {
        return ADC_BTN_STATE_PRESSED;
    }

    if (btn_dscp[act_id].long_click) {
        return ADC_BTN_STATE_IDLE;
    }
    if (btn_dscp[act_id].click_cnt >= (info->press_judge_time / ADC_BTN_DETECT_TIME_MS)) {
        //Send long click event.
        ESP_LOGD(TAG, "long press: Act ID:%d, ID:%d, Cnt:%d", act_id, id, btn_dscp[act_id].click_cnt);
        st = ADC_BTN_STATE_LONG_PRESSED;
        btn_dscp[act_id].long_click = 1;
    }
    return st;
}

static void button_task(void *parameters)
{
    _task_flag = true;
    adc_btn_tag_t *tag = (adc_btn_tag_t *)parameters;
    adc_btn_list *head = tag->head;
    adc_btn_list *find = head;
    xEventGroupClearBits(g_event_bit, DESTROY_BIT);
    if (s_read_mv) {
        ESP_LOGI(TAG, "ADC1 via application reader");
    } else {
        fallback_unit_open();
    }

    while (find) {
        adc_arr_t *info = &(find->adc_info);
        reset_btn(find->btn_dscp, info->total_steps);
        if (!s_read_mv) {
            fallback_channel_init(info->adc_ch);
        }
        find = find->next;
    }
    find = head;

#if defined ENABLE_ADC_VOLUME
    if (!s_read_mv) {
        fallback_channel_init(DIAL_adc_ch);
    }
    short adc_vol_prev = ADC_BTN_INVALID_ID;
    short adc_vol_cur = ADC_BTN_INVALID_ID;
    short internal_time_ms = DIAL_VOL_INTERVAL_TIME_MS / ADC_SAMPLE_INTERVAL_TIME_MS; /// 10 * 10 = 100ms
    static bool empty_flag;
    static bool full_flag;
    bool is_first_time = true;
#endif // ENABLE_ADC_VOLUME

    static adc_btn_state_t cur_state = ADC_BTN_STATE_ADC;
    adc_btn_state_t btn_st = ADC_BTN_STATE_IDLE;
    int cur_act_id = ADC_BTN_INVALID_ACT_ID;
    while (_task_flag) {
#if defined ENABLE_ADC_VOLUME
        if (internal_time_ms == 0) {
            adc_vol_cur = get_adc_voltage(DIAL_adc_ch);
            internal_time_ms = DIAL_VOL_INTERVAL_TIME_MS / ADC_SAMPLE_INTERVAL_TIME_MS;
            if (adc_vol_prev > 0) {
                short n = abs(adc_vol_cur - adc_vol_prev);
                if (is_first_time) {
                    is_first_time = false;
                }
                if (adc_vol_cur < 200) {
                    if (empty_flag == false) {
                        ESP_LOGI(TAG, "ABS_LOW:%d, %d->0", n, adc_vol_cur / 25);
                        empty_flag = true;
                    }
                } else if (adc_vol_cur > 2500) {
                    if (full_flag == false) {
                        ESP_LOGI(TAG, "ABS_HIGH:%d, %d->100", n, adc_vol_cur / 25);
                        full_flag = true;
                    }
                } else if (n > 80) {
                    empty_flag = false;
                    full_flag = false;
                }
            }
            adc_vol_prev = adc_vol_cur;
        }
        internal_time_ms--;
#else
        find = head;
        while (find) {
            adc_arr_t *info = &(find->adc_info);
            int act_id = ADC_BTN_INVALID_ACT_ID;
            btn_decription *btn_dscp = find->btn_dscp;
            switch (cur_state) {
                case ADC_BTN_STATE_ADC: {
                        int adc = get_adc_voltage(info->adc_ch);
                        ESP_LOGD(TAG, "ADC:%d", adc);
                        for (int i = 0; i < info->total_steps; ++i) {
                            if (btn_dscp[i].active_id > ADC_BTN_INVALID_ID) {
                                act_id = i;
                                break;
                            }
                        }
                        btn_st = get_adc_btn_state(adc, act_id, find);
                        if (btn_st != ADC_BTN_STATE_IDLE) {
                            cur_act_id = act_id;
                            cur_state = btn_st;
                            ESP_LOGD(TAG, "ADC ID:%d", act_id);
                        }
                        break;
                    }
                case ADC_BTN_STATE_PRESSED: {
                        int voltage = get_adc_voltage(info->adc_ch);
                        // Re-validate at dispatch time. cur_act_id was latched two
                        // samples earlier (ADC_BTN_DETECTED_CNT), but this sample is
                        // fresh: during rapid clicking the finger may already be off
                        // (idle voltage -> invalid id) or have bounced into another
                        // window by now. Dispatching the stale id would fire a real
                        // CLICK for a button no longer held (phantom press). If the
                        // fresh window no longer matches, drop the press; a genuine
                        // hold still reads the same id (its own ladder window) and
                        // dispatches normally. The state machine returns to ADC below
                        // either way, so release handling is unchanged.
                        int fresh_id = get_button_id(find, voltage);
                        if (fresh_id != cur_act_id) {
                            ESP_LOGW(TAG, "phantom press dropped: act_id=%d fresh=%dmV", cur_act_id, voltage);
                        } else {
                            ESP_LOGI(TAG, "Button pressed - Measured voltage: %dmV, Button ID: %d", voltage, cur_act_id);
                            tag->btn_callback((void *)tag->user_data, info->adc_ch, cur_act_id, ADC_BTN_STATE_PRESSED);
                        }
                        cur_state = ADC_BTN_STATE_ADC;
                        break;
                    }
                case ADC_BTN_STATE_LONG_PRESSED: {
                        int voltage = get_adc_voltage(info->adc_ch);
                        ESP_LOGI(TAG, "Button long pressed - Measured voltage: %dmV, Button ID: %d", voltage, cur_act_id);
                        tag->btn_callback((void *)tag->user_data, info->adc_ch, cur_act_id, ADC_BTN_STATE_LONG_PRESSED);
                        cur_state = ADC_BTN_STATE_ADC;
                        break;
                    }
                case ADC_BTN_STATE_LONG_RELEASE: {
                        int voltage = get_adc_voltage(info->adc_ch);
                        ESP_LOGI(TAG, "Button long release - Measured voltage: %dmV, Button ID: %d", voltage, cur_act_id);
                        tag->btn_callback((void *)tag->user_data, info->adc_ch, cur_act_id, ADC_BTN_STATE_LONG_RELEASE);
                        cur_state = ADC_BTN_STATE_ADC;
                        break;
                    }
                case ADC_BTN_STATE_RELEASE: {
                        int voltage = get_adc_voltage(info->adc_ch);
                        ESP_LOGI(TAG, "Button release - Measured voltage: %dmV, Button ID: %d", voltage, cur_act_id);
                        tag->btn_callback((void *)tag->user_data, info->adc_ch, cur_act_id, ADC_BTN_STATE_RELEASE);
                        cur_state = ADC_BTN_STATE_ADC;
                        break;
                    }
                default:
                    ESP_LOGE(TAG, "Not support state %d", cur_state);
                    break;
            }
            find = find->next;
        }
#endif // ENABLE_ADC_VOLUME

        vTaskDelay(ADC_SAMPLE_INTERVAL_TIME_MS / portTICK_PERIOD_MS);
    }

    if (!s_read_mv) {
        fallback_unit_close();
    }
    s_last_mv = -1;
    if (g_event_bit) {
        xEventGroupSetBits(g_event_bit, DESTROY_BIT);
    }
    audio_free(tag);
    vTaskDelete(NULL);
}

void adc_btn_delete_task(void)
{
    if (_task_flag) {
        _task_flag = false;
    }

    if (g_event_bit) {
        xEventGroupWaitBits(g_event_bit, DESTROY_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        vEventGroupDelete(g_event_bit);
        g_event_bit = NULL;
    }
}

void adc_btn_init(void *user_data, adc_button_callback cb, adc_btn_list *head, adc_btn_task_cfg_t *task_cfg)
{
    adc_btn_init_with_reader(user_data, cb, head, task_cfg, NULL, NULL);
}

void adc_btn_init_with_reader(void *user_data, adc_button_callback cb, adc_btn_list *head,
                              adc_btn_task_cfg_t *task_cfg, adc_btn_read_mv_fn read_mv, void *read_ctx)
{
    adc_btn_tag_t *tag = audio_calloc(1, sizeof(adc_btn_tag_t));
    if (NULL == tag) {
        ESP_LOGE(TAG, "Memory allocation failed! Line: %d", __LINE__);
        return;
    }
    s_read_mv = read_mv;
    s_read_ctx = read_ctx;
    tag->user_data = user_data;
    tag->head = head;
    tag->btn_callback = cb;

    g_event_bit = xEventGroupCreate();

    audio_thread_create(&tag->audio_thread,
                        "button_task", button_task,
                        (void *)tag,
                        task_cfg->task_stack,
                        task_cfg->task_prio,
                        task_cfg->ext_stack,
                        task_cfg->task_core);
}
