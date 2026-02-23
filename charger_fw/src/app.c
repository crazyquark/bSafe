// =============================================================================
// app.c — Port of main.py App class to ESP-IDF C
// =============================================================================
#include "app.h"
#include "config.h"
#include "nvs_settings.h"
#include "bq25895.h"
#include "ina226.h"
#include "tca9535.h"
#include "display.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "app";

// ---------------------------------------------------------------------------
// Time helpers
// ---------------------------------------------------------------------------
static inline uint32_t ms_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline uint32_t us_now(void)
{
    return (uint32_t)(esp_timer_get_time());
}

// ---------------------------------------------------------------------------
// MCU internal temperature sensor
// ---------------------------------------------------------------------------
static temperature_sensor_handle_t s_temp_sensor = NULL;

static void _temp_sensor_init(void)
{
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    temperature_sensor_install(&cfg, &s_temp_sensor);
    temperature_sensor_enable(s_temp_sensor);
}

static float _read_mcu_temp_c(void)
{
    if (!s_temp_sensor) return -99.0f;
    float t = 0.0f;
    temperature_sensor_get_celsius(s_temp_sensor, &t);
    return t;
}

// ---------------------------------------------------------------------------
// External NTC (GPIO1, ADC)
// ---------------------------------------------------------------------------
static adc_oneshot_unit_handle_t s_adc = NULL;
static bool s_adc_ready = false;

static void _ext_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&init_cfg, &s_adc) != ESP_OK) return;

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,   // 0..3.3V range
        .bitwidth = ADC_BITWIDTH_12,
    };
    // GPIO1 = ADC1_CH1 on ESP32-C3
    if (adc_oneshot_config_channel(s_adc, ADC_CHANNEL_1, &ch_cfg) != ESP_OK) return;
    s_adc_ready = true;
}

static bool _read_ext_temp_c(float *out_c)
{
    if (!s_adc_ready || !out_c) return false;

    // 20-sample average
    int64_t acc = 0;
    for (int i = 0; i < 20; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, ADC_CHANNEL_1, &raw);
        acc += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    float raw = (float)(acc / 20);
    if (raw <= 5.0f || raw >= 4090.0f) return false;

    float r_ntc = EXT_R_PULL_OHM * (raw / (4095.0f - raw));
    if (r_ntc <= 1.0f) return false;

    float lnr = logf(r_ntc);
    float inv_t = EXT_SH_A + EXT_SH_B * lnr + EXT_SH_C * (lnr * lnr * lnr);
    *out_c = (1.0f / inv_t) - 273.15f;
    return true;
}

// ---------------------------------------------------------------------------
// BQ25895 TS% -> °C
// ---------------------------------------------------------------------------
static bool _ts_pct_to_temp_c(float ts_pct, float *out_c)
{
    float x = ts_pct / 100.0f;
    if (x < 1e-6f)  x = 1e-6f;
    if (x > 0.999999f) x = 0.999999f;
    float r_ntc = TS_R_PULLUP_OHM * (x / (1.0f - x));
    float t0_k = TS_T0_C + 273.15f;
    float inv_t = (1.0f / t0_k) + (1.0f / TS_BETA_K) * logf(r_ntc / TS_R0_OHM);
    *out_c = (1.0f / inv_t) - 273.15f;
    return true;
}

// ---------------------------------------------------------------------------
// LEDC (discharge / measure PWM on GPIO0)
// ---------------------------------------------------------------------------
static bool s_ledc_ready = false;

static void _ledc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = DSC_TIMER,
        .duty_resolution = DSC_DUTY_BITS,
        .freq_hz         = DSC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer) != ESP_OK) return;

    ledc_channel_config_t ch = {
        .gpio_num   = DSC_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = DSC_CHANNEL,
        .timer_sel  = DSC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    if (ledc_channel_config(&ch) != ESP_OK) return;
    s_ledc_ready = true;
}

static void _ledc_set_pct(int pct)
{
    if (!s_ledc_ready) return;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    uint32_t duty = (uint32_t)(DSC_DUTY_MAX * pct / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, DSC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, DSC_CHANNEL);
}

// ---------------------------------------------------------------------------
// SK6812 NeoPixel via RMT (led_strip v3 component)
// ---------------------------------------------------------------------------
#include "led_strip.h"

static led_strip_handle_t s_led_strip = NULL;

// Global LED state — updated each loop tick
typedef enum {
    LED_MODE_IDLE = 0,  // main menu / status — no backlight
    LED_MODE_CHARGE,    // backlight green
    LED_MODE_DISCHARGE, // backlight red
    LED_MODE_MEASURE,   // backlight red (load applied)
    LED_MODE_WAIT,      // backlight dim white (CAN: waiting for command)
} led_mode_t;

static led_mode_t s_led_mode  = LED_MODE_IDLE;
static uint32_t   s_led_frame = 0;   // incremented once per tick in app_run()

// Blink state derived from frame counter (400ms @ 20ms poll = 20 frames)
#define BLINK_FRAMES 20
static inline bool _blink_on(void) { return (s_led_frame % (BLINK_FRAMES * 2)) < BLINK_FRAMES; }

// Raw color helpers — (r, g, b) passed directly to led_strip_set_pixel
static inline void _px(int i, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_strip || i < 0 || i >= LED_COUNT) return;
    led_strip_set_pixel(s_led_strip, i, r, g, b);
}

static void _leds_strip_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num         = LED_GPIO,
        .max_leds               = LED_COUNT,
        .led_model              = LED_MODEL_SK6812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out       = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma    = false,
    };
    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE("led", "led_strip_new_rmt_device failed: %s", esp_err_to_name(ret));
        s_led_strip = NULL;
        return;
    }
    led_strip_clear(s_led_strip);
    ESP_LOGI("led", "SK6812 strip ready (GPIO%d, %d LEDs)", LED_GPIO, LED_COUNT);
}

// ---------------------------------------------------------------------------
// update_leds() — single function owning all 10 LEDs
//
//  LEDs 0-4  : animation (ping-pong idle dot, or charge/discharge progress)
//  LEDs 5-9  : keyboard backlight — color set by s_led_mode
//
// Call once per main loop tick. Uses s_led_frame and s_led_mode globals.
// Takes optional charge/discharge data for progress animation on LEDs 0-4.
// Pass vbat_valid=false when data is not available.
// ---------------------------------------------------------------------------
static void update_leds(
    float vbat, bool vbat_valid,
    float vmin,  float vmax,        // used by CHARGE animation
    float v_full, float v_empty,    // used by DISCHARGE animation
    bool  batt_present,
    bool  chg_error, bool chg_full,
    const char *chg_mode)           // "PRE","CC","CV","DONE","OFF",NULL
{
    if (!s_led_strip) return;

    bool blink = _blink_on();

    // ---- LEDs 5-9: backlight ------------------------------------------------
    uint8_t bl_r = 0, bl_g = 0, bl_b = 0;
    switch (s_led_mode) {
    case LED_MODE_CHARGE:    bl_r =  0; bl_g = 30; bl_b =  0; break;  // green
    case LED_MODE_DISCHARGE: bl_r = 40; bl_g =  0; bl_b =  0; break;  // red
    case LED_MODE_MEASURE:   bl_r = 40; bl_g =  0; bl_b =  0; break;  // red
    case LED_MODE_WAIT:      bl_r = 15; bl_g = 15; bl_b = 15; break;  // dim white
    default:                 bl_r =  0; bl_g =  0; bl_b =  0; break;  // off
    }
    for (int i = 5; i < 10; i++) _px(i, bl_r, bl_g, bl_b);

    // ---- LEDs 0-4: animation ------------------------------------------------
    // Each pixel value as {r,g,b}
    uint8_t ar[5]={}, ag[5]={}, ab[5]={};

    if (s_led_mode == LED_MODE_IDLE) {
        // Ping-pong: 0->1->2->3->4->3->2->1->(repeat) = 8 steps/cycle
        // /2 half-speed: each position holds 40ms, full sweep 320ms
        int half = (s_led_frame / 2) % 8;
        int pos  = (half <= 4) ? half : (8 - half);  // 0,1,2,3,4,3,2,1
        ar[pos] = LED_IDLE_R; ag[pos] = LED_IDLE_G; ab[pos] = LED_IDLE_B;

    } else if (s_led_mode == LED_MODE_CHARGE) {
        // Error latch: all blink red
        if (chg_error) {
            uint8_t r = blink ? LED_ERR_R : 0;
            uint8_t g = blink ? LED_ERR_G : 0;
            for (int i=0;i<5;i++) { ar[i]=r; ag[i]=g; }
            goto write;
        }
        // Full latch: all blink green
        if (chg_full) {
            uint8_t r = blink ? LED_CHG_R : 0;
            uint8_t g = blink ? LED_CHG_G : 0;
            for (int i=0;i<5;i++) { ar[i]=r; ag[i]=g; }
            goto write;
        }
        // No battery
        if (!vbat_valid || !batt_present) goto write;  // all off
        // PRE mode: blink LED0 only
        if (chg_mode && strcmp(chg_mode, "PRE") == 0) {
            ar[0] = blink ? LED_CHG_R : 0;
            ag[0] = blink ? LED_CHG_G : 0;
            goto write;
        }
        // CC/CV: progress
        float pct = (vmax > vmin) ? ((vbat - vmin) / (vmax - vmin) * 100.0f) : 0.0f;
        if (pct < 0.0f)   pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        
        int solid, blink_idx;
        if      (pct < 25.0f) { solid = 1; blink_idx = 1; }
        else if (pct < 50.0f) { solid = 2; blink_idx = 2; }
        else if (pct < 75.0f) { solid = 3; blink_idx = 3; }
        else                  { solid = 4; blink_idx = 4; }
        for (int i=0;i<solid;i++) { ar[i]=LED_CHG_R; ag[i]=LED_CHG_G; }
        ar[blink_idx] = blink ? LED_CHG_R : 0;
        ag[blink_idx] = blink ? LED_CHG_G : 0;

    } else if (s_led_mode == LED_MODE_DISCHARGE || s_led_mode == LED_MODE_MEASURE) {
        if (!vbat_valid || v_full <= v_empty) {
            // Config error: blink red
            uint8_t r = blink ? LED_ERR_R : 0;
            uint8_t g = blink ? LED_ERR_G : 0;
            for (int i=0;i<5;i++) { ar[i]=r; ag[i]=g; }
            goto write;
        }
        float dsc_pct;
        if      (vbat >= v_full)  dsc_pct = 0.0f;
        else if (vbat <= v_empty) dsc_pct = 100.0f;
        else                      dsc_pct = (v_full - vbat) / (v_full - v_empty) * 100.0f;
        int solid, blink_idx;
        if      (dsc_pct < 25.0f) { solid = 1; blink_idx = 1; }
        else if (dsc_pct < 50.0f) { solid = 2; blink_idx = 2; }
        else if (dsc_pct < 75.0f) { solid = 3; blink_idx = 3; }
        else                      { solid = 4; blink_idx = 4; }
        for (int i=0;i<solid;i++) { ar[i]=LED_DSC_R; ag[i]=LED_DSC_G; }
        ar[blink_idx] = blink ? LED_DSC_R : 0;
        ag[blink_idx] = blink ? LED_DSC_G : 0;

    } else if (s_led_mode == LED_MODE_WAIT) {
        // All dim white, slow blink
        uint8_t v = (s_led_frame % 40) < 20 ? 10 : 0;
        for (int i=0;i<5;i++) { ar[i]=v; ag[i]=v; ab[i]=v; }
    }

write:
    for (int i=0;i<5;i++) _px(i, ar[i], ag[i], ab[i]);
    led_strip_refresh(s_led_strip);
}

// ---------------------------------------------------------------------------
// Tachometer GPIO interrupt
// ---------------------------------------------------------------------------
static void IRAM_ATTR _tach_isr(void *arg)
{
    app_t *app = (app_t *)arg;
    uint32_t now = (uint32_t)(esp_timer_get_time());  // us
    if (app->tach_valid) {
        uint32_t dt = now - app->tach_last_us;
        if (dt >= TACH_MIN_PERIOD_US) {
            app->tach_period_us  = dt;
            app->tach_last_us    = now;
            app->tach_last_edge_us = now;
        }
        // else glitch, ignore
    } else {
        app->tach_last_us      = now;
        app->tach_last_edge_us = now;
        app->tach_valid        = true;
    }
}

static void _tach_init(app_t *app)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << TACH_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TACH_GPIO, _tach_isr, app);
}

static float _tach_compute_rpm(app_t *app)
{
    if (!app->tach_valid) return 0.0f;

    uint32_t now = us_now();
    if ((now - app->tach_last_edge_us) > TACH_TIMEOUT_US) {
        app->tach_period_us = 0;
        app->tach_valid     = false;
        app->tach_ema_rpm   = 0.0f;
        return 0.0f;
    }

    uint32_t p = app->tach_period_us;
    if (p == 0) return 0.0f;

    float rpm = 60000000.0f / ((float)p * (float)TACH_PPR);
    float a = 0.3f;
    if (app->tach_ema_rpm == 0.0f)
        app->tach_ema_rpm = rpm;
    else
        app->tach_ema_rpm = (1.0f - a) * app->tach_ema_rpm + a * rpm;
    return app->tach_ema_rpm;
}

// ---------------------------------------------------------------------------
// Repeat button
// ---------------------------------------------------------------------------
static bool _repeat_poll(repeat_btn_t *btn, bool is_down, uint32_t now_ms)
{
    bool fired = false;
    if (is_down && !btn->was_down) {
        btn->t_down_ms    = now_ms;
        btn->t_last_rep_ms = now_ms;
        fired = true;
    } else if (is_down && btn->was_down) {
        uint32_t held = now_ms - btn->t_down_ms;
        if (held >= REPEAT_DELAY_MS) {
            if ((now_ms - btn->t_last_rep_ms) >= REPEAT_MS) {
                btn->t_last_rep_ms = now_ms;
                fired = true;
            }
        }
    }
    btn->was_down = is_down;
    return fired;
}

// ---------------------------------------------------------------------------
// Expander I/O helpers (wrappers over board_t)
// ---------------------------------------------------------------------------
static bool _refresh_exp(app_t *app)
{
    if (!app->board->has_exp) return false;
    esp_err_t r = tca9535_read_all(&app->board->exp,
                                   &app->board->in0, &app->board->in1);
    return (r == ESP_OK);
}

static int _exp_level(app_t *app, int pin)
{
    if (pin < 8) return (app->board->in0 >> pin) & 1;
    return (app->board->in1 >> (pin - 8)) & 1;
}

static void _exp_write(app_t *app, int pin, bool val)
{
    if (!app->board->has_exp) return;
    tca9535_write_pin(&app->board->exp, pin, val ? 1 : 0);
}

static void _set_fan(app_t *app, bool on)     { _exp_write(app, P_FAN, on); }
static void _set_ce(app_t *app, bool enabled) { _exp_write(app, P_CE,  enabled ? 0 : 1); }
static void _set_qon(app_t *app, bool on)     { _exp_write(app, P_QON, on); }

// ---------------------------------------------------------------------------
// Input polling
// ---------------------------------------------------------------------------
typedef struct { bool left, right, menu, ok; } events_t;

static events_t _poll_inputs(app_t *app)
{
    uint32_t now = ms_now();
    bool exp_ok = _refresh_exp(app);

    bool left_down  = exp_ok ? (_exp_level(app, P_BTN2) == 0) : false;
    bool right_down = exp_ok ? (_exp_level(app, P_BTN3) == 0) : false;
    bool menu_level = exp_ok ? (_exp_level(app, P_BTN1) == 0) : false;
    bool ok_level   = (gpio_get_level(GPIO_OK) == 0);

    events_t ev = {
        .left  = _repeat_poll(&app->rep_left,  left_down,  now),
        .right = _repeat_poll(&app->rep_right, right_down, now),
        .menu  = (menu_level && !app->menu_prev),
        .ok    = (ok_level   && !app->ok_prev),
    };
    app->menu_prev = menu_level;
    app->ok_prev   = ok_level;
    return ev;
}

// ---------------------------------------------------------------------------
// Menu helpers
// ---------------------------------------------------------------------------
static menu_item_t *_item(app_t *app, int i) { return &app->items[i]; }
static menu_item_t *_sel(app_t *app)         { return &app->items[app->sel]; }

static float _get_float(app_t *app, const char *key, float def)
{
    for (int i = 0; i < app->num_items; i++) {
        if (strcmp(app->items[i].key, key) == 0) {
            if (app->items[i].type == ITEM_LIST)
                return def;
            return app->items[i].val;
        }
    }
    return def;
}


static void _move_sel(app_t *app, int dir)
{
    int n = app->num_items;
    if (n <= 0) return;
    app->sel = ((app->sel + dir) % n + n) % n;
}

static void _step_float(menu_item_t *it, int dir)
{
    float v = it->val + it->step * (dir > 0 ? 1.0f : -1.0f);
    if (v > it->vmax + 1e-9f) v = it->vmin;
    else if (v < it->vmin - 1e-9f) v = it->vmax;
    // snap to step grid
    int n = (int)roundf((v - it->vmin) / it->step);
    v = it->vmin + n * it->step;
    if (v > it->vmax) v = it->vmax;
    if (v < it->vmin) v = it->vmin;
    it->val = v;
}

static void _step_int(menu_item_t *it, int dir)
{
    int v = (int)it->val + (int)it->step * (dir > 0 ? 1 : -1);
    if (v > (int)it->vmax) v = (int)it->vmin;
    else if (v < (int)it->vmin) v = (int)it->vmax;
    it->val = (float)v;
}

static void _step_list(menu_item_t *it, int dir)
{
    it->idx = ((it->idx + dir) % it->nopts + it->nopts) % it->nopts;
}

static void _enter_edit(app_t *app)
{
    app->mode = MODE_EDIT;
    menu_item_t *it = _sel(app);
    if (it->type == ITEM_LIST)
        app->edit_backup_idx = it->idx;
    else
        app->edit_backup_val = it->val;
}

static void _cancel_edit(app_t *app)
{
    menu_item_t *it = _sel(app);
    if (it->type == ITEM_LIST)
        it->idx = app->edit_backup_idx;
    else
        it->val = app->edit_backup_val;
    app->mode = MODE_MAIN;
}

static void _confirm_edit(app_t *app)
{
    menu_item_t *it = _sel(app);
    if (strcmp(it->key, "Exec") == 0) {
        const char *action = it->opts[it->idx];
        if      (strcmp(action, "Charge")    == 0) app->page = PAGE_CHARGE;
        else if (strcmp(action, "Discharge") == 0) app->page = PAGE_DISCHARGE;
        else if (strcmp(action, "Measure")   == 0) app->page = PAGE_MEASURE;
        else if (strcmp(action, "Status")    == 0) app->page = PAGE_STATUS;
        else if (strcmp(action, "CAN")       == 0) app->page = PAGE_CAN;
        app->mode = MODE_PAGE;
    } else {
        // Persist changed value to NVS (no-op if g_skip_nvs)
        const char *nvs_key = nvs_key_for_menu_item(it->key);
        if (nvs_key) {
            if (it->type == ITEM_FLOAT) {
                nvs_settings_save_float(nvs_key, it->val);
            } else if (it->type == ITEM_INT) {
                nvs_settings_save_int(nvs_key, (int32_t)it->val);
            }
        }
        app->mode = MODE_MAIN;
    }
}

static void _step_current(app_t *app, int dir)
{
    menu_item_t *it = _sel(app);
    if      (it->type == ITEM_FLOAT) _step_float(it, dir);
    else if (it->type == ITEM_INT)   _step_int(it, dir);
    else if (it->type == ITEM_LIST)  _step_list(it, dir);
}

// ---------------------------------------------------------------------------
// BQ25895 helpers
// ---------------------------------------------------------------------------
static bool _bq_try_read_status(app_t *app, bq25895_status_t *st)
{
    if (!app->board->has_bq) return false;
    return (bq25895_read_status(&app->board->bq, st) == ESP_OK);
}

static bool _bq_is_powered(app_t *app)
{
    bq25895_status_t st;
    if (!_bq_try_read_status(app, &st)) return false;
    return st.power_good;
}

static bool _read_bq_temp_c(app_t *app, float *out_c)
{
    if (!app->board->has_bq) return false;
    float ts_pct = 0.0f;
    // REG10[6:0]: TS% = 21% + code*0.465% — read directly, no dedicated API function
    uint8_t raw_ts = 0;
    uint8_t reg_ts = BQ_REG10;
    if (i2c_master_transmit_receive(app->board->bq.dev,
            &reg_ts, 1, &raw_ts, 1, 50) != ESP_OK) return false;
    ts_pct = 21.0f + (float)(raw_ts & 0x7F) * 0.465f;
    return _ts_pct_to_temp_c(ts_pct, out_c);
}

// static void _bq_wake_once(app_t *app)
// {
//     if (app->bq_wake_attempted) return;
//     app->bq_wake_attempted = true;

//     _set_qon(app, true);
//     uint32_t t0 = ms_now();
//     while ((ms_now() - t0) < 2500) {
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
//     _set_qon(app, false);

//     // Re-init BQ after wake — re-register device on the existing bus
//     if (app->board->has_bq)
//         bq25895_init(app->board->i2c_bus, 0x6A, &app->board->bq);
// }

static bool _read_vbat_v(app_t *app, float *out)
{
    if (!app->board->has_ina_batt) return false;
    ina226_reading_t r;
    if (ina226_read_all(&app->board->ina_batt, &r) != ESP_OK) return false;
    *out = r.bus_v;
    return true;
}

static bool _battery_present(float vbat) { return vbat >= VBAT_PRESENT_V; }

// static float _estimate_pct(float vbat, float vmin, float vmax)
// {
//     if (vmax <= vmin) return 0.0f;
//     float x = (vbat - vmin) / (vmax - vmin);
//     if (x < 0.0f) x = 0.0f;
//     if (x > 1.0f) x = 1.0f;
//     return x * 100.0f;
// }

static bool _input_power_available(app_t *app)
{
    bq25895_status_t st;
    if (!_bq_try_read_status(app, &st)) return false;
    if (!app->board->has_ina_sys) return false;
    ina226_reading_t r;
    if (ina226_read_all(&app->board->ina_sys, &r) != ESP_OK) return false;
    return (r.bus_v > VIN_MIN_OK_V);
}

// charge mode string from BQ status
static const char *_bq_charge_mode(bq25895_status_t *st, float vbat, float v_target)
{
    if (st == NULL) return "UNPOWERED";
    switch (st->chrg_stat) {
        case 0: return "OFF";
        case 1: return "PRE";
        case 2: return (vbat >= (v_target - 0.03f)) ? "CV" : "CC";
        case 3: return "DONE";
        default: return "UNK";
    }
}

static bool _apply_bq_for_charge(app_t *app)
{
    if (!app->board->has_bq) return false;
    if (!_bq_is_powered(app)) return false;

    float v_target = _get_float(app, "ChgTo",   4.20f);
    int   cap_mah  = (int)_get_float(app, "Capacity", 3000.0f);

    int cap_for_c = (cap_mah < 10200) ? cap_mah : 10200;
    int ichg_ma   = cap_for_c / 2;
    if (ichg_ma > 5056) ichg_ma = 5056;
    int iinlim_ma = 3250;

    ESP_LOGI(TAG, "[CHG] Vtgt=%.2fV  Ichg=%dmA  Iinlim=%dmA", v_target, ichg_ma, iinlim_ma);

    bq25895_set_hiz(&app->board->bq, false);
    bq25895_set_charge_enable(&app->board->bq, true);
    _set_ce(app, true);
    bq25895_set_input_current_limit_ma(&app->board->bq, iinlim_ma);
    bq25895_set_charge_voltage_mv(&app->board->bq, (int)(v_target * 1000.0f));
    bq25895_set_fast_charge_current_ma(&app->board->bq, ichg_ma);

    int ipre_ma  = ichg_ma / 10;  if (ipre_ma  < 64) ipre_ma  = 64;
    int iterm_ma = ichg_ma / 20;  if (iterm_ma < 64) iterm_ma = 64;
    bq25895_set_precharge_current_ma(&app->board->bq, ipre_ma);
    bq25895_set_termination_current_ma(&app->board->bq, iterm_ma);

    // REG07: disable watchdog (bits[5:4]=00), disable safety timer (bit3=0), enable termination (bit7=1)
    // Use the driver's bitmask constants directly — no raw register access API exposed.
    // BQ_B_EN_TERM=0x80, BQ_M_WATCHDOG=0x30, BQ_B_EN_TIMER(bit3)=0x08
    // We call set functions for what the driver exposes; for the rest we note it's handled
    // by the driver default (watchdog already disabled in REG07 default = 0x8F on BQ25895).
    // Kick watchdog to reset the timer regardless:
    bq25895_watchdog_kick(&app->board->bq);
    return true;
}

// ---------------------------------------------------------------------------
// OLED rendering helpers
// ---------------------------------------------------------------------------
static void _draw_inverted_title(display_t *d, const char *text)
{
    // 16px tall title bar (rows 0+1), centred
    display_fill_rect(d, 0, 0, DISP_W, 16, true);
    int x = 64 - (int)(strlen(text) * 4);
    if (x < 0) x = 0;
    display_draw_str(d, x, 4, text, true);
}

static void _draw_inverted_text(display_t *d, int x, int y, const char *text)
{
    int w = (int)(strlen(text) * 8);
    display_fill_rect(d, x, y, w, 8, true);
    display_draw_str(d, x + 1, y, text, true);
}

static void _draw_progress_bar(display_t *d, int x, int y, int w, int h, float frac)
{
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    display_rect(d, x, y, w, h, true);
    int fill = (int)((w - 2) * frac);
    if (fill > 0) display_fill_rect(d, x + 1, y + 1, fill, h - 2, true);
}

// ---------------------------------------------------------------------------
// Render: STARTUP splash
// ---------------------------------------------------------------------------
static void _render_startup(display_t *d, const char *line1,
                            const char *line2, const char *line3, float frac)
{
    display_clear(d, 0);
    _draw_inverted_title(d, "bSafe V1");
    if (line1 && line1[0]) display_draw_str(d, 0, 12, line1, false);
    if (line2 && line2[0]) display_draw_str(d, 0, 24, line2, false);
    if (line3 && line3[0]) display_draw_str(d, 0, 36, line3, false);
    if (frac >= 0.0f)      _draw_progress_bar(d, 0, 56, DISP_W, 8, frac);
    display_flush(d);
}

// ---------------------------------------------------------------------------
// Render: MAIN menu
// ---------------------------------------------------------------------------
static void _render_main(app_t *app)
{
    display_t *d = app->disp;
    display_clear(d, 0);

    for (int i = 0; i < app->num_items; i++) {
        int y = i * 9;
        menu_item_t *it = _item(app, i);

        char key_s[24], val_s[16];
        snprintf(key_s, sizeof(key_s), "%s:", it->key);
        if (it->type == ITEM_LIST) {
            snprintf(val_s, sizeof(val_s), " %s ", it->opts[it->idx]);
        } else if (it->decimals == 0) {
            snprintf(val_s, sizeof(val_s), " %d ", (int)it->val);
        } else if (it->decimals == 1) {
            snprintf(val_s, sizeof(val_s), " %.1f ", it->val);
        } else {
            snprintf(val_s, sizeof(val_s), " %.2f ", it->val);
        }

        int kw = (int)(strlen(key_s) * 8);

        if (i == app->sel && app->mode == MODE_MAIN) {
            _draw_inverted_text(d, 0, y, key_s);
            display_draw_str(d, kw, y, val_s, false);
        } else if (i == app->sel && app->mode == MODE_EDIT) {
            display_draw_str(d, 0, y, key_s, false);
            _draw_inverted_text(d, kw, y, val_s);
        } else {
            display_draw_str(d, 0, y, key_s, false);
            display_draw_str(d, kw, y, val_s, false);
        }
    }
    display_flush(d);
}

// ---------------------------------------------------------------------------
// Render: STATUS page
// ---------------------------------------------------------------------------
static void _render_status(app_t *app, uint32_t now_ms)
{
    if ((now_ms - app->status_next_draw_ms) > 0x80000000UL) return;
    app->status_next_draw_ms = now_ms + 1000;

    float vin = 0.0f, iin = 0.0f, vbat = 0.0f, ibat = 0.0f;
    bool  vin_ok = false, vbat_ok = false;

    if (app->board->has_ina_sys) {
        ina226_reading_t r;
        if (ina226_read_all(&app->board->ina_sys, &r) == ESP_OK) {
            vin = r.bus_v; iin = r.current_a; vin_ok = true;
        }
    }
    if (app->board->has_ina_batt) {
        ina226_reading_t r;
        if (ina226_read_all(&app->board->ina_batt, &r) == ESP_OK) {
            vbat = r.bus_v; ibat = r.current_a; vbat_ok = true;
        }
    }

    float t_mcu = _read_mcu_temp_c();
    float t_bq  = 0.0f;
    bool  t_bq_ok = _read_bq_temp_c(app, &t_bq);
    float rpm = _tach_compute_rpm(app);

    // BQ charger status label
    bq25895_status_t st = {0};
    bool has_st = _bq_try_read_status(app, &st);
    char chg11[12] = "UNPOWERED";
    if (has_st) {
        const char *cs_map[] = {"IDLE","PCHG","FCHG","DONE"};
        const char *cs = (st.chrg_stat < 4) ? cs_map[st.chrg_stat] : "UNK";
        const char *pg = st.power_good ? "PG" : "--";
        snprintf(chg11, sizeof(chg11), "%s %s", cs, pg);
    }

    display_t *d = app->disp;
    display_clear(d, 0);

    char line[20];
    snprintf(line, sizeof(line), "INP:%.2f/%.2f", vin_ok ? vin : 0.0f, vin_ok ? iin : 0.0f);
    display_draw_str(d, 0, 0,  line, false);
    snprintf(line, sizeof(line), "BAT:%.2f/%.2f", vbat_ok ? vbat : 0.0f, vbat_ok ? ibat : 0.0f);
    display_draw_str(d, 0, 12, line, false);
    snprintf(line, sizeof(line), "TMP:%.1f/%.1f", t_mcu, t_bq_ok ? t_bq : 0.0f);
    display_draw_str(d, 0, 24, line, false);
    snprintf(line, sizeof(line), "CHG:%s", chg11);
    display_draw_str(d, 0, 36, line, false);
    snprintf(line, sizeof(line), "FAN:%.0frpm", rpm);
    display_draw_str(d, 0, 48, line, false);

    display_flush(d);
}

// ---------------------------------------------------------------------------
// Render: CHARGE page
// ---------------------------------------------------------------------------
static void _render_charge(app_t *app, uint32_t now_ms)
{
    if ((now_ms - app->charge_next_draw_ms) > 0x80000000UL) return;
    app->charge_next_draw_ms = now_ms + 1000;

    bool pwr_ok = _input_power_available(app);

    if (pwr_ok && !app->chg_error_latched && !app->chg_full_latched) {
        if (app->board->has_bq) {
            bq25895_set_hiz(&app->board->bq, false);
            bq25895_set_charge_enable(&app->board->bq, true);
        }
        _set_ce(app, true);
    } else {
        _set_ce(app, false);
    }

    display_t *d = app->disp;
    display_clear(d, 0);

    if (!pwr_ok) {
        display_draw_str(d, 0, 16, "Plug in XT-30", false);
        display_draw_str(d, 0, 32, "or USB-C PD 12V", false);
        display_flush(d);
        s_led_mode = LED_MODE_CHARGE;  // still set mode so caller update_leds works
        return;
    }

    float vbat = 0.0f;
    bool  vbat_ok = _read_vbat_v(app, &vbat);
    float vmin    = _get_float(app, "ChgFrom", 2.5f);
    float vtar    = _get_float(app, "ChgTo",   4.2f);
    float t_bq    = 0.0f;
    bool  t_bq_ok = _read_bq_temp_c(app, &t_bq);

    bq25895_status_t st = {0};
    bool has_st = _bq_try_read_status(app, &st);
    const char *mode   = has_st ? _bq_charge_mode(&st, vbat, vtar) : "UNP";
    const char *status;

    if (!has_st || !_bq_is_powered(app)) {
        status = "UNPOWERED";
    } else if (!vbat_ok || !_battery_present(vbat)) {
        status = "NO BATT";
    } else if (vbat < (vmin - 1e-6f)) {
        status = "LOWV"; mode = "LOWV";
    } else if (strcmp(mode, "DONE") == 0) {
        status = "CHARGED";
    } else if (strcmp(mode, "PRE") == 0 ||
               strcmp(mode, "CC")  == 0 ||
               strcmp(mode, "CV")  == 0) {
        status = "CHARGING";
    } else if (strcmp(mode, "OFF") == 0) {
        status = "IDLE";
    } else {
        status = mode;
    }

    float ichg_a = 0.0f;
    bool  ichg_ok = false;
    if (app->board->has_bq &&
        (strcmp(mode,"PRE")==0 || strcmp(mode,"CC")==0 || strcmp(mode,"CV")==0)) {
        int ichg_ma = 0;
        if (bq25895_read_charge_current_ma(&app->board->bq, &ichg_ma) == ESP_OK) {
            ichg_a = ichg_ma / 1000.0f;
            ichg_ok = true;
        }
    }

    char line[20];
    snprintf(line, sizeof(line), "S:%s", status);
    display_draw_str(d, 0, 0, line, false);
    snprintf(line, sizeof(line), "V:%.2f->%.2f", vbat_ok ? vbat : 0.0f, vtar);
    display_draw_str(d, 0, 16, line, false);
    if (ichg_ok)
        snprintf(line, sizeof(line), "A:%.2f/%s", ichg_a, mode);
    else
        snprintf(line, sizeof(line), "A:N/A/%s", mode);
    display_draw_str(d, 0, 32, line, false);
    if (t_bq_ok)
        snprintf(line, sizeof(line), "T:%.1fC", t_bq);
    else
        snprintf(line, sizeof(line), "T:N/A");
    display_draw_str(d, 0, 48, line, false);
    display_flush(d);

    // Check error/full latches
    if (vbat_ok && vbat < (vmin - 1e-6f) && !app->chg_error_latched) {
        _set_ce(app, false); app->chg_error_latched = true;
    }
    if (has_st && strcmp(mode, "DONE") == 0 && !app->chg_full_latched) {
        _set_ce(app, false); app->chg_full_latched = true;
    }

    s_led_mode = LED_MODE_CHARGE;
    // NOTE: update_leds() called by caller every tick — not here
}

// ---------------------------------------------------------------------------
// Render: DISCHARGE page
// ---------------------------------------------------------------------------
static void _render_discharge(app_t *app, uint32_t now_ms)
{
    if ((now_ms - app->dsc_next_draw_ms) > 0x80000000UL) return;
    app->dsc_next_draw_ms = now_ms + 1000;

    float vbat = 0.0f;
    bool  vbat_ok = _read_vbat_v(app, &vbat);
    float v_target = _get_float(app, "DscTo", 3.70f);

    // Stop PWM if battery empty
    if (vbat_ok && vbat <= v_target) _ledc_set_pct(0);

    float ibat_a = 0.0f;
    bool  ibat_ok = false;
    if (app->board->has_ina_batt) {
        ina226_reading_t r;
        if (ina226_read_all(&app->board->ina_batt, &r) == ESP_OK) {
            ibat_a = r.current_a; ibat_ok = true;
        }
    }
    int ibat_ma = ibat_ok ? (int)(ibat_a * 1000.0f) : 0;

    float rpm = _tach_compute_rpm(app);
    float t_ext = 0.0f;
    bool  t_ext_ok = _read_ext_temp_c(&t_ext);

    display_t *d = app->disp;
    display_clear(d, 0);

    // Row 0: RATE label + inverted value
    char label[] = "RATE:";
    display_draw_str(d, 0, 0, label, false);
    char rate_val[10];
    snprintf(rate_val, sizeof(rate_val), " %d%% ", app->dsc_pct);
    _draw_inverted_text(d, (int)(strlen(label)+1)*8, 0, rate_val);

    char line[20];
    snprintf(line, sizeof(line), "VB:%.2f->%.2f", vbat_ok ? vbat : 0.0f, v_target);
    display_draw_str(d, 0, 12, line, false);
    if (ibat_ok)
        snprintf(line, sizeof(line), "I:%dmA", ibat_ma);
    else
        snprintf(line, sizeof(line), "I:N/A");
    display_draw_str(d, 0, 24, line, false);
    snprintf(line, sizeof(line), "FAN:%.0frpm", rpm);
    display_draw_str(d, 0, 36, line, false);
    if (t_ext_ok)
        snprintf(line, sizeof(line), "TEMP:%.0fC", t_ext);
    else
        snprintf(line, sizeof(line), "TEMP:LOW");
    display_draw_str(d, 0, 48, line, false);
    display_flush(d);
}

// ---------------------------------------------------------------------------
// Render: MEASURE page
// ---------------------------------------------------------------------------
static void _render_measure(app_t *app, uint32_t now_ms)
{
    if ((now_ms - app->meas_next_draw_ms) > 0x80000000UL) return;
    app->meas_next_draw_ms = now_ms + 250;

    display_t *d = app->disp;
    display_clear(d, 0);

    display_draw_str(d, 0, 0, "MEASURE I.R.", false);

    _draw_inverted_text(d, 0, 12, "<");
    display_draw_str(d, 8, 12, " Discharge@30%", false);
    _draw_inverted_text(d, 0, 24, ">");
    display_draw_str(d, 8, 24, " Discharge@60%", false);

    // 30% result
    if (!app->meas_30.valid) {
        display_draw_str(d, 0, 36, "30% not yet read", false);
    } else {
        char line[20];
        int dv_mv = (int)(app->meas_30.v_drop * 1000.0f);
        int r_mo  = (int)(app->meas_30.r_mohm - CIRCUIT_R_DISCHG_MOHM);
        snprintf(line, sizeof(line), "%dmV/%.2fA=>%dmO",
                 dv_mv, fabsf(app->meas_30.i_loaded), r_mo);
        display_draw_str(d, 0, 36, line, false);
    }

    // 60% result
    if (!app->meas_60.valid) {
        display_draw_str(d, 0, 48, "60% not yet read", false);
    } else {
        char line[20];
        int dv_mv = (int)(app->meas_60.v_drop * 1000.0f);
        int r_mo  = (int)(app->meas_60.r_mohm - CIRCUIT_R_DISCHG_MOHM);
        snprintf(line, sizeof(line), "%dmV/%.2fA=>%dmO",
                 dv_mv, fabsf(app->meas_60.i_loaded), r_mo);
        display_draw_str(d, 0, 48, line, false);
    }
    display_flush(d);
}

// ---------------------------------------------------------------------------
// Measure I.R. run (blocks for ~5s with live display updates)
// ---------------------------------------------------------------------------
static void _measure_ir_run(app_t *app, int pct)
{
    meas_result_t *res = (pct == 30) ? &app->meas_30 : &app->meas_60;
    *res = (meas_result_t){0};

    // Baseline: no load
    _ledc_set_pct(0);
    vTaskDelay(pdMS_TO_TICKS(100));

    float v0 = 0.0f;
    _read_vbat_v(app, &v0);

    // Apply load
    _ledc_set_pct(pct);

    // Settle for 5s with live display updates
    uint32_t t0 = ms_now();
    while ((ms_now() - t0) < 5000) {
        app->meas_next_draw_ms = 0;
        _render_measure(app, ms_now());
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Sample under load
    float v1 = 0.0f; _read_vbat_v(app, &v1);
    float i1 = 0.0f;
    if (app->board->has_ina_batt) {
        ina226_reading_t r;
        if (ina226_read_all(&app->board->ina_batt, &r) == ESP_OK) i1 = r.current_a;
    }

    // Disable load
    _ledc_set_pct(0);

    float dv    = v0 - v1;
    float i_abs = fabsf(i1);
    float r     = (i_abs > 0.001f) ? (dv / i_abs * 1000.0f) : 0.0f; // mΩ

    ESP_LOGI(TAG, "[MEAS] pct=%d  v0=%.3f  v1=%.3f  dv=%.3f  i=%.3f  r=%.1fmO",
             pct, v0, v1, dv, i1, r);

    res->v_noload  = v0;
    res->v_loaded  = v1;
    res->i_loaded  = i1;
    res->v_drop    = dv;
    res->r_mohm    = r;
    res->valid     = (i_abs > 0.001f);

    app->meas_next_draw_ms = 0;
}

// ---------------------------------------------------------------------------
// CAN telemetry TX — called from any active page
// Frame layout (8 bytes):
//   [0]    mode byte: 0=IDLE 1=CHARGE 2=DISCHARGE 3=MEASURE 4=WAIT
//   [1-2]  vbat mV  (uint16, big-endian)
//   [3-4]  ibat mA  (int16,  big-endian, signed)
//   [5]    charge status flags: bit0=error_latch, bit1=full_latch, bit2=pwr_ok
//   [6]    address (from menu item "Address")
//   [7]    reserved / 0
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// CAN frame definitions
// ---------------------------------------------------------------------------
#define CAN_ID_STATUS  0x100   // charger → host  (ID + address offset)
#define CAN_ID_CMD     0x200   // host → charger  (ID + address offset)

// Mode bytes (shared between CMD and STATUS frames)
#define CAN_MODE_IDLE       0
#define CAN_MODE_CHARGE     1
#define CAN_MODE_DISCHARGE  2
#define CAN_MODE_MEASURE    3
#define CAN_MODE_WAIT       4
#define CAN_MODE_SETTLE     5

// ---------------------------------------------------------------------------
// STATUS frame layout (charger → host, 8 bytes):
//   [0]     mode byte (current mode)
//   [1-2]   vbat mV  (uint16 BE)
//   [3-4]   ibat mA  (int16  BE, signed)
//   [5]     flags: bit0=error bit1=full bit2=pwr_ok bit3=batt_present
//                  bit4=mode_change_request
//   [6]     chrg_stat (full byte, BQ REG0B [4:3], 0-3)
//   [7]     dsc_pct   (full byte, 0-100)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// CMD frame layout (host → charger, 8 bytes):
//   [0]     mode byte
//   [1]     discharge duty % (0-100)
//   [2-3]   discharge pulse duration ms (uint16 BE)
//   [4-5]   settling remaining minutes (uint16 BE, 0-999)
//   [6]     flags: bit0=request_immediate_vbat
//   [7]     reserved
// ---------------------------------------------------------------------------

// Parsed CMD frame — filled by RX handler, read by render/logic
typedef struct {
    uint8_t  mode;
    uint8_t  dsc_duty_pct;
    uint16_t dsc_pulse_ms;
    uint16_t settle_remaining_min;
    bool     req_immediate_vbat;
} can_cmd_t;

static can_cmd_t  s_can_cmd        = { CAN_MODE_WAIT, 30, 5000, 0, false };
static uint32_t   s_can_last_rx_ms = 0;
static uint32_t   s_can_tx_next_ms = 0;

static void _can_parse_cmd(const twai_message_t *rx, can_cmd_t *out)
{
    out->mode             = (rx->data_length_code >= 1) ? (rx->data[0] & 0x07) : CAN_MODE_WAIT;
    out->dsc_duty_pct     = (rx->data_length_code >= 2) ? rx->data[1] : 30;
    out->dsc_pulse_ms     = (rx->data_length_code >= 4)
                            ? ((uint16_t)rx->data[2] << 8 | rx->data[3]) : 5000;
    out->settle_remaining_min = (rx->data_length_code >= 6)
                            ? ((uint16_t)rx->data[4] << 8 | rx->data[5]) : 0;
    out->req_immediate_vbat = (rx->data_length_code >= 7) ? (rx->data[6] & 0x01) : false;
}

static void _can_tx_status(app_t *app, uint8_t mode_byte)
{
    if (!app->board->has_twai) return;

    float vbat = 0.0f; bool vbat_ok = _read_vbat_v(app, &vbat);
    float ibat = 0.0f;
    if (app->board->has_ina_batt) {
        ina226_reading_t r;
        if (ina226_read_all(&app->board->ina_batt, &r) == ESP_OK) ibat = r.current_a;
    }

    bq25895_status_t bq_st = {0};
    bool has_bq_st = _bq_try_read_status(app, &bq_st);

    int      addr    = (int)_get_float(app, "Address", 0);
    uint16_t vbat_mv = vbat_ok ? (uint16_t)(vbat * 1000.0f) : 0;
    int16_t  ibat_ma = (int16_t)(ibat * 1000.0f);
    bool     batt_present = vbat_ok && _battery_present(vbat);
    uint8_t  flags   = (app->chg_error_latched ? 0x01 : 0)
                     | (app->chg_full_latched   ? 0x02 : 0)
                     | (has_bq_st && bq_st.power_good ? 0x04 : 0)
                     | (batt_present             ? 0x08 : 0);

    uint8_t data[8] = {
        mode_byte,
        (uint8_t)(vbat_mv >> 8), (uint8_t)(vbat_mv & 0xFF),
        (uint8_t)((uint16_t)ibat_ma >> 8), (uint8_t)((uint16_t)ibat_ma & 0xFF),
        flags,
        has_bq_st ? bq_st.chrg_stat : 0,
        (uint8_t)app->dsc_pct
    };

    uint32_t tx_id = CAN_ID_STATUS + (addr & 0x3F);
    board_twai_transmit(app->board, tx_id, data, 8, 5);
}

// ---------------------------------------------------------------------------
// Render: CAN remote-control page
// Non-blocking: polls RX queue with 0 timeout each tick.
// ---------------------------------------------------------------------------
static void _render_can_page(app_t *app, uint32_t now_ms)
{
    // ---- RX: drain queue, parse any command addressed to us ----
    if (app->board->has_twai) {
        twai_message_t rx;
        int addr = (int)_get_float(app, "Address", 0);
        while (board_twai_receive(app->board, &rx, 0) == ESP_OK) {
            if (rx.identifier == (uint32_t)(CAN_ID_CMD + (addr & 0x3F))) {
                _can_parse_cmd(&rx, &s_can_cmd);
                s_can_last_rx_ms = now_ms;
            }
        }
    }

    // ---- Map commanded mode → LED backlight ----
    switch (s_can_cmd.mode) {
    case CAN_MODE_CHARGE:    s_led_mode = LED_MODE_CHARGE;    break;
    case CAN_MODE_DISCHARGE: s_led_mode = LED_MODE_DISCHARGE; break;
    case CAN_MODE_MEASURE:   s_led_mode = LED_MODE_MEASURE;   break;
    case CAN_MODE_SETTLE:    s_led_mode = LED_MODE_WAIT;      break;
    default:                 s_led_mode = LED_MODE_WAIT;      break;
    }

    // ---- Act on commanded mode ----
    // CHARGE: CE follows host command; CE off when full
    if (s_can_cmd.mode == CAN_MODE_CHARGE) {
        if (!app->chg_full_latched && !app->chg_error_latched) {
            _set_ce(app, true);
            if (app->board->has_bq) {
                bq25895_set_hiz(&app->board->bq, false);
                bq25895_set_charge_enable(&app->board->bq, true);
            }
        } else {
            _set_ce(app, false);
        }
    } else if (s_can_cmd.mode == CAN_MODE_DISCHARGE) {
        _set_ce(app, false);
        // Apply host-commanded duty, respect vbat cutoff
        float vbat_c = 0.0f; bool vb_c = _read_vbat_v(app, &vbat_c);
        float v_empty = _get_float(app, "DscTo", 3.7f);
        if (!vb_c || vbat_c > v_empty) {
            app->dsc_pct = s_can_cmd.dsc_duty_pct;
            _ledc_set_pct(app->dsc_pct);
        } else {
            _ledc_set_pct(0);
        }
    } else {
        // All other modes: load off, CE off
        _ledc_set_pct(0);
        _set_ce(app, false);
    }

    // ---- Gather sensor data ----
    float vbat = 0.0f; bool vbat_ok = _read_vbat_v(app, &vbat);
    float ibat = 0.0f;
    if (app->board->has_ina_batt) {
        ina226_reading_t r;
        if (ina226_read_all(&app->board->ina_batt, &r) == ESP_OK) ibat = r.current_a;
    }
    bool batt_present = vbat_ok && _battery_present(vbat);

    // Reset IR results if battery absent
    if (!batt_present) {
        app->meas_30 = (meas_result_t){0};
        app->meas_60 = (meas_result_t){0};
    }

    // Check/latch charge states
    if (s_can_cmd.mode == CAN_MODE_CHARGE) {
        bq25895_status_t st = {0};
        bool has_st = _bq_try_read_status(app, &st);
        float vmin = _get_float(app, "ChgFrom", 2.5f);
        // float vtar = _get_float(app, "ChgTo",   4.2f);
        if (has_st && vbat_ok && vbat < (vmin - 1e-6f) && !app->chg_error_latched) {
            _set_ce(app, false); app->chg_error_latched = true;
        }
        if (has_st && st.chrg_stat == 3 && !app->chg_full_latched) {
            _set_ce(app, false); app->chg_full_latched = true;
            // Request mode change to SETTLE via next status TX (flag set in _can_tx_status)
        }
    }

    float t_bq = 0.0f; bool t_bq_ok = _read_bq_temp_c(app, &t_bq);
    int   max_temp = (int)_get_float(app, "MaxTemp", 50);

    // Best IR result: prefer 30%, fall back to 60%
    float ir_uohm = 0.0f; bool ir_valid = false;
    if (app->meas_30.valid) {
        ir_uohm = (app->meas_30.r_mohm - CIRCUIT_R_DISCHG_MOHM) * 1000.0f;
        ir_valid = true;
    } else if (app->meas_60.valid) {
        ir_uohm = (app->meas_60.r_mohm - CIRCUIT_R_DISCHG_MOHM) * 1000.0f;
        ir_valid = true;
    }

    // ---- Title: CAN: <commanded mode> ----
    uint32_t age_ms = (s_can_last_rx_ms == 0) ? 0xFFFFFFFF : (now_ms - s_can_last_rx_ms);
    const char *title_mode;
    if (!app->board->has_twai)       title_mode = "NO BUS";
    else if (s_can_last_rx_ms == 0)  title_mode = "WAITING";
    else if (age_ms > 5000)          title_mode = "OFFLINE";
    else {
        const char *mn[] = {"IDLE","CHARGE","DISC.","MEAS.","WAIT","SETTLE","?","?"};
        title_mode = mn[s_can_cmd.mode & 0x07];
    }

    // ---- S: detail status line ----
    char s_detail[16];
    bool connected = app->board->has_twai && s_can_last_rx_ms != 0 && age_ms <= 5000;
    if (!connected) {
        snprintf(s_detail, sizeof(s_detail), "%s",
                 (s_can_last_rx_ms == 0) ? "WAITING" : "OFFLINE");
    } else {
        switch (s_can_cmd.mode) {
        case CAN_MODE_IDLE:
            snprintf(s_detail, sizeof(s_detail), "IDLE");
            break;
        case CAN_MODE_CHARGE: {
            if (app->chg_error_latched) {
                snprintf(s_detail, sizeof(s_detail), "LOW VOLT");
            } else if (app->chg_full_latched) {
                snprintf(s_detail, sizeof(s_detail), "CHARGED");
            } else if (!batt_present) {
                snprintf(s_detail, sizeof(s_detail), "NO BATT");
            } else {
                bq25895_status_t st = {0};
                bool has_st = _bq_try_read_status(app, &st);
                if (!has_st || !st.power_good) {
                    snprintf(s_detail, sizeof(s_detail), "UNPOWERED");
                } else {
                    float vtar = _get_float(app, "ChgTo", 4.2f);
                    switch (st.chrg_stat) {
                    case 1: snprintf(s_detail, sizeof(s_detail), "PRE-COND");    break;
                    case 2: snprintf(s_detail, sizeof(s_detail), "%s",
                                vbat >= (vtar - 0.03f) ? "CONST.VOLT." : "CONST.CURR."); break;
                    case 3: snprintf(s_detail, sizeof(s_detail), "CHARGED");     break;
                    default: snprintf(s_detail, sizeof(s_detail), "IDLE");       break;
                    }
                }
            }
            break;
        }
        case CAN_MODE_DISCHARGE: {
            int duty = s_can_cmd.dsc_duty_pct;
            if (duty >= 100)
                snprintf(s_detail, sizeof(s_detail), "DISC.FULL");
            else
                snprintf(s_detail, sizeof(s_detail), "DISC.@%d%%", duty);
            break;
        }
        case CAN_MODE_MEASURE:
            // IR measurement via _measure_ir_run is local; status frozen in CAN mode
            snprintf(s_detail, sizeof(s_detail), "MEASURING");
            break;
        case CAN_MODE_SETTLE: {
            uint16_t rem = s_can_cmd.settle_remaining_min;
            if (rem > 0)
                snprintf(s_detail, sizeof(s_detail), "SETTLING %um", rem);
            else
                snprintf(s_detail, sizeof(s_detail), "SETTLING");
            break;
        }
        case CAN_MODE_WAIT:
        default:
            snprintf(s_detail, sizeof(s_detail), "WAITING");
            break;
        }
    }

    // ---- TX: telemetry (rate controlled by host via req_immediate_vbat or 2 Hz) ----
    bool force_tx = connected && s_can_cmd.req_immediate_vbat;
    if (force_tx || (int32_t)(now_ms - s_can_tx_next_ms) >= 0) {
        s_can_tx_next_ms = now_ms + 500;
        uint8_t tx_mode = s_can_cmd.mode;
        // Signal mode change request if charge just completed
        // (bit4 of flags is set inside _can_tx_status via chg_full_latched)
        _can_tx_status(app, tx_mode);
    }

    // ---- Display ----
    display_t *d = app->disp;
    display_clear(d, 0);
    char line[20];

    // Row 0 (y=0): inverted "CAN: <mode>"
    snprintf(line, sizeof(line), "CAN: %s", title_mode);
    _draw_inverted_title(d, line);

    // Row 1 (y=20): "B: 3.87V/+1.23A"
    if (vbat_ok) {
        char sign_i = (ibat >= 0.0f) ? '+' : '-';
        snprintf(line, sizeof(line), "B: %.2fV/%c%.2fA", vbat, sign_i, fabsf(ibat));
    } else {
        snprintf(line, sizeof(line), "B: --V/--A");
    }
    display_draw_str(d, 0, 20, line, false);

    // Row 2 (y=32): "T: 38C of 50C"
    if (t_bq_ok)
        snprintf(line, sizeof(line), "T: %dC of %dC", (int)t_bq, max_temp);
    else
        snprintf(line, sizeof(line), "T: --C of %dC", max_temp);
    display_draw_str(d, 0, 32, line, false);

    // Row 3 (y=44): "IR: 12345uOhm"
    if (ir_valid)
        snprintf(line, sizeof(line), "IR: %duOhm", (int)ir_uohm);
    else
        snprintf(line, sizeof(line), "IR: --");
    display_draw_str(d, 0, 44, line, false);

    // Row 4 (y=56): "S: <detail>"
    snprintf(line, sizeof(line), "S: %s", s_detail);
    display_draw_str(d, 0, 56, line, false);

    display_flush(d);

    // ---- LEDs ----
    update_leds(vbat, vbat_ok, 0, 0, 0, 0, false, false, false, NULL);
}

// ---------------------------------------------------------------------------
// Page enter / exit hooks
// ---------------------------------------------------------------------------
static void _page_enter(app_t *app, app_page_t page, uint32_t now_ms)
{
    switch (page) {
    case PAGE_STATUS:
        s_led_mode = LED_MODE_IDLE;
        _set_fan(app, true);
        app->status_next_draw_ms = now_ms;
        break;

    case PAGE_CHARGE:
        s_led_mode = LED_MODE_CHARGE;
        _set_fan(app, true);
        app->charge_next_draw_ms = now_ms;
        app->charge_configured   = false;
        app->chg_error_latched   = false;
        app->chg_full_latched    = false;
        app->charge_configured   = _apply_bq_for_charge(app);
        break;

    case PAGE_DISCHARGE:
        s_led_mode = LED_MODE_DISCHARGE;
        _set_fan(app, true);
        _set_ce(app, false);
        app->dsc_next_draw_ms = now_ms;
        _ledc_set_pct(app->dsc_pct);
        break;

    case PAGE_MEASURE:
        s_led_mode = LED_MODE_MEASURE;
        _set_fan(app, true);
        _set_ce(app, false);
        _ledc_set_pct(0);
        app->meas_next_draw_ms = now_ms;
        break;

    case PAGE_CAN:
        s_led_mode                    = LED_MODE_WAIT;
        s_can_cmd.mode                = CAN_MODE_WAIT;
        s_can_cmd.dsc_duty_pct        = 30;
        s_can_cmd.dsc_pulse_ms        = 5000;
        s_can_cmd.settle_remaining_min = 0;
        s_can_cmd.req_immediate_vbat  = false;
        s_can_last_rx_ms              = 0;
        s_can_tx_next_ms              = now_ms;
        app->chg_error_latched        = false;
        app->chg_full_latched         = false;
        _set_fan(app, true);
        break;

    default: break;
    }
}

static void _page_exit(app_t *app, app_page_t page)
{
    switch (page) {
    case PAGE_CHARGE:
        _set_fan(app, false);
        _set_ce(app, false);
        app->chg_error_latched = false;
        app->chg_full_latched  = false;
        break;

    case PAGE_DISCHARGE:
        _set_fan(app, false);
        _ledc_set_pct(0);
        break;

    case PAGE_MEASURE:
        _set_fan(app, false);
        _ledc_set_pct(0);
        break;

    case PAGE_STATUS:
        _set_fan(app, false);
        break;

    case PAGE_CAN:
        _set_fan(app, false);
        _ledc_set_pct(0);
        _set_ce(app, false);
        break;

    default: break;
    }
    s_led_mode = LED_MODE_IDLE;
    if (s_led_strip) { led_strip_clear(s_led_strip); led_strip_refresh(s_led_strip); }
}

// ---------------------------------------------------------------------------
// Startup sequence (BQ wake + power detect)
// ---------------------------------------------------------------------------
static void _startup_sequence(app_t *app)
{
    // Try BQ; if absent, pulse QON for 2.5s
    bq25895_status_t st;
    bool bq_up = _bq_try_read_status(app, &st);

    if (!bq_up) {
        _set_qon(app, true);
        uint32_t t0 = ms_now();
        while ((ms_now() - t0) < 2500) {
            float frac = (float)(ms_now() - t0) / 2500.0f;
            _render_startup(app->disp, "", "  INITIALIZING", "", frac);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        _set_qon(app, false);
        app->bq_wake_attempted = true;
         // Re-probe BQ after QON wake — update has_bq flag
        esp_err_t bq_ret = bq25895_init(app->board->i2c_bus, BQ25895_ADDR, &app->board->bq);
        if (bq_ret == ESP_OK) {
            app->board->has_bq = true;
            bq25895_set_charge_enable(&app->board->bq, false);
            bq25895_set_hiz(&app->board->bq, false);
        }
    } else {
        // BQ already up: show 2.5s progress anyway
        uint32_t t0 = ms_now();
        while ((ms_now() - t0) < 2500) {
            float frac = (float)(ms_now() - t0) / 2500.0f;
            _render_startup(app->disp, "", "  INITIALIZING", "", frac);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    // Wait for valid input power (or OK+MENU bypass)
    while (1) {
        float vin = 0.0f;
        if (app->board->has_ina_sys) {
            ina226_reading_t r;
            if (ina226_read_all(&app->board->ina_sys, &r) == ESP_OK) vin = r.bus_v;
        }

        bool ok_down   = (gpio_get_level(GPIO_OK) == 0);
        bool menu_down = app->board->has_exp ?
                         (_exp_level(app, P_BTN1) == 0) : false;
        if (ok_down && menu_down) {
            app->no_input_override = true;
            return;
        }

        if (vin < VIN_DETECT_V) {
            _render_startup(app->disp, "", "Plug in XT-30", "or USB-C PD 12V", -1.0f);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }
        if (vin < VIN_MIN_OK_V) {
            _render_startup(app->disp, "", "FAN NOT AVAILABLE", "LIMITED POWER", -1.0f);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }
        return;
    }
}

// ---------------------------------------------------------------------------
// app_init — populate menu items, init peripherals
// ---------------------------------------------------------------------------
void app_init(app_t *app, board_t *board, display_t *disp)
{
    memset(app, 0, sizeof(*app));
    app->board = board;
    app->disp  = disp;

    app->mode      = MODE_MAIN;
    app->page      = PAGE_NONE;
    app->last_page = PAGE_NONE;

    app->dsc_pct        = 10;

    // Menu items
    int n = 0;
    #define MFLOAT(k, s, lo, hi, v, dp) do { \
        strncpy(app->items[n].key, k, ITEM_KEY_LEN-1); \
        app->items[n].type = ITEM_FLOAT; \
        app->items[n].step = s; app->items[n].vmin = lo; \
        app->items[n].vmax = hi; app->items[n].val = v; \
        app->items[n].decimals = dp; n++; } while(0)
    #define MINT(k, s, lo, hi, v) do { \
        strncpy(app->items[n].key, k, ITEM_KEY_LEN-1); \
        app->items[n].type = ITEM_INT; \
        app->items[n].step = s; app->items[n].vmin = lo; \
        app->items[n].vmax = hi; app->items[n].val = v; \
        app->items[n].decimals = 0; n++; } while(0)

    MINT("Address",  1,    0,     63,    0);
    MFLOAT("ChgTo",  0.01f, 4.00f, 4.30f, 4.20f, 2);
    MFLOAT("DscTo",  0.01f, 3.00f, 4.00f, 3.70f, 2);
    MINT("Capacity", 100,  300,   65500, 10000);
    MINT("MaxTemp",  1,    30,    80,    50);

    // Exec list item
    strncpy(app->items[n].key, "Exec", ITEM_KEY_LEN-1);
    app->items[n].type  = ITEM_LIST;
    app->items[n].nopts = 5;
    app->items[n].opts[0] = "Charge";
    app->items[n].opts[1] = "Discharge";
    app->items[n].opts[2] = "Measure";
    app->items[n].opts[3] = "Status";
    app->items[n].opts[4] = "CAN";
    app->items[n].idx   = 3;  // default: Status
    n++;

    app->num_items = n;

    // Peripheral init
    _temp_sensor_init();
    _ext_adc_init();
    _ledc_init();
    _tach_init(app);
    _leds_strip_init();
}

// ---------------------------------------------------------------------------
// app_run — main loop (never returns)
// ---------------------------------------------------------------------------
void app_run(app_t *app)
{
    const int POLL_MS = 20;

    display_init(app->disp, app->board->lcd_panel);

    // Clear LEDs
    if (s_led_strip) { led_strip_clear(s_led_strip); led_strip_refresh(s_led_strip); }

    // Startup
    _startup_sequence(app);

    // Enter main menu
    app->mode      = MODE_MAIN;
    app->page      = PAGE_NONE;
    app->last_page = PAGE_NONE;
    _render_main(app);

    while (1) {
        uint32_t now = ms_now();
        events_t ev  = _poll_inputs(app);
        s_led_frame++;

        // ----- PAGE mode -----
        if (app->mode == MODE_PAGE) {

            // Page transition
            if (app->last_page != app->page) {
                if (app->last_page != PAGE_NONE) _page_exit(app, app->last_page);
                if (app->page     != PAGE_NONE)  _page_enter(app, app->page, now);
                app->last_page = app->page;
            }

            // MENU exits page
            if (ev.menu) {
                _page_exit(app, app->page);
                app->mode      = MODE_MAIN;
                app->page      = PAGE_NONE;
                app->last_page = PAGE_NONE;
                _render_main(app);
                vTaskDelay(pdMS_TO_TICKS(POLL_MS));
                continue;
            }

            // Render current page
            switch (app->page) {
            case PAGE_STATUS:
                update_leds(0, false, 0, 0, 0, 0, false, false, false, NULL);
                _render_status(app, now);
                if ((int32_t)(now - s_can_tx_next_ms) >= 0) {
                    s_can_tx_next_ms = now + 1000;
                    _can_tx_status(app, CAN_MODE_IDLE);
                }
                break;

            case PAGE_CHARGE: {
                _render_charge(app, now);
                // Read charge state for LED update every tick regardless of render timer
                { float vbat=0.0f; bool vb=_read_vbat_v(app,&vbat);
                  float vmin=_get_float(app,"ChgFrom",2.5f), vtar=_get_float(app,"ChgTo",4.2f);
                  bq25895_status_t st={0}; bool has_st=_bq_try_read_status(app,&st);
                  const char *mode=has_st?_bq_charge_mode(&st,vbat,vtar):"UNP";
                  update_leds(vbat,vb,vmin,vtar,0,0,vb&&_battery_present(vbat),
                              app->chg_error_latched,app->chg_full_latched,mode); }
                if ((int32_t)(now - s_can_tx_next_ms) >= 0) {
                    s_can_tx_next_ms = now + 1000;
                    _can_tx_status(app, CAN_MODE_CHARGE);
                }
                break;
            }

            case PAGE_DISCHARGE:
                if (ev.left || ev.right) {
                    if (ev.left)  app->dsc_pct--;
                    if (ev.right) app->dsc_pct++;
                    if (app->dsc_pct < 10)  app->dsc_pct = 10;
                    if (app->dsc_pct > 100) app->dsc_pct = 100;
                    float vbat = 0.0f; bool vb = _read_vbat_v(app, &vbat);
                    float v_target = _get_float(app, "DscTo", 3.70f);
                    if (!vb || vbat > v_target)
                        _ledc_set_pct(app->dsc_pct);
                    else
                        _ledc_set_pct(0);
                    app->dsc_next_draw_ms = now;
                }
                _render_discharge(app, now);
                { float vbat=0.0f; bool vb=_read_vbat_v(app,&vbat);
                  float vf=_get_float(app,"ChgTo",4.2f), ve=_get_float(app,"DscTo",3.7f);
                  update_leds(vbat, vb, 0, 0, vf, ve, vb && _battery_present(vbat),
                              false, false, NULL); }
                if ((int32_t)(now - s_can_tx_next_ms) >= 0) {
                    s_can_tx_next_ms = now + 1000;
                    _can_tx_status(app, CAN_MODE_DISCHARGE);
                }
                break;

            case PAGE_MEASURE:
                if (ev.left)  _measure_ir_run(app, 30);
                if (ev.right) _measure_ir_run(app, 60);
                _render_measure(app, now);
                update_leds(0, false, 0, 0, 0, 0, false, false, false, NULL);
                if ((int32_t)(now - s_can_tx_next_ms) >= 0) {
                    s_can_tx_next_ms = now + 1000;
                    _can_tx_status(app, CAN_MODE_MEASURE);
                }
                break;

            case PAGE_CAN:
                _render_can_page(app, now);
                break;

            default:
                update_leds(0, false, 0, 0, 0, 0, false, false, false, NULL);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        // ----- MAIN / EDIT mode -----
        s_led_mode = LED_MODE_IDLE;
        update_leds(0, false, 0, 0, 0, 0, false, false, false, NULL);

        if (app->mode == MODE_MAIN) {
            if (ev.left)  _move_sel(app, -1);
            if (ev.right) _move_sel(app, +1);
            if (ev.ok)    _enter_edit(app);
        } else { // EDIT
            if (ev.left)  _step_current(app, -1);
            if (ev.right) _step_current(app, +1);
            if (ev.ok)    _confirm_edit(app);
            if (ev.menu)  _cancel_edit(app);
        }

        _render_main(app);
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}
