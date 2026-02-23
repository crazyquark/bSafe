// =============================================================================
// app.c — Port of main.py App class to ESP-IDF C
// =============================================================================
#include "app.h"
#include "config.h"
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
// SK6812 NeoPixel via RMT (led_strip component)
// SK6812 is GRB order — led_strip_set_pixel takes (r, g, b) and the
// LED_PIXEL_FORMAT_GRB encoding handles the wire order automatically.
// ---------------------------------------------------------------------------
#include "led_strip.h"

static led_strip_handle_t s_led_strip = NULL;

static void _leds_strip_init(void)
{
    // v3 API: color format is set via led_color_component_format_t, not led_pixel_format
    led_strip_config_t strip_cfg = {
        .strip_gpio_num   = LED_GPIO,
        .max_leds         = LED_COUNT,
        .led_model        = LED_MODEL_SK6812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src            = RMT_CLK_SRC_DEFAULT,
        .resolution_hz      = 10 * 1000 * 1000,
        .mem_block_symbols  = 64,
        .flags.with_dma     = false,
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

static void _leds_push(app_t *app)
{
    if (!s_led_strip) return;
    for (int i = 0; i < LED_COUNT; i++) {
        // led_color_t stores {g, r, b} — pass as r, g, b to the component
        led_strip_set_pixel(s_led_strip, i,
                            app->leds[i].r,
                            app->leds[i].g,
                            app->leds[i].b);
    }
    led_strip_refresh(s_led_strip);
}

static const led_color_t LED_OFF  = {LED_OFF_G,  LED_OFF_R,  LED_OFF_B};
static const led_color_t LED_CHG  = {LED_CHG_G,  LED_CHG_R,  LED_CHG_B};
static const led_color_t LED_ERR  = {LED_ERR_G,  LED_ERR_R,  LED_ERR_B};
static const led_color_t LED_DSC  = {LED_DSC_G,  LED_DSC_R,  LED_DSC_B};
static const led_color_t LED_IDLE = {LED_IDLE_G, LED_IDLE_R, LED_IDLE_B};

static void _leds_fill(app_t *app, led_color_t c)
{
    for (int i = 0; i < LED_COUNT; i++) app->leds[i] = c;
}

static void _leds_apply_5(app_t *app, led_color_t c[5])
{
    for (int i = 0; i < 5; i++) app->leds[i] = c[i];
    _leds_push(app);
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
        app->mode = MODE_PAGE;
    } else {
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

static void _bq_wake_once(app_t *app)
{
    if (app->bq_wake_attempted) return;
    app->bq_wake_attempted = true;

    _set_qon(app, true);
    uint32_t t0 = ms_now();
    while ((ms_now() - t0) < 2500) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    _set_qon(app, false);

    // Re-init BQ after wake — re-register device on the existing bus
    if (app->board->has_bq)
        bq25895_init(app->board->i2c_bus, 0x6A, &app->board->bq);
}

static bool _read_vbat_v(app_t *app, float *out)
{
    if (!app->board->has_ina_batt) return false;
    ina226_reading_t r;
    if (ina226_read_all(&app->board->ina_batt, &r) != ESP_OK) return false;
    *out = r.bus_v;
    return true;
}

static bool _battery_present(float vbat) { return vbat >= VBAT_PRESENT_V; }

static float _estimate_pct(float vbat, float vmin, float vmax)
{
    if (vmax <= vmin) return 0.0f;
    float x = (vbat - vmin) / (vmax - vmin);
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x * 100.0f;
}

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
// LED logic
// ---------------------------------------------------------------------------
static void _blink_tick(app_t *app, uint32_t now_ms)
{
    if ((now_ms - app->led_blink_next_ms) <= 0x80000000UL) {
        app->led_blink_on       = !app->led_blink_on;
        app->led_blink_next_ms  = now_ms + LED_BLINK_MS;
    }
}

static void _idle_led_update(app_t *app, uint32_t now_ms)
{
    if ((now_ms - app->idle_led_next_ms) > 0x80000000UL) return;
    app->idle_led_next_ms = now_ms + 120;
    _leds_fill(app, LED_OFF);
    app->leds[app->idle_led_pos] = LED_IDLE;
    _leds_push(app);
    app->idle_led_pos += app->idle_led_dir;
    if (app->idle_led_pos >= 4) { app->idle_led_pos = 4; app->idle_led_dir = -1; }
    else if (app->idle_led_pos <= 0) { app->idle_led_pos = 0; app->idle_led_dir = 1; }
}

static void _charge_led_update(app_t *app, uint32_t now_ms,
                               float vbat, bool vbat_valid,
                               float vmin, float vmax,
                               const char *mode)
{
    _blink_tick(app, now_ms);
    led_color_t pat[5];

    if (vbat_valid && vbat < (vmin - 1e-6f)) {
        if (!app->chg_error_latched) { _set_ce(app, false); app->chg_error_latched = true; }
        led_color_t c = app->led_blink_on ? LED_ERR : LED_OFF;
        for (int i = 0; i < 5; i++) pat[i] = c;
        _leds_apply_5(app, pat);
        return;
    }

    if (strcmp(mode, "DONE") == 0) {
        if (!app->chg_full_latched) { _set_ce(app, false); app->chg_full_latched = true; }
        led_color_t c = app->led_blink_on ? LED_CHG : LED_OFF;
        for (int i = 0; i < 5; i++) pat[i] = c;
        _leds_apply_5(app, pat);
        return;
    }

    if (app->chg_error_latched) {
        led_color_t c = app->led_blink_on ? LED_ERR : LED_OFF;
        for (int i = 0; i < 5; i++) pat[i] = c;
        _leds_apply_5(app, pat); return;
    }
    if (app->chg_full_latched) {
        led_color_t c = app->led_blink_on ? LED_CHG : LED_OFF;
        for (int i = 0; i < 5; i++) pat[i] = c;
        _leds_apply_5(app, pat); return;
    }

    if (!vbat_valid || !_battery_present(vbat)) {
        for (int i = 0; i < 5; i++) pat[i] = LED_OFF;
        _leds_apply_5(app, pat); return;
    }

    if (strcmp(mode, "PRE") == 0) {
        for (int i = 0; i < 5; i++) pat[i] = LED_OFF;
        pat[0] = app->led_blink_on ? LED_CHG : LED_OFF;
        _leds_apply_5(app, pat); return;
    }

    float pct = _estimate_pct(vbat, vmin, vmax);
    int solid_on, blink_idx;
    if      (pct < 25.0f) { solid_on = 1; blink_idx = 1; }
    else if (pct < 50.0f) { solid_on = 2; blink_idx = 2; }
    else if (pct < 75.0f) { solid_on = 3; blink_idx = 3; }
    else                  { solid_on = 4; blink_idx = 4; }

    for (int i = 0; i < 5; i++) pat[i] = LED_OFF;
    for (int i = 0; i < solid_on; i++) pat[i] = LED_CHG;
    pat[blink_idx] = app->led_blink_on ? LED_CHG : LED_OFF;
    _leds_apply_5(app, pat);
}

static void _discharge_led_update(app_t *app, uint32_t now_ms,
                                  float vbat, bool vbat_valid,
                                  float v_full, float v_empty)
{
    _blink_tick(app, now_ms);
    led_color_t pat[5];

    if (!vbat_valid) {
        for (int i = 0; i < 5; i++) pat[i] = LED_OFF;
        _leds_apply_5(app, pat); return;
    }
    if (v_full <= v_empty) {
        led_color_t c = app->led_blink_on ? LED_ERR : LED_OFF;
        for (int i = 0; i < 5; i++) pat[i] = c;
        _leds_apply_5(app, pat); return;
    }

    float dsc_pct;
    if      (vbat >= v_full)  dsc_pct = 0.0f;
    else if (vbat <= v_empty) dsc_pct = 100.0f;
    else                      dsc_pct = (v_full - vbat) / (v_full - v_empty) * 100.0f;

    int solid_on, blink_idx;
    if      (dsc_pct < 25.0f) { solid_on = 1; blink_idx = 1; }
    else if (dsc_pct < 50.0f) { solid_on = 2; blink_idx = 2; }
    else if (dsc_pct < 75.0f) { solid_on = 3; blink_idx = 3; }
    else                      { solid_on = 4; blink_idx = 4; }

    for (int i = 0; i < 5; i++) pat[i] = LED_OFF;
    for (int i = 0; i < solid_on; i++) pat[i] = LED_DSC;
    pat[blink_idx] = app->led_blink_on ? LED_DSC : LED_OFF;
    _leds_apply_5(app, pat);
}

// ---------------------------------------------------------------------------
// OLED rendering helpers
// ---------------------------------------------------------------------------
static void _draw_inverted_title(display_t *d, const char *text)
{
    // 18px tall title bar (rows 0+1), centred
    display_fill_rect(d, 0, 0, DISP_W, 18, true);
    int x = 64 - (int)(strlen(text) * 4);
    if (x < 0) x = 0;
    display_draw_str(d, x, 5, text, true);
}

static void _draw_inverted_text(display_t *d, int x, int y, const char *text)
{
    int w = (int)(strlen(text) * 8);
    display_fill_rect(d, x, y, w, 8, true);
    display_draw_str(d, x, y, text, true);
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

    // LEDs
    if (!has_st || !_bq_is_powered(app)) {
        _blink_tick(app, now_ms);
        led_color_t c = app->led_blink_on ? LED_ERR : LED_OFF;
        led_color_t pat[5]; for (int i=0;i<5;i++) pat[i]=c;
        _leds_apply_5(app, pat);
    } else {
        _charge_led_update(app, now_ms, vbat, vbat_ok, vmin, vtar, mode);
    }
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
// Page enter / exit hooks
// ---------------------------------------------------------------------------
static void _page_enter(app_t *app, app_page_t page, uint32_t now_ms)
{
    switch (page) {
    case PAGE_STATUS:
        _set_fan(app, true);
        app->status_next_draw_ms = now_ms;
        break;

    case PAGE_CHARGE:
        _set_fan(app, true);
        app->charge_next_draw_ms = now_ms;
        app->charge_configured   = false;
        app->chg_error_latched   = false;
        app->chg_full_latched    = false;
        app->led_blink_on        = false;
        app->led_blink_next_ms   = now_ms;
        app->charge_configured   = _apply_bq_for_charge(app);
        break;

    case PAGE_DISCHARGE:
        _set_fan(app, true);
        _set_ce(app, false);   // disable charger while discharging
        app->dsc_next_draw_ms = now_ms;
        _ledc_set_pct(app->dsc_pct);
        break;

    case PAGE_MEASURE:
        _set_fan(app, true);
        _set_ce(app, false);
        _ledc_set_pct(0);
        app->meas_next_draw_ms = now_ms;
        break;

    default: break;
    }
}

static void _page_exit(app_t *app, app_page_t page)
{
    switch (page) {
    case PAGE_STATUS:
        _set_fan(app, false);
        break;

    case PAGE_CHARGE:
        _set_fan(app, false);
        _set_ce(app, false);
        app->chg_error_latched = false;
        app->chg_full_latched  = false;
        { led_color_t pat[5]; for(int i=0;i<5;i++) pat[i]=LED_OFF; _leds_apply_5(app,pat); }
        break;

    case PAGE_DISCHARGE:
        _set_fan(app, false);
        _ledc_set_pct(0);
        { led_color_t pat[5]; for(int i=0;i<5;i++) pat[i]=LED_OFF; _leds_apply_5(app,pat); }
        break;

    case PAGE_MEASURE:
        _set_fan(app, false);
        _ledc_set_pct(0);
        break;

    default: break;
    }
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
        if (app->board->has_bq) bq25895_init(app->board->i2c_bus, 0x6A, &app->board->bq);
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
    app->idle_led_pos   = 0;
    app->idle_led_dir   = 1;
    app->idle_led_next_ms = 0;

    // Menu items (mirrors Python self.items)
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
    app->items[n].nopts = 4;
    app->items[n].opts[0] = "Charge";
    app->items[n].opts[1] = "Discharge";
    app->items[n].opts[2] = "Measure";
    app->items[n].opts[3] = "Status";
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
    _leds_fill(app, LED_OFF);
    _leds_push(app);

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
                _idle_led_update(app, now);
                _render_status(app, now);
                break;

            case PAGE_CHARGE:
                _render_charge(app, now);
                break;

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
                    app->dsc_next_draw_ms = now; // force immediate redraw
                }
                _render_discharge(app, now);
                { float vbat=0.0f; bool vb=_read_vbat_v(app,&vbat);
                  float vf=_get_float(app,"ChgTo",4.2f), ve=_get_float(app,"DscTo",3.7f);
                  _discharge_led_update(app, now, vbat, vb, vf, ve); }
                break;

            case PAGE_MEASURE:
                if (ev.left)  _measure_ir_run(app, 30);
                if (ev.right) _measure_ir_run(app, 60);
                _render_measure(app, now);
                break;

            default:
                _idle_led_update(app, now);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            continue;
        }

        // ----- MAIN / EDIT mode -----
        _idle_led_update(app, now);

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
