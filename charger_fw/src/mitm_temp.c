// =============================================================================
// mitm_temp.c — Temperature differential diagnostic
//
// Drop this file into your project and call mitm_temp_task() from app_main()
// BEFORE app_run(), or spawn it as a FreeRTOS task.
//
// Wiring: AHT20+BMP280 board soldered in parallel with the OLED
//   SDA → GPIO5, SCL → GPIO6 (same bus, no config changes needed)
//   AHT20: 0x38, BMP280: 0x76
//
// External NTC: 10k pull-up to 3V3, NTC to GND, ADC channel 1 (GPIO1)
// =============================================================================

#include "mitm_temp.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "display.h"

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mitm_temp";

// ---------------------------------------------------------------------------
// NTC spline — fitted to measured points:
//   R(Ω)   T(°C)
//   10600   51
//   15000   39
//   17000   33
//   18400   26
// ---------------------------------------------------------------------------
#define NTC_PULL_OHM    10000.0f
#define NTC_ADC_FULL    4095.0f

#define SH_A  (1.668575e-03f)
#define SH_B  (1.321981e-04f)
#define SH_C  (3.128401e-07f)

// ---------------------------------------------------------------------------
// AHT20 — bare I2C driver
// Datasheet: ASAIR AHT20, addr 0x38
// ---------------------------------------------------------------------------
#define AHT20_ADDR          0x38
#define AHT20_CMD_INIT      0xBE
#define AHT20_CMD_TRIGGER   0xAC
#define AHT20_CMD_SOFTRESET 0xBA
#define AHT20_INIT_PARAM0   0x08
#define AHT20_INIT_PARAM1   0x00
#define AHT20_TRIG_PARAM0   0x33
#define AHT20_TRIG_PARAM1   0x00
#define AHT20_TIMEOUT_MS    100

typedef struct {
    i2c_master_dev_handle_t dev;
    bool ready;
} aht20_t;

static esp_err_t _aht20_init(i2c_master_bus_handle_t bus, aht20_t *a)
{
    a->ready = false;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AHT20_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &a->dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Soft reset
    uint8_t rst = AHT20_CMD_SOFTRESET;
    i2c_master_transmit(a->dev, &rst, 1, AHT20_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Init / calibrate command
    uint8_t init_cmd[] = {AHT20_CMD_INIT, AHT20_INIT_PARAM0, AHT20_INIT_PARAM1};
    ret = i2c_master_transmit(a->dev, init_cmd, sizeof(init_cmd), AHT20_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 init cmd failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    a->ready = true;
    ESP_LOGI(TAG, "AHT20 OK");
    return ESP_OK;
}

// Returns true and fills *temp_c, *rh_pct on success
static bool _aht20_read(aht20_t *a, float *temp_c, float *rh_pct)
{
    if (!a->ready) return false;

    // Trigger measurement
    uint8_t trig[] = {AHT20_CMD_TRIGGER, AHT20_TRIG_PARAM0, AHT20_TRIG_PARAM1};
    if (i2c_master_transmit(a->dev, trig, sizeof(trig), AHT20_TIMEOUT_MS) != ESP_OK)
        return false;

    vTaskDelay(pdMS_TO_TICKS(80)); // measurement time per datasheet

    // Read 6 bytes: status + hum(20b) + temp(20b)
    uint8_t buf[6] = {0};
    if (i2c_master_receive(a->dev, buf, sizeof(buf), AHT20_TIMEOUT_MS) != ESP_OK)
        return false;

    // Bit 7 of status byte: busy flag
    if (buf[0] & 0x80) return false;

    uint32_t raw_hum  = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t raw_temp = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    *rh_pct = (float)raw_hum  / 1048576.0f * 100.0f;
    *temp_c = (float)raw_temp / 1048576.0f * 200.0f - 50.0f;
    return true;
}

// ---------------------------------------------------------------------------
// BMP280 — bare I2C driver
// Datasheet: Bosch BMP280, addr 0x76 (SDO→GND) or 0x77 (SDO→VCC)
// ---------------------------------------------------------------------------
#define BMP280_ADDR         0x77   // change to 0x77 if SDO pulled high
#define BMP280_REG_ID       0xD0
#define BMP280_REG_RESET    0xE0
#define BMP280_REG_CTRL     0xF4
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_PRESS    0xF7   // 3 bytes
#define BMP280_REG_TEMP     0xFA   // 3 bytes
#define BMP280_REG_CALIB    0x88   // 24 bytes (T1..P9)
#define BMP280_CHIP_ID      0x60   // BMP280=0x58, BME280=0x60 — module may be BME280
#define BMP280_TIMEOUT_MS   50

typedef struct {
    i2c_master_dev_handle_t dev;
    bool ready;
    // Trimming params T1..T3
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
} bmp280_t;

static esp_err_t _bmp280_write_reg(bmp280_t *b, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(b->dev, buf, 2, BMP280_TIMEOUT_MS);
}

static esp_err_t _bmp280_read_regs(bmp280_t *b, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(b->dev, &reg, 1, out, len, BMP280_TIMEOUT_MS);
}

static esp_err_t _bmp280_init(i2c_master_bus_handle_t bus, bmp280_t *b)
{
    b->ready = false;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BMP280_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &b->dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BMP280 add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check chip ID — BMP280=0x58, BME280=0x60; both work for temp
    uint8_t id = 0;
    ret = _bmp280_read_regs(b, BMP280_REG_ID, &id, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BMP280 ID read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if (id != 0x58 && id != 0x60) {
        ESP_LOGW(TAG, "BMP280 unexpected chip ID: 0x%02X", id);
        // Don't fail — proceed anyway, calibration may still work
    }
    ESP_LOGI(TAG, "BMP280/BME280 chip ID: 0x%02X", id);

    // Reset
    _bmp280_write_reg(b, BMP280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read trimming parameters T1..T3 (bytes 0x88..0x8D)
    uint8_t calib[6] = {0};
    ret = _bmp280_read_regs(b, BMP280_REG_CALIB, calib, 6);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BMP280 calib read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    b->dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    b->dig_T2 = (int16_t) (calib[3] << 8 | calib[2]);
    b->dig_T3 = (int16_t) (calib[5] << 8 | calib[4]);

    // ctrl_meas: osrs_t=x2 (011), osrs_p=skip (000), mode=forced (01) → 0x61
    // We'll re-trigger each read with forced mode, so set normal standby config here
    _bmp280_write_reg(b, BMP280_REG_CONFIG, 0x00); // no filter, 0.5ms standby

    b->ready = true;
    ESP_LOGI(TAG, "BMP280 OK (T1=%u T2=%d T3=%d)", b->dig_T1, b->dig_T2, b->dig_T3);
    return ESP_OK;
}

// Bosch compensate_temperature_double() ported to float
static float _bmp280_compensate_temp(bmp280_t *b, int32_t adc_T, int32_t *t_fine_out)
{
    float var1 = ((float)adc_T / 16384.0f - (float)b->dig_T1 / 1024.0f)
                 * (float)b->dig_T2;
    float var2 = (((float)adc_T / 131072.0f - (float)b->dig_T1 / 8192.0f)
                  * ((float)adc_T / 131072.0f - (float)b->dig_T1 / 8192.0f))
                 * (float)b->dig_T3;
    if (t_fine_out) *t_fine_out = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

static bool _bmp280_read_temp(bmp280_t *b, float *temp_c)
{
    if (!b->ready) return false;

    // Trigger forced mode measurement: osrs_t=x2, osrs_p=skip, mode=forced
    _bmp280_write_reg(b, BMP280_REG_CTRL, 0x61);
    vTaskDelay(pdMS_TO_TICKS(10)); // forced mode measurement time

    uint8_t raw[3] = {0};
    if (_bmp280_read_regs(b, BMP280_REG_TEMP, raw, 3) != ESP_OK)
        return false;

    int32_t adc_T = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    *temp_c = _bmp280_compensate_temp(b, adc_T, NULL);
    return true;
}

// ---------------------------------------------------------------------------
// ADC — NTC voltage + resistance
//
// ADC_UNIT_1 can only have one owner — app.c's _ext_adc_init() already
// calls adc_oneshot_new_unit(). A second call here would fail with
// ESP_ERR_INVALID_STATE and silently produce dashes on the OLED.
//
// mitm_temp_run() therefore accepts the handle directly from the caller
// (via mitm_temp.h: pass app's s_adc handle).  The channel is already
// configured as ADC_CHANNEL_1 / ADC_ATTEN_DB_12 / 12-bit in app.c —
// no re-configuration needed.
// ---------------------------------------------------------------------------

// Returns raw voltage in volts and approximate NTC resistance.
// Returns false if handle is NULL, or ADC reading is rail-to-rail.
static bool _ntc_read(adc_oneshot_unit_handle_t adc, float *v_out, float *r_out)
{
    if (!adc) return false;

    int64_t acc = 0;
    for (int i = 0; i < 20; i++) {
        int raw = 0;
        adc_oneshot_read(adc, ADC_CHANNEL_1, &raw);
        acc += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    float adcf = (float)acc / 20.0f;
    if (adcf <= 5.0f || adcf >= 4090.0f) return false;

    // V = 3.3 * (adc / 4095)
    *v_out = 3.3f * (adcf / NTC_ADC_FULL);

    // Divider: pull-up to 3V3, NTC to GND → R_ntc = R_pull * adc / (4095 - adc)
    float r = NTC_PULL_OHM * (adcf / (NTC_ADC_FULL - adcf));
    if (r <= 1.0f) return false;
    *r_out = r;
    return true;
}

// ---------------------------------------------------------------------------
// OLED rendering — 4 rows on 128x64 (font ~8px tall, rows at y=0,16,32,48)
// ---------------------------------------------------------------------------
#define ROW0_Y  0
#define ROW1_Y  16
#define ROW2_Y  32
#define ROW3_Y  48

static void _oled_render(display_t *disp,
                          float aht_t,  bool aht_ok,
                          float bmp_t,  bool bmp_ok,
                          float ntc_v,  float ntc_r, bool ntc_ok)
{
    char buf[24];
    display_clear(disp, 0);

    // Row 0 — AHT20 temperature
    if (aht_ok)
        snprintf(buf, sizeof(buf), "AHT: %.1f C", aht_t);
    else
        snprintf(buf, sizeof(buf), "AHT: ---");
    display_draw_str(disp, 0, ROW0_Y, buf, false);

    // Row 1 — BMP280 temperature
    if (bmp_ok)
        snprintf(buf, sizeof(buf), "BMP: %.1f C", bmp_t);
    else
        snprintf(buf, sizeof(buf), "BMP: ---");
    display_draw_str(disp, 0, ROW1_Y, buf, false);

    // Row 2 — NTC voltage
    if (ntc_ok)
        snprintf(buf, sizeof(buf), "NTC: %.3f V", ntc_v);
    else
        snprintf(buf, sizeof(buf), "NTC: ---");
    display_draw_str(disp, 0, ROW2_Y, buf, false);

    // Row 3 — NTC approx resistance
    if (ntc_ok) {
        if (ntc_r >= 1000.0f)
            snprintf(buf, sizeof(buf), "R~: %.1f k", ntc_r / 1000.0f);
        else
            snprintf(buf, sizeof(buf), "R~: %.0f ohm", ntc_r);
    } else {
        snprintf(buf, sizeof(buf), "R~: ---");
    }
    display_draw_str(disp, 0, ROW3_Y, buf, false);

    display_flush(disp);
}

// ---------------------------------------------------------------------------
// Public entry point
// Call mitm_temp_run() from app_main() instead of (or before) app_run().
// It never returns.
// ---------------------------------------------------------------------------
void mitm_temp_run(board_t *board, display_t *disp, adc_oneshot_unit_handle_t adc)
{
    ESP_LOGI(TAG, "=== MITM TEMP DIAGNOSTIC START ===");

    // --- Init AHT20 ---
    aht20_t aht = {0};
    bool aht_present = (_aht20_init(board->i2c_bus, &aht) == ESP_OK);

    // --- Init BMP280 ---
    bmp280_t bmp = {0};
    bool bmp_present = (_bmp280_init(board->i2c_bus, &bmp) == ESP_OK);

    if (!aht_present) ESP_LOGW(TAG, "AHT20 not found — check wiring");
    if (!bmp_present) ESP_LOGW(TAG, "BMP280 not found — check wiring / try addr 0x77");
    if (!adc)         ESP_LOGW(TAG, "NTC ADC handle NULL — pass app's s_adc");

    // --- Infinite 1 Hz loop ---
    for (;;) {
        float aht_temp = 0.0f, aht_rh = 0.0f;
        float bmp_temp = 0.0f;
        float ntc_v    = 0.0f, ntc_r = 0.0f;

        bool aht_ok = aht_present && _aht20_read(&aht, &aht_temp, &aht_rh);
        bool bmp_ok = bmp_present && _bmp280_read_temp(&bmp, &bmp_temp);
        bool ntc_ok = _ntc_read(adc, &ntc_v, &ntc_r);

        // Compute S-H temperature estimate
        float ntc_est = 0.0f;
        if (ntc_ok) {
            float lnR   = logf(ntc_r);
            float inv_t = SH_A + SH_B * lnR + SH_C * lnR * lnR * lnR;
            ntc_est = (inv_t > 0.0f) ? (1.0f / inv_t) - 273.15f : 0.0f;
        }

        // Console output
        ESP_LOGI(TAG,
            "AHT: %s%.1f°C  BMP: %s%.1f°C  NTC: %s%.3fV / ~%.0fΩ (~%.1f°C)",
            aht_ok ? "" : "ERR / ", aht_ok ? aht_temp : 0.0f,
            bmp_ok ? "" : "ERR / ", bmp_ok ? bmp_temp : 0.0f,
            ntc_ok ? "" : "ERR / ", ntc_ok ? ntc_v   : 0.0f,
            ntc_ok ? ntc_r  : 0.0f,
            ntc_ok ? ntc_est: 0.0f);

        // if (aht_ok)
        //     ESP_LOGI(TAG, "AHT RH: %.1f%%", aht_rh);

        // // Differential summary when all three are valid
        // if (aht_ok && bmp_ok && ntc_ok) {
        //     ESP_LOGI(TAG, "DIFF  AHT-BMP: %+.1f°C  AHT-NTC: %+.1f°C  BMP-NTC: %+.1f°C",
        //         aht_temp - bmp_temp,
        //         aht_temp - ntc_est,
        //         bmp_temp - ntc_est);
        // }

        // OLED
        if (board->has_lcd)
            _oled_render(disp, aht_temp, aht_ok, bmp_temp, bmp_ok, ntc_v, ntc_r, ntc_ok);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
