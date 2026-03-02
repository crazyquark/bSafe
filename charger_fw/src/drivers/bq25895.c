// =============================================================================
// drivers/bq25895.c — TI BQ25895 switch-mode charger / power-path
// =============================================================================
#include "bq25895.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "bq25895";
#define I2C_TIMEOUT_MS 50

// ---------------------------------------------------------------------------
// Low-level helpers
// ---------------------------------------------------------------------------

static esp_err_t _wr(bq25895_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev->dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t _rd(bq25895_t *dev, uint8_t reg, uint8_t *out)
{
    return i2c_master_transmit_receive(dev->dev, &reg, 1, out, 1, I2C_TIMEOUT_MS);
}

static esp_err_t _update_bits(bq25895_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t cur;
    esp_err_t ret = _rd(dev, reg, &cur);
    if (ret != ESP_OK) return ret;
    uint8_t new = (cur & ~mask) | (value & mask);
    if (new == cur) return ESP_OK;
    return _wr(dev, reg, new);
}

static inline int _clamp(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t bq25895_init(i2c_master_bus_handle_t bus, uint8_t addr, bq25895_t *out)
{
    memset(out, 0, sizeof(*out));
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &out->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BQ25895 at 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }
    // Enable continuous ADC conversion — required for all ADC registers
    // (REG10 TS%, REG0F VSYS, REG11 VBUS, REG12 ICHG) to update continuously.
    // Default power-on state has CONV_RATE=0 (one-shot) and CONV_START=0
    // meaning all ADC registers hold their reset values until triggered.
    _update_bits(out, BQ_REG02, BQ_B_CONV_RATE, BQ_B_CONV_RATE);
    return ESP_OK;
}

esp_err_t bq25895_set_charge_enable(bq25895_t *dev, bool enabled)
{
    return _update_bits(dev, BQ_REG03, BQ_B_CHG_CONFIG,
                        enabled ? BQ_B_CHG_CONFIG : 0);
}

esp_err_t bq25895_set_hiz(bq25895_t *dev, bool enabled)
{
    return _update_bits(dev, BQ_REG00, BQ_B_EN_HIZ,
                        enabled ? BQ_B_EN_HIZ : 0);
}

esp_err_t bq25895_watchdog_kick(bq25895_t *dev)
{
    return _update_bits(dev, BQ_REG03, BQ_B_WD_RST, BQ_B_WD_RST);
}

esp_err_t bq25895_set_input_current_limit_ma(bq25895_t *dev, int ma)
{
    ma = _clamp(ma, 100, 3250);
    int code = _clamp((ma - 100) / 50, 0, 0x3F);
    return _update_bits(dev, BQ_REG00, BQ_M_IINLIM, (uint8_t)code);
}

esp_err_t bq25895_set_fast_charge_current_ma(bq25895_t *dev, int ma)
{
    ma = _clamp(ma, 0, 5056);
    int code = _clamp(ma / 64, 0, 0x7F);
    return _update_bits(dev, BQ_REG04, 0x7F, (uint8_t)code);
}

esp_err_t bq25895_set_precharge_current_ma(bq25895_t *dev, int ma)
{
    ma = _clamp(ma, 64, 1024);
    int code = _clamp((ma - 64) / 64, 0, 0x0F);
    return _update_bits(dev, BQ_REG05, 0xF0, (uint8_t)(code << 4));
}

esp_err_t bq25895_set_termination_current_ma(bq25895_t *dev, int ma)
{
    ma = _clamp(ma, 64, 1024);
    int code = _clamp((ma - 64) / 64, 0, 0x0F);
    return _update_bits(dev, BQ_REG05, 0x0F, (uint8_t)code);
}

esp_err_t bq25895_set_charge_voltage_mv(bq25895_t *dev, int mv)
{
    mv = _clamp(mv, 3840, 4608);
    int code = _clamp((mv - 3840) / 16, 0, 0x3F);
    return _update_bits(dev, BQ_REG06, 0xFC, (uint8_t)(code << 2));
}

esp_err_t bq25895_set_batlowv(bq25895_t *dev, float threshold_v)
{
    // REG06 bit[1]: BATLOWV — 0=2.8V, 1=3.0V
    uint8_t bit = (threshold_v >= 3.0f) ? 0x02 : 0x00;
    return _update_bits(dev, BQ_REG06, 0x02, bit);
}

esp_err_t bq25895_adc_start_oneshot(bq25895_t *dev)
{
    esp_err_t ret = _update_bits(dev, BQ_REG02, BQ_B_CONV_RATE, 0);
    if (ret != ESP_OK) return ret;
    return _update_bits(dev, BQ_REG02, BQ_B_CONV_START, BQ_B_CONV_START);
}

esp_err_t bq25895_adc_set_continuous(bq25895_t *dev, bool enabled)
{
    return _update_bits(dev, BQ_REG02, BQ_B_CONV_RATE,
                        enabled ? BQ_B_CONV_RATE : 0);
}

esp_err_t bq25895_read_vsys_v(bq25895_t *dev, float *out)
{
    uint8_t raw;
    esp_err_t ret = _rd(dev, BQ_REG0F, &raw);
    if (ret == ESP_OK) *out = 2.304f + (float)(raw & 0x7F) * 0.020f;
    return ret;
}

esp_err_t bq25895_read_vbus_v(bq25895_t *dev, float *out)
{
    uint8_t raw;
    esp_err_t ret = _rd(dev, BQ_REG11, &raw);
    if (ret == ESP_OK) *out = 2.6f + (float)(raw & 0x7F) * 0.100f;
    return ret;
}

esp_err_t bq25895_read_vbus_good(bq25895_t *dev, bool *out)
{
    uint8_t raw;
    esp_err_t ret = _rd(dev, BQ_REG11, &raw);
    if (ret == ESP_OK) *out = (raw & 0x80) != 0;
    return ret;
}

esp_err_t bq25895_read_charge_current_ma(bq25895_t *dev, int *out)
{
    uint8_t raw;
    esp_err_t ret = _rd(dev, BQ_REG12, &raw);
    if (ret == ESP_OK) *out = (int)(raw & 0x7F) * 50;
    return ret;
}

esp_err_t bq25895_read_status(bq25895_t *dev, bq25895_status_t *out)
{
    uint8_t v;
    esp_err_t ret = _rd(dev, BQ_REG0B, &v);
    if (ret != ESP_OK) return ret;
    out->raw        = v;
    out->vbus_stat  = (v & BQ_M_VBUS_STAT) >> 5;
    out->chrg_stat  = (v & BQ_M_CHRG_STAT) >> 3;
    out->power_good = (v & BQ_B_PG_STAT)   != 0;
    out->vsys_stat  = (v & BQ_B_VSYS_STAT) != 0;
    return ESP_OK;
}

esp_err_t bq25895_read_faults(bq25895_t *dev, bq25895_faults_t *out)
{
    uint8_t v;
    esp_err_t ret = _rd(dev, BQ_REG0C, &v);
    if (ret != ESP_OK) return ret;
    out->raw            = v;
    out->watchdog_fault = (v & BQ_B_WATCHDOG_FAULT) != 0;
    out->boost_fault    = (v & BQ_B_BOOST_FAULT)    != 0;
    out->chrg_fault     = (v & BQ_M_CHRG_FAULT) >> 4;
    out->bat_fault      = (v & BQ_B_BAT_FAULT)      != 0;
    out->ntc_fault      = (v & BQ_M_NTC_FAULT);
    return ESP_OK;
}

const char *bq25895_chrg_stat_label(uint8_t chrg_stat)
{
    switch (chrg_stat) {
        case 0: return "not_charging";
        case 1: return "precharge";
        case 2: return "fast_charge";
        case 3: return "done";
        default: return "unknown";
    }
}
