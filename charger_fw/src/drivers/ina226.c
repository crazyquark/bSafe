// =============================================================================
// drivers/ina226.c — INA226 bi-directional current/power monitor
// =============================================================================
#include "ina226.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ina226";
#define I2C_TIMEOUT_MS 50

// ---------------------------------------------------------------------------
// Low-level I2C helpers (16-bit big-endian registers)
// ---------------------------------------------------------------------------

static esp_err_t _wr16(ina226_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, (val >> 8) & 0xFF, val & 0xFF};
    return i2c_master_transmit(dev->dev, buf, 3, I2C_TIMEOUT_MS);
}

static esp_err_t _rd16(ina226_t *dev, uint8_t reg, uint16_t *out)
{
    uint8_t raw[2];
    esp_err_t ret = i2c_master_transmit_receive(dev->dev,
                                                 &reg, 1,
                                                 raw,  2,
                                                 I2C_TIMEOUT_MS);
    if (ret == ESP_OK) {
        *out = ((uint16_t)raw[0] << 8) | raw[1];
    }
    return ret;
}

static inline int16_t _to_signed16(uint16_t v)
{
    return (int16_t)v;  // reinterpret — two's-complement is guaranteed in C99+
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t ina226_init(i2c_master_bus_handle_t bus, uint8_t addr,
                      float r_shunt_ohms, ina226_t *out)
{
    memset(out, 0, sizeof(*out));
    out->r_shunt = r_shunt_ohms;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &out->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA226 at 0x%02X: %s", addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ina226_reset(ina226_t *dev)
{
    esp_err_t ret = _wr16(dev, INA226_REG_CONFIG, 1u << 15);
    vTaskDelay(pdMS_TO_TICKS(2));
    return ret;
}

esp_err_t ina226_configure(ina226_t *dev,
                            ina226_avg_t avg,
                            ina226_ct_t  vbus_ct,
                            ina226_ct_t  vsh_ct,
                            ina226_mode_t mode)
{
    uint16_t cfg = ((uint16_t)avg    << 9)
                 | ((uint16_t)vbus_ct << 6)
                 | ((uint16_t)vsh_ct  << 3)
                 | ((uint16_t)mode    & 0x7u);
    return _wr16(dev, INA226_REG_CONFIG, cfg);
}

esp_err_t ina226_calibrate_for_max_current(ina226_t *dev, float max_current_a)
{
    if (max_current_a <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    // ~80% of full-scale: max headroom, best resolution
    float current_lsb = max_current_a / 26214.0f;

    // Round to nearest µA step (match Python logic)
    float step = 1e-6f;
    if (current_lsb > 1e-3f) step = 1e-5f;
    if (current_lsb > 1e-2f) step = 1e-4f;
    current_lsb = roundf(current_lsb / step) * step;

    uint32_t cal = (uint32_t)(0.00512f / (current_lsb * dev->r_shunt));
    if (cal == 0 || cal > 0xFFFF) {
        ESP_LOGE(TAG, "Calibration out of range: cal=%" PRIu32, cal);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = _wr16(dev, INA226_REG_CALIBRATION, (uint16_t)cal);
    if (ret == ESP_OK) {
        dev->current_lsb = current_lsb;
        dev->power_lsb   = 25.0f * current_lsb;
    }
    return ret;
}

esp_err_t ina226_read_bus_voltage(ina226_t *dev, float *v_out)
{
    uint16_t raw;
    esp_err_t ret = _rd16(dev, INA226_REG_BUS_V, &raw);
    if (ret == ESP_OK) *v_out = (float)raw * INA226_BUS_LSB_V;
    return ret;
}

esp_err_t ina226_read_shunt_voltage(ina226_t *dev, float *v_out)
{
    uint16_t raw;
    esp_err_t ret = _rd16(dev, INA226_REG_SHUNT_V, &raw);
    if (ret == ESP_OK) *v_out = (float)_to_signed16(raw) * INA226_SHUNT_LSB_V;
    return ret;
}

esp_err_t ina226_read_current(ina226_t *dev, float *a_out)
{
    if (dev->current_lsb == 0.0f) {
        ESP_LOGE(TAG, "INA226 not calibrated");
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t raw;
    esp_err_t ret = _rd16(dev, INA226_REG_CURRENT, &raw);
    if (ret == ESP_OK) *a_out = (float)_to_signed16(raw) * dev->current_lsb;
    return ret;
}

esp_err_t ina226_read_power(ina226_t *dev, float *w_out)
{
    if (dev->power_lsb == 0.0f) {
        ESP_LOGE(TAG, "INA226 not calibrated");
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t raw;
    esp_err_t ret = _rd16(dev, INA226_REG_POWER, &raw);
    if (ret == ESP_OK) *w_out = (float)raw * dev->power_lsb;
    return ret;
}

esp_err_t ina226_read_all(ina226_t *dev, ina226_reading_t *r)
{
    esp_err_t ret;
    if ((ret = ina226_read_bus_voltage (dev, &r->bus_v))    != ESP_OK) return ret;
    if ((ret = ina226_read_shunt_voltage(dev, &r->shunt_v)) != ESP_OK) return ret;
    if ((ret = ina226_read_current      (dev, &r->current_a)) != ESP_OK) return ret;
    return     ina226_read_power       (dev, &r->power_w);
}

esp_err_t ina226_clear_alert_latch(ina226_t *dev)
{
    uint16_t dummy;
    return _rd16(dev, INA226_REG_MASK_ENABLE, &dummy);
}

static esp_err_t _write_mask_enable(ina226_t *dev, uint16_t enable_bits,
                                     bool latch, bool active_low)
{
    uint16_t mask = enable_bits & 0xFC00u;
    if (latch)       mask |= INA226_ME_LEN;
    if (!active_low) mask |= INA226_ME_APOL; // active-high
    return _wr16(dev, INA226_REG_MASK_ENABLE, mask);
}

esp_err_t ina226_set_bus_overvoltage_alert(ina226_t *dev, float limit_v,
                                            bool latch, bool active_low)
{
    uint16_t limit = (uint16_t)(limit_v / INA226_BUS_LSB_V);
    esp_err_t ret = _wr16(dev, INA226_REG_ALERT_LIMIT, limit);
    if (ret != ESP_OK) return ret;
    return _write_mask_enable(dev, INA226_ME_BOL, latch, active_low);
}

esp_err_t ina226_set_shunt_overcurrent_alert(ina226_t *dev, float limit_a,
                                              bool latch, bool active_low)
{
    float v_limit = limit_a * dev->r_shunt;
    uint16_t raw  = (uint16_t)(v_limit / INA226_SHUNT_LSB_V);
    esp_err_t ret = _wr16(dev, INA226_REG_ALERT_LIMIT, raw);
    if (ret != ESP_OK) return ret;
    return _write_mask_enable(dev, INA226_ME_SOL, latch, active_low);
}

esp_err_t ina226_read_ids(ina226_t *dev, uint16_t *manuf, uint16_t *die)
{
    esp_err_t ret;
    if ((ret = _rd16(dev, INA226_REG_MANUF_ID, manuf)) != ESP_OK) return ret;
    return _rd16(dev, INA226_REG_DIE_ID, die);
}
