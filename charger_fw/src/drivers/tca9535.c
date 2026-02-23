// =============================================================================
// drivers/tca9535.c — TCA9535 16-bit I/O expander
// =============================================================================
#include "tca9535.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "tca9535";

// I2C timeout for all transactions (ms)
#define I2C_TIMEOUT_MS  50

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static esp_err_t _write_reg(tca9535_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev->dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t _read_reg(tca9535_t *dev, uint8_t reg, uint8_t *out)
{
    return i2c_master_transmit_receive(dev->dev,
                                       &reg, 1,
                                       out,  1,
                                       I2C_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t tca9535_init(i2c_master_bus_handle_t bus, uint8_t addr, tca9535_t *out)
{
    memset(out, 0, sizeof(*out));
    out->dir0 = 0xFF; // all inputs by default
    out->dir1 = 0xFF;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &out->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device at 0x%02X: %s", addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t tca9535_configure(tca9535_t *dev, uint8_t dir0, uint8_t dir1)
{
    esp_err_t ret;

    dev->dir0 = dir0;
    dev->dir1 = dir1;

    // Direction registers
    if ((ret = _write_reg(dev, TCA9535_REG_CONFIG_0, dir0)) != ESP_OK) return ret;
    if ((ret = _write_reg(dev, TCA9535_REG_CONFIG_1, dir1)) != ESP_OK) return ret;

    // Clear polarity inversion
    if ((ret = _write_reg(dev, TCA9535_REG_POL_0, 0x00)) != ESP_OK) return ret;
    if ((ret = _write_reg(dev, TCA9535_REG_POL_1, 0x00)) != ESP_OK) return ret;

    // Write initial output state (0 everywhere)
    if ((ret = _write_reg(dev, TCA9535_REG_OUTPUT_0, dev->out0)) != ESP_OK) return ret;
    if ((ret = _write_reg(dev, TCA9535_REG_OUTPUT_1, dev->out1)) != ESP_OK) return ret;

    // Prime input cache
    _read_reg(dev, TCA9535_REG_INPUT_0, &dev->last_in0);
    _read_reg(dev, TCA9535_REG_INPUT_1, &dev->last_in1);

    return ESP_OK;
}

esp_err_t tca9535_write_pin(tca9535_t *dev, uint8_t pin, bool value)
{
    esp_err_t ret;
    if (pin < 8) {
        if (value) dev->out0 |=  (1u << pin);
        else       dev->out0 &= ~(1u << pin);
        ret = _write_reg(dev, TCA9535_REG_OUTPUT_0, dev->out0);
    } else {
        uint8_t b = pin - 8;
        if (value) dev->out1 |=  (1u << b);
        else       dev->out1 &= ~(1u << b);
        ret = _write_reg(dev, TCA9535_REG_OUTPUT_1, dev->out1);
    }
    return ret;
}

esp_err_t tca9535_write_mask(tca9535_t *dev, uint8_t mask0, uint8_t mask1)
{
    esp_err_t ret;
    dev->out0 = mask0;
    dev->out1 = mask1;
    if ((ret = _write_reg(dev, TCA9535_REG_OUTPUT_0, mask0)) != ESP_OK) return ret;
    return _write_reg(dev, TCA9535_REG_OUTPUT_1, mask1);
}

esp_err_t tca9535_read_all(tca9535_t *dev, uint8_t *in0, uint8_t *in1)
{
    esp_err_t ret;
    if ((ret = _read_reg(dev, TCA9535_REG_INPUT_0, in0)) != ESP_OK) return ret;
    ret = _read_reg(dev, TCA9535_REG_INPUT_1, in1);
    if (ret == ESP_OK) {
        dev->last_in0 = *in0;
        dev->last_in1 = *in1;
    }
    return ret;
}

esp_err_t tca9535_read_changes(tca9535_t *dev,
                                uint8_t *chg0, uint8_t *chg1,
                                uint8_t *new0, uint8_t *new1)
{
    uint8_t n0, n1;
    esp_err_t ret = tca9535_read_all(dev, &n0, &n1);
    if (ret != ESP_OK) return ret;

    *chg0 = n0 ^ dev->last_in0;
    *chg1 = n1 ^ dev->last_in1;
    *new0 = n0;
    *new1 = n1;
    dev->last_in0 = n0;
    dev->last_in1 = n1;
    return ESP_OK;
}

bool tca9535_pin_is_low(const tca9535_t *dev, uint8_t pin)
{
    if (pin < 8) {
        return ((dev->last_in0 >> pin) & 1) == 0;
    } else {
        return ((dev->last_in1 >> (pin - 8)) & 1) == 0;
    }
}
