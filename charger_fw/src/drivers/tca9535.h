#pragma once
// =============================================================================
// drivers/tca9535.h — TCA9535 16-bit I/O expander
// =============================================================================
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

// Register addresses
#define TCA9535_REG_INPUT_0     0x00
#define TCA9535_REG_INPUT_1     0x01
#define TCA9535_REG_OUTPUT_0    0x02
#define TCA9535_REG_OUTPUT_1    0x03
#define TCA9535_REG_POL_0       0x04
#define TCA9535_REG_POL_1       0x05
#define TCA9535_REG_CONFIG_0    0x06
#define TCA9535_REG_CONFIG_1    0x07

typedef struct {
    i2c_master_dev_handle_t dev;

    uint8_t dir0;   // direction register cache (1=input)
    uint8_t dir1;
    uint8_t out0;   // output register cache
    uint8_t out1;

    uint8_t last_in0; // previous input state for change detection
    uint8_t last_in1;
} tca9535_t;

/**
 * @brief Initialise the TCA9535 handle and attach it to an I2C bus.
 *
 * @param bus     I2C master bus handle from i2c_new_master_bus()
 * @param addr    7-bit I2C address (default 0x20)
 * @param out     Handle struct to initialise
 */
esp_err_t tca9535_init(i2c_master_bus_handle_t bus, uint8_t addr, tca9535_t *out);

/**
 * @brief Configure direction and clear polarity inversion.
 *        dir bit = 1 → input, 0 → output (matching TCA9535 datasheet).
 */
esp_err_t tca9535_configure(tca9535_t *dev, uint8_t dir0, uint8_t dir1);

/** @brief Write a single pin (pin 0..15). */
esp_err_t tca9535_write_pin(tca9535_t *dev, uint8_t pin, bool value);

/** @brief Write both output ports at once. */
esp_err_t tca9535_write_mask(tca9535_t *dev, uint8_t mask0, uint8_t mask1);

/** @brief Read both input ports. */
esp_err_t tca9535_read_all(tca9535_t *dev, uint8_t *in0, uint8_t *in1);

/**
 * @brief Read inputs and compute XOR change masks vs previous read.
 *        Useful for interrupt-driven or polled change detection.
 */
esp_err_t tca9535_read_changes(tca9535_t *dev,
                                uint8_t *chg0, uint8_t *chg1,
                                uint8_t *new0, uint8_t *new1);

/** @brief Helper: return true if a given pin is currently LOW (active-low button logic). */
bool tca9535_pin_is_low(const tca9535_t *dev, uint8_t pin);
