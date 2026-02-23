#pragma once
// =============================================================================
// drivers/ina226.h — INA226 bi-directional current/power monitor
// =============================================================================
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

// Registers
#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNT_V      0x01
#define INA226_REG_BUS_V        0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIBRATION  0x05
#define INA226_REG_MASK_ENABLE  0x06
#define INA226_REG_ALERT_LIMIT  0x07
#define INA226_REG_MANUF_ID     0xFE
#define INA226_REG_DIE_ID       0xFF

// CONFIG field helpers
// AVG [11:9], VBUSCT [8:6], VSHCT [5:3], MODE [2:0]
typedef enum {
    INA226_AVG_1    = 0b000,
    INA226_AVG_4    = 0b001,
    INA226_AVG_16   = 0b010,
    INA226_AVG_64   = 0b011,
    INA226_AVG_128  = 0b100,
    INA226_AVG_256  = 0b101,
    INA226_AVG_512  = 0b110,
    INA226_AVG_1024 = 0b111,
} ina226_avg_t;

typedef enum {
    INA226_CT_140US  = 0b000,
    INA226_CT_204US  = 0b001,
    INA226_CT_332US  = 0b010,
    INA226_CT_588US  = 0b011,
    INA226_CT_1100US = 0b100,
    INA226_CT_2116US = 0b101,
    INA226_CT_4156US = 0b110,
    INA226_CT_8244US = 0b111,
} ina226_ct_t;

typedef enum {
    INA226_MODE_POWER_DOWN         = 0b000,
    INA226_MODE_SHUNT_TRIG         = 0b001,
    INA226_MODE_BUS_TRIG           = 0b010,
    INA226_MODE_SHUNT_BUS_TRIG     = 0b011,
    INA226_MODE_SHUNT_CONT         = 0b101,
    INA226_MODE_BUS_CONT           = 0b110,
    INA226_MODE_SHUNT_BUS_CONT     = 0b111,
} ina226_mode_t;

// Mask/Enable bits
#define INA226_ME_SOL   (1u << 15)  // Shunt over-voltage
#define INA226_ME_SUL   (1u << 14)  // Shunt under-voltage
#define INA226_ME_BOL   (1u << 13)  // Bus over-voltage
#define INA226_ME_BUL   (1u << 12)  // Bus under-voltage
#define INA226_ME_POL   (1u << 11)  // Power over-limit
#define INA226_ME_CNVR  (1u << 10)  // Conversion ready
#define INA226_ME_AFF   (1u <<  4)  // Alert function flag (RO)
#define INA226_ME_CVRF  (1u <<  3)  // Conversion ready flag (RO)
#define INA226_ME_OVF   (1u <<  2)  // Math overflow (RO)
#define INA226_ME_APOL  (1u <<  1)  // Alert polarity (1=active-high)
#define INA226_ME_LEN   (1u <<  0)  // Latch enable

#define INA226_BUS_LSB_V    1.25e-3f    // V/bit
#define INA226_SHUNT_LSB_V  2.5e-6f    // V/bit (shunt)

typedef struct {
    i2c_master_dev_handle_t dev;
    float r_shunt;          // Ω
    float current_lsb;      // A/bit (set after calibrate_for_max_current)
    float power_lsb;        // W/bit (= 25 * current_lsb)
} ina226_t;

typedef struct {
    float bus_v;
    float shunt_v;
    float current_a;
    float power_w;
} ina226_reading_t;

esp_err_t ina226_init(i2c_master_bus_handle_t bus, uint8_t addr,
                      float r_shunt_ohms, ina226_t *out);

esp_err_t ina226_reset(ina226_t *dev);

esp_err_t ina226_configure(ina226_t *dev,
                            ina226_avg_t avg,
                            ina226_ct_t  vbus_ct,
                            ina226_ct_t  vsh_ct,
                            ina226_mode_t mode);

/**
 * @brief Compute and write calibration register for a given max current.
 *        Must be called before reading current or power.
 */
esp_err_t ina226_calibrate_for_max_current(ina226_t *dev, float max_current_a);

esp_err_t ina226_read_bus_voltage(ina226_t *dev, float *v_out);
esp_err_t ina226_read_shunt_voltage(ina226_t *dev, float *v_out);
esp_err_t ina226_read_current(ina226_t *dev, float *a_out);
esp_err_t ina226_read_power(ina226_t *dev, float *w_out);
esp_err_t ina226_read_all(ina226_t *dev, ina226_reading_t *r);

esp_err_t ina226_clear_alert_latch(ina226_t *dev);
esp_err_t ina226_set_bus_overvoltage_alert(ina226_t *dev, float limit_v,
                                            bool latch, bool active_low);
esp_err_t ina226_set_shunt_overcurrent_alert(ina226_t *dev, float limit_a,
                                              bool latch, bool active_low);

esp_err_t ina226_read_ids(ina226_t *dev, uint16_t *manuf, uint16_t *die);
