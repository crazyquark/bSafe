#pragma once
// =============================================================================
// drivers/bq25895.h — TI BQ25895 switch-mode charger / power-path
// =============================================================================
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

// Register map
#define BQ_REG00  0x00
#define BQ_REG01  0x01
#define BQ_REG02  0x02
#define BQ_REG03  0x03
#define BQ_REG04  0x04
#define BQ_REG05  0x05
#define BQ_REG06  0x06
#define BQ_REG07  0x07
#define BQ_REG0B  0x0B  // status
#define BQ_REG0C  0x0C  // faults
#define BQ_REG0F  0x0F  // VSYS ADC
#define BQ_REG10  0x10  // TS% ADC
#define BQ_REG11  0x11  // VBUS ADC + VBUS_GD
#define BQ_REG12  0x12  // ICHG ADC
#define BQ_REG14  0x14

// REG00 bits
#define BQ_B_EN_HIZ   0x80
#define BQ_B_EN_ILIM  0x40
#define BQ_M_IINLIM   0x3F

// REG02 bits
#define BQ_B_CONV_START 0x80
#define BQ_B_CONV_RATE  0x40

// REG03 bits
#define BQ_B_WD_RST     0x40
#define BQ_B_OTG_CONFIG 0x20
#define BQ_B_CHG_CONFIG 0x10

// REG07 bits
#define BQ_B_EN_TERM   0x80
#define BQ_B_STAT_DIS  0x40
#define BQ_M_WATCHDOG  0x30

// REG0B status
#define BQ_M_VBUS_STAT  0xE0
#define BQ_M_CHRG_STAT  0x18
#define BQ_B_PG_STAT    0x04
#define BQ_B_VSYS_STAT  0x01

// REG0C faults
#define BQ_B_WATCHDOG_FAULT 0x80
#define BQ_B_BOOST_FAULT    0x40
#define BQ_M_CHRG_FAULT     0x30
#define BQ_B_BAT_FAULT      0x08
#define BQ_M_NTC_FAULT      0x07

typedef struct {
    i2c_master_dev_handle_t dev;
} bq25895_t;

typedef struct {
    uint8_t raw;
    uint8_t vbus_stat;      // [7:5]
    uint8_t chrg_stat;      // [4:3]: 0=none 1=pre 2=fast 3=done
    bool    power_good;
    bool    vsys_stat;
} bq25895_status_t;

typedef struct {
    uint8_t raw;
    bool    watchdog_fault;
    bool    boost_fault;
    uint8_t chrg_fault;     // [5:4]
    bool    bat_fault;
    uint8_t ntc_fault;      // [2:0]
} bq25895_faults_t;

esp_err_t bq25895_init(i2c_master_bus_handle_t bus, uint8_t addr, bq25895_t *out);

// Charge control
esp_err_t bq25895_set_charge_enable(bq25895_t *dev, bool enabled);
esp_err_t bq25895_set_hiz(bq25895_t *dev, bool enabled);
esp_err_t bq25895_watchdog_kick(bq25895_t *dev);

// Programmable limits
esp_err_t bq25895_set_input_current_limit_ma(bq25895_t *dev, int ma);
esp_err_t bq25895_set_fast_charge_current_ma(bq25895_t *dev, int ma);
esp_err_t bq25895_set_precharge_current_ma(bq25895_t *dev, int ma);
esp_err_t bq25895_set_termination_current_ma(bq25895_t *dev, int ma);
esp_err_t bq25895_set_charge_voltage_mv(bq25895_t *dev, int mv);

// ADC
esp_err_t bq25895_adc_start_oneshot(bq25895_t *dev);
esp_err_t bq25895_adc_set_continuous(bq25895_t *dev, bool enabled);
esp_err_t bq25895_read_vsys_v(bq25895_t *dev, float *out);
esp_err_t bq25895_read_vbus_v(bq25895_t *dev, float *out);
esp_err_t bq25895_read_vbus_good(bq25895_t *dev, bool *out);
esp_err_t bq25895_read_charge_current_ma(bq25895_t *dev, int *out);

// Status & faults
esp_err_t bq25895_read_status(bq25895_t *dev, bq25895_status_t *out);
esp_err_t bq25895_read_faults(bq25895_t *dev, bq25895_faults_t *out);
const char *bq25895_chrg_stat_label(uint8_t chrg_stat);
