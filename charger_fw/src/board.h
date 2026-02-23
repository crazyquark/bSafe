#pragma once
// =============================================================================
// board.h — hardware abstraction (port of board.py)
// =============================================================================
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "drivers/tca9535.h"
#include "drivers/ina226.h"
#include "drivers/bq25895.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"

// ---------------------------------------------------------------------------
// Button event struct — mirrors Python poll_button_edges() return dict
// ---------------------------------------------------------------------------
typedef struct {
    bool menu;
    bool left;
    bool right;
    bool ok;
} board_btn_edges_t;

// ---------------------------------------------------------------------------
// Board state
// ---------------------------------------------------------------------------
typedef struct {
    // I2C
    i2c_master_bus_handle_t i2c_bus;

    // Peripheral presence flags
    bool has_exp;
    bool has_ina_batt;
    bool has_ina_sys;
    bool has_bq;
    bool has_twai;
    bool has_lcd;

    // Device handles
    tca9535_t exp;
    ina226_t  ina_batt;
    ina226_t  ina_sys;
    bq25895_t bq;
    esp_lcd_panel_io_handle_t  lcd_io; 
    esp_lcd_panel_handle_t     lcd_panel;

    // Cached expander input state (refresh each poll)
    uint8_t in0;
    uint8_t in1;

    // Button edge detection state
    bool btn_prev_menu;
    bool btn_prev_left;
    bool btn_prev_right;
    bool btn_prev_ok;
} board_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/** @brief Initialise the I2C master bus. Call first. */
esp_err_t board_init_i2c(board_t *b);

/** @brief Scan and initialise the TCA9535 expander (optional). */
esp_err_t board_init_expander(board_t *b);

/** @brief Scan and initialise both INA226 monitors (optional). */
esp_err_t board_init_ina226(board_t *b, float max_current_a);

/** @brief Scan and initialise BQ25895 charger (optional). */
esp_err_t board_init_bq25895(board_t *b);

/**
 * @brief Initialise TWAI (CAN) controller.
 *        Requires an external transceiver (e.g. SN65HVD230) on TWAI_TX/RX_GPIO.
 */
esp_err_t board_init_twai(board_t *b);

/** @brief Convenience: call all init functions in order. */
esp_err_t board_init_all(board_t *b, float max_current_a);

// ---------------------------------------------------------------------------
// Control helpers
// ---------------------------------------------------------------------------

/** @brief Drive /CE pin (active-low charge enable via expander). No-op without expander. */
esp_err_t board_set_ce_enabled(board_t *b, bool enabled);

/** @brief Drive QON pin via expander. */
esp_err_t board_set_qon(board_t *b, bool enabled);

/** @brief Drive FAN pin via expander. */
esp_err_t board_set_fan(board_t *b, bool enabled);

// ---------------------------------------------------------------------------
// TWAI helpers
// ---------------------------------------------------------------------------

/**
 * @brief Transmit a CAN frame (standard 11-bit ID, up to 8 bytes data).
 * @param id      11-bit CAN identifier
 * @param data    payload bytes
 * @param len     data length (0..8)
 * @param timeout_ms  tx queue block timeout
 */
esp_err_t board_twai_transmit(board_t *b, uint32_t id,
                               const uint8_t *data, uint8_t len,
                               uint32_t timeout_ms);

/**
 * @brief Receive a CAN frame (blocking up to timeout_ms).
 */
esp_err_t board_twai_receive(board_t *b, twai_message_t *msg, uint32_t timeout_ms);

/** @brief Read TWAI driver status / alert flags. */
esp_err_t board_twai_get_status(board_t *b, twai_status_info_t *info);

/** @brief Init OLED display. */
esp_err_t board_init_ssd1306(board_t *b);