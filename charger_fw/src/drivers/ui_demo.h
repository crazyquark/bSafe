#pragma once
// =============================================================================
// ui_demo.h — rotating screen UI for charger_fw
// =============================================================================
#include <stdint.h>
#include <stdbool.h>
#include "display.h"

// Rotate interval — pulled from config.h if available, otherwise default 5s
#ifndef CONFIG_STATUS_ROTATE_MS
#define CONFIG_STATUS_ROTATE_MS 5000
#endif

typedef enum {
    UI_SCREEN_STATUS  = 0,
    UI_SCREEN_BATTERY = 1,
    UI_SCREEN_SYSTEM  = 2,
    UI_SCREEN_COUNT   = 3,
} ui_screen_t;

// All the live data the UI needs — populated by main loop from sensors
typedef struct {
    // INA226 battery side
    float vbat;
    float ibat;
    float pbat;

    // BQ25895
    float vsys;
    float vbus;
    bool  power_good;
    const char *chrg_label;   // "not_charging" / "precharge" / "fast_charge" / "done"

    // Computed
    int   charge_pct;         // 0..100, estimated from vbat vs chemistry limits

    // Board state
    bool  fan_on;
    bool  ce_enabled;
    bool  qon_on;
    bool  can_ok;

    // Timing
    uint32_t uptime_s;
} ui_data_t;

typedef struct {
    display_t  *d;
    int         screen;
    uint32_t    last_rotate_ms;
} ui_state_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void ui_init(ui_state_t *ui, display_t *d);

/**
 * @brief Call from main loop. Rotates screens on timer, redraws, flushes.
 * @param now_ms  esp_timer_get_time() / 1000  or  xTaskGetTickCount() * portTICK_PERIOD_MS
 */
void ui_update(ui_state_t *ui, const ui_data_t *data, uint32_t now_ms);

/** @brief Jump to a specific screen immediately (e.g. on button press). */
void ui_force_screen(ui_state_t *ui, int screen, uint32_t now_ms);
