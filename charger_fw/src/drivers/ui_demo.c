// =============================================================================
// ui_demo.c — TextDemo-style display driver for charger_fw
//
// Screens cycle every STATUS_ROTATE_MS milliseconds:
//
//   Screen 0 — STATUS
//   ┌────────────────┐
//   │ CHARGER   FAST │  ← status bar (inverted)
//   │ Vbat  3.782 V  │
//   │ Ibat  1.234 A  │
//   │ Pbat  4.66 W   │
//   │ Vsys  3.801 V  │
//   │ Vbus  5.103 V  │
//   │ ████████░░░ 72%│  ← charge bar
//   │ bSafe v0.1     │
//   └────────────────┘
//
//   Screen 1 — BATTERY (big numbers)
//   ┌────────────────┐
//   │ BATTERY        │
//   │ 3.78 V         │  ← 2x font
//   │ 1.23 A         │  ← 2x font
//   │ CHARGING       │
//   └────────────────┘
//
//   Screen 2 — SYS
//   ┌────────────────┐
//   │ SYSTEM         │
//   │ Vsys  3.801 V  │
//   │ Vbus  5.103 V  │
//   │ pg: YES fan: ON│
//   │ CE: ON QON: OFF│
//   └────────────────┘
// =============================================================================

#include "ui_demo.h"
#include "display.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "ui";

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Safe snprintf into a fixed 20-char local buffer — avoids stack waste
#define FMT(buf, ...) snprintf(buf, sizeof(buf), __VA_ARGS__)

static void _screen_status(display_t *d, const ui_data_t *data)
{
    char val[20];

    // Row 0: inverted status bar
    FMT(val, "%s", data->chrg_label);
    display_status_bar(d, "CHARGER", val);

    // Row 1: battery voltage
    FMT(val, "%.3fV", data->vbat);
    display_data_row(d, 1, "Vbat", val);

    // Row 2: battery current
    FMT(val, "%.3fA", data->ibat);
    display_data_row(d, 2, "Ibat", val);

    // Row 3: power
    FMT(val, "%.2fW", data->pbat);
    display_data_row(d, 3, "Pbat", val);

    // Row 4: sys voltage
    FMT(val, "%.3fV", data->vsys);
    display_data_row(d, 4, "Vsys", val);

    // Row 5: vbus voltage
    FMT(val, "%.3fV", data->vbus);
    display_data_row(d, 5, "Vbus", val);

    // Row 6: charge bar
    display_bar(d, 6, data->charge_pct);

    // Row 7: firmware label
    display_text(d, 0, 7, "bSafe v0.1", false);
}

static void _screen_battery(display_t *d, const ui_data_t *data)
{
    char val[20];

    // Row 0: section header (inverted)
    display_status_bar(d, "BATTERY", "");

    // Rows 1-2: voltage 2x
    FMT(val, "%.2fV", data->vbat);
    display_draw_str_2x(d, 0, 16, val, false);

    // Rows 3-4: current 2x
    FMT(val, "%.2fA", data->ibat);
    display_draw_str_2x(d, 0, 32, val, false);

    // Row 6: charge state
    display_text(d, 0, 6, data->chrg_label, false);

    // Row 7: power good indicator
    display_text(d, 0, 7, data->power_good ? "PG: YES" : "PG: NO", false);
}

static void _screen_system(display_t *d, const ui_data_t *data)
{
    char val[20];

    display_status_bar(d, "SYSTEM", "");

    FMT(val, "%.3fV", data->vsys);
    display_data_row(d, 1, "Vsys", val);

    FMT(val, "%.3fV", data->vbus);
    display_data_row(d, 2, "Vbus", val);

    FMT(val, "pg:%-3s fan:%-3s",
        data->power_good ? "YES" : "NO",
        data->fan_on     ? "ON"  : "OFF");
    display_text(d, 0, 3, val, false);

    FMT(val, "CE:%-3s QON:%-3s",
        data->ce_enabled ? "ON"  : "OFF",
        data->qon_on     ? "ON"  : "OFF");
    display_text(d, 0, 4, val, false);

    // Row 6: CAN status
    FMT(val, "CAN:%s", data->can_ok ? "OK" : "ERR");
    display_text(d, 0, 6, val, false);

    // Row 7: uptime seconds
    FMT(val, "up:%lus", (unsigned long)data->uptime_s);
    display_text(d, 0, 7, val, false);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ui_init(ui_state_t *ui, display_t *d)
{
    memset(ui, 0, sizeof(*ui));
    ui->d = d;
    ui->screen = 0;
    ui->last_rotate_ms = 0;
}

void ui_update(ui_state_t *ui, const ui_data_t *data, uint32_t now_ms)
{
    // Rotate screen every STATUS_ROTATE_MS
    if ((now_ms - ui->last_rotate_ms) >= CONFIG_STATUS_ROTATE_MS) {
        ui->screen = (ui->screen + 1) % UI_SCREEN_COUNT;
        ui->last_rotate_ms = now_ms;
        ESP_LOGD(TAG, "Screen → %d", ui->screen);
    }

    display_clear(ui->d, 0x00);

    switch (ui->screen) {
        case UI_SCREEN_STATUS:
            _screen_status(ui->d, data);
            break;
        case UI_SCREEN_BATTERY:
            _screen_battery(ui->d, data);
            break;
        case UI_SCREEN_SYSTEM:
            _screen_system(ui->d, data);
            break;
        default:
            display_text(ui->d, 0, 3, "???", false);
            break;
    }

    display_flush(ui->d);
}

void ui_force_screen(ui_state_t *ui, int screen, uint32_t now_ms)
{
    if (screen >= 0 && screen < UI_SCREEN_COUNT) {
        ui->screen = screen;
        ui->last_rotate_ms = now_ms;
    }
}
