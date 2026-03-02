#pragma once
// =============================================================================
// app.h — Application state, types, and public API
// Port of main.py App class to ESP-IDF C
// =============================================================================
#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "display.h"
#include "ui_demo.h"
#include "esp_adc/adc_oneshot.h"

// ---------------------------------------------------------------------------
// NTC temperature constants (external probe on GPIO1)
// ---------------------------------------------------------------------------

#define SH_A  (1.668575e-03f)
#define SH_B  (1.321981e-04f)
#define SH_C  (3.128401e-07f)

#define TS_R_PULLUP_OHM  10000.0f  // 10k pull-up to REGN
#define EXT_R_PULL_OHM   10000.0f   // 10k pull-up to 3V3

// ---------------------------------------------------------------------------
// Thresholds
// ---------------------------------------------------------------------------
#define VBAT_PRESENT_V      1.5f
#define VIN_DETECT_V        0.5f
#define VIN_MIN_OK_V        7.5f
#define TACH_MIN_PERIOD_US  1000
#define TACH_PPR            2
#define TACH_TIMEOUT_US     2000000  // 2s

// ---------------------------------------------------------------------------
// Discharge / Measure load (LEDC on GPIO0)
// ---------------------------------------------------------------------------
#define DSC_GPIO        0
#define DSC_FREQ_HZ     10000
#define DSC_TIMER       LEDC_TIMER_0
#define DSC_CHANNEL     LEDC_CHANNEL_0
#define DSC_DUTY_BITS   10           // 10-bit = 0..1023
#define DSC_DUTY_MAX    1023

// ---------------------------------------------------------------------------
// SK6812 NeoPixel strip (GPIO8, 10 LEDs)
// ---------------------------------------------------------------------------
#define LED_GPIO        8
#define LED_COUNT       10
#define LED_BLINK_MS    400

// LED colors (GRB order for SK6812)
#define LED_OFF_G  0
#define LED_OFF_R  0
#define LED_OFF_B  0
#define LED_CHG_G  50   // green
#define LED_CHG_R  10
#define LED_CHG_B  0
#define LED_ERR_G  0    // red
#define LED_ERR_R  60
#define LED_ERR_B  0
#define LED_DSC_G  25   // orange
#define LED_DSC_R  50
#define LED_DSC_B  0
#define LED_IDLE_G 20   // purple-ish
#define LED_IDLE_R 40
#define LED_IDLE_B 50

// ---------------------------------------------------------------------------
// Repeat button
// ---------------------------------------------------------------------------
#define REPEAT_DELAY_MS  400
#define REPEAT_MS        200

// ---------------------------------------------------------------------------
// Application mode / page
// ---------------------------------------------------------------------------
typedef enum {
    MODE_MAIN = 0,
    MODE_EDIT,
    MODE_PAGE,
} app_mode_t;

typedef enum {
    PAGE_NONE = 0,
    PAGE_STATUS,
    PAGE_CHARGE,
    PAGE_DISCHARGE,
    PAGE_MEASURE,
    PAGE_CAN,
} app_page_t;

// ---------------------------------------------------------------------------
// Menu item types
// ---------------------------------------------------------------------------
typedef enum {
    ITEM_FLOAT = 0,
    ITEM_INT,
    ITEM_LIST,
} item_type_t;

#define ITEM_MAX_OPTS  8
#define ITEM_KEY_LEN   12

typedef struct {
    char        key[ITEM_KEY_LEN];
    item_type_t type;
    // float/int fields
    float       val;
    float       step;
    float       vmin;
    float       vmax;
    // list fields
    int         idx;
    int         nopts;
    const char *opts[ITEM_MAX_OPTS];
    // display format: 0=int, 1=1dp, 2=2dp
    int         decimals;
} menu_item_t;

#define MENU_ITEMS_MAX 8

// ---------------------------------------------------------------------------
// Repeat button state
// ---------------------------------------------------------------------------
typedef struct {
    bool     was_down;
    uint32_t t_down_ms;
    uint32_t t_last_rep_ms;
} repeat_btn_t;

// ---------------------------------------------------------------------------
// LED pixel (GRB)
// ---------------------------------------------------------------------------
typedef struct { uint8_t g, r, b; } led_color_t;

// ---------------------------------------------------------------------------
// Measure I.R. results
// ---------------------------------------------------------------------------
typedef struct {
    float v_noload;     // V before load
    float v_loaded;     // V under load
    float i_loaded;     // A (negative = discharge)
    float v_drop;       // v_noload - v_loaded
    float r_mohm;       // mΩ result (NULL_MOHM if invalid)
    bool  valid;
} meas_result_t;

// Note: CIRCUIT_R_DISCHG_MOHM is defined in config.h

// ---------------------------------------------------------------------------
// Full application state
// ---------------------------------------------------------------------------
typedef struct {
    board_t   *board;
    display_t *disp;

    // --- Mode / page ---
    app_mode_t  mode;
    app_page_t  page;
    app_page_t  last_page;

    // --- Menu ---
    menu_item_t items[MENU_ITEMS_MAX];
    int         num_items;
    int         sel;
    // edit backup
    float       edit_backup_val;
    int         edit_backup_idx;

    // --- Button edge state ---
    bool menu_prev;
    bool ok_prev;
    // repeat buttons for left/right
    repeat_btn_t rep_left;
    repeat_btn_t rep_right;

    // --- No-input override (OK+MENU held at startup) ---
    bool no_input_override;

    // --- Charge page ---
    uint32_t charge_next_draw_ms;
    bool     charge_configured;
    bool     chg_error_latched;
    bool     chg_full_latched;

    // --- Discharge page ---
    int      dsc_pct;           // 10..100
    uint32_t dsc_next_draw_ms;
    bool     dsc_pwm_running;

    // --- Measure page ---
    meas_result_t meas_30;
    meas_result_t meas_60;
    uint32_t      meas_next_draw_ms;

    // --- Status page ---
    uint32_t status_next_draw_ms;

    // --- Tachometer (GPIO interrupt, period measurement) ---
    volatile uint32_t tach_last_us;
    volatile uint32_t tach_period_us;
    volatile uint32_t tach_last_edge_us;
    volatile bool     tach_valid;
    float             tach_ema_rpm;

    // --- BQ wake ---
    bool bq_wake_attempted;
} app_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
void app_init(app_t *app, board_t *board, display_t *disp);
void app_run(app_t *app);   // never returns
extern adc_oneshot_unit_handle_t s_adc;