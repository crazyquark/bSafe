// =============================================================================
// main.c — Entry point for charger_fw (bSafe V1)
// =============================================================================
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "display.h"
#include "app.h"
#include "version.h"
#include "nvs_settings.h"
#include "config.h"

static const char *TAG = "main";

// -----------------------------------------------------------------------------
// Firmware identity — compile-time constants, lives in flash (not NVS)
// This is the authoritative source for all version comparisons.
// -----------------------------------------------------------------------------
const fw_identity_t g_fw_identity = {
    .major                = FW_VERSION_MAJOR,
    .minor                = FW_VERSION_MINOR,
    .build                = FW_VERSION_BUILD,
    .schema_version       = FW_SCHEMA_VERSION,
    .min_compat_schema    = FW_MIN_COMPAT_SCHEMA,
    .version_str          = FW_VERSION_STR,
    .build_date           = FW_BUILD_DATE,
    .build_time           = FW_BUILD_TIME,
    .hw_version           = FW_HW_VERSION,
    .manufacture_location = FW_MANUFACTURE_LOCATION,
    .oem_name             = FW_OEM_NAME,
    .oem_model            = FW_OEM_MODEL,
    .oem_url              = FW_OEM_URL,
};

// -----------------------------------------------------------------------------
// Static instances
// -----------------------------------------------------------------------------
static board_t    board;
static display_t  disp;
static app_t      app;
static settings_t settings;

// -----------------------------------------------------------------------------
// Conflict resolution prompt
// Blocks until user presses LEFT (erase) or RIGHT (skip NVS).
// Renders directly to display without going through app_run().
// -----------------------------------------------------------------------------
static void _show_conflict_prompt(display_t *d, board_t *b,
                                  const nvs_conflict_t *c, settings_t *s)
{
    char line[24];

    for (;;) {
        display_clear(d, 0);

        // Title row — inverted
        display_draw_str(d, 0, 0, " NVS CONFLICT ", true);

        // Conflict detail
        if (c->too_new) {
            snprintf(line, sizeof(line), "NVS v%u > FW v%u", c->stored_schema, c->fw_schema);
            display_draw_str(d, 0, 12, line, false);
            display_draw_str(d, 0, 22, "Newer FW wrote NVS", false);
        } else {
            snprintf(line, sizeof(line), "NVS v%u < MIN v%u", c->stored_schema, c->min_compat);
            display_draw_str(d, 0, 12, line, false);
            display_draw_str(d, 0, 22, "No migration path", false);
        }

        display_draw_str(d, 0, 36, "LEFT  = ERASE+RESET", false);
        display_draw_str(d, 0, 46, "RIGHT = SKIP (no save)", false);
        display_draw_str(d, 0, 56, "Waiting...", false);

        display_flush(d);

        // Poll buttons via expander
        if (b->has_exp) {
            tca9535_read_all(&b->exp, &b->in0, &b->in1);
            bool left_down  = !(b->in0 & (1 << (P_BTN2 & 0x07)));
            bool right_down = !(b->in0 & (1 << (P_BTN3 & 0x07)));

            if (left_down) {
                display_clear(d, 0);
                display_draw_str(d, 0, 24, "  ERASING NVS...", false);
                display_flush(d);
                nvs_settings_erase_and_init(s);
                g_skip_nvs = false;
                vTaskDelay(pdMS_TO_TICKS(800));
                return;
            }
            if (right_down) {
                g_skip_nvs = true;
                display_clear(d, 0);
                display_draw_str(d, 0, 20, "  SKIP NVS mode", false);
                display_draw_str(d, 0, 32, "Settings not saved", false);
                display_flush(d);
                vTaskDelay(pdMS_TO_TICKS(800));
                return;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// -----------------------------------------------------------------------------
// app_main
// -----------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "=== %s %s v%s (schema %u, min_compat %u) built %s %s ===",
             g_fw_identity.oem_name,
             g_fw_identity.oem_model,
             g_fw_identity.version_str,
             g_fw_identity.schema_version,
             g_fw_identity.min_compat_schema,
             g_fw_identity.build_date,
             g_fw_identity.build_time);
    ESP_LOGI(TAG, "HW: %s  Location: %s",
             g_fw_identity.hw_version,
             g_fw_identity.manufacture_location);

    // --- Board init ---
    board_init_all(&board, 5.1f);

    // --- Display init early — needed for conflict prompt ---
    display_init(&disp, board.lcd_panel);

    // --- NVS load ---
    nvs_conflict_t conflict = {0};
    nvs_load_result_t load_result = nvs_settings_load(&settings, &conflict);

    switch (load_result) {
    case NVS_LOAD_OK:
        ESP_LOGI(TAG, "Settings loaded OK (schema v%u)", settings.schema_ver);
        break;
    case NVS_LOAD_FIRST_BOOT:
        ESP_LOGI(TAG, "First boot — defaults written to NVS");
        break;
    case NVS_LOAD_MIGRATED:
        ESP_LOGI(TAG, "Settings migrated to schema v%u", FW_SCHEMA_VERSION);
        break;
    case NVS_LOAD_PROMPT:
        ESP_LOGW(TAG, "NVS conflict — showing user prompt");
        _show_conflict_prompt(&disp, &board, &conflict, &settings);
        break;
    case NVS_LOAD_ERROR:
        ESP_LOGE(TAG, "NVS error — running on defaults, writes disabled");
        g_skip_nvs = true;
        break;
    }

    // --- App init (builds menu with compiled-in defaults) ---
    app_init(&app, &board, &disp);

    // --- Overlay NVS settings onto menu defaults ---
    if (!g_skip_nvs) {
        nvs_settings_apply(&settings, &app);
    }

    // --- Run (never returns) ---
    app_run(&app);
}
