#pragma once
// =============================================================================
// nvs_settings.h — NVS-backed persistent settings
//
// Layout versioning via version.h (X, Y, Z logic).
// All NVS reads/writes go through this module.
// Callers check g_skip_nvs before calling nvs_settings_save().
// =============================================================================
#include <stdint.h>
#include <stdbool.h>
#include "version.h"
#include "esp_err.h"
#include "app.h"

// -----------------------------------------------------------------------------
// NVS namespace and reserved key names
// -----------------------------------------------------------------------------
#define NVS_NAMESPACE           "bsafe"

// Version keys
#define NVS_KEY_SCHEMA_VER      "schema_ver"    // Z: uint8, current schema
#define NVS_KEY_MIN_COMPAT      "min_compat"    // Y: uint8, min compat at write time

// OEM / identity keys
#define NVS_KEY_OEM_NAME        "oem_name"
#define NVS_KEY_OEM_MODEL       "oem_model"
#define NVS_KEY_OEM_SERIAL      "oem_serial"
#define NVS_KEY_OEM_URL         "oem_url"
#define NVS_KEY_HW_VER          "hw_ver"
#define NVS_KEY_LOCATION        "location"

// Splash keys
#define NVS_KEY_SPLASH_EN       "splash_en"
#define NVS_KEY_SPLASH_MS       "splash_ms"
#define NVS_KEY_SPLASH_LOGO_X   "logo_x"
#define NVS_KEY_SPLASH_LOGO_Y   "logo_y"
#define NVS_KEY_SPLASH_LOGO_W   "logo_w"
#define NVS_KEY_SPLASH_LOGO_H   "logo_h"
#define NVS_KEY_SPLASH_PROG_X   "prog_x"
#define NVS_KEY_SPLASH_PROG_Y   "prog_y"
#define NVS_KEY_SPLASH_PROG_W   "prog_w"
#define NVS_KEY_SPLASH_PROG_H   "prog_h"
#define NVS_KEY_SPLASH_OEM_EN   "oem_name_en"
#define NVS_KEY_SPLASH_OEM_X    "oem_name_x"
#define NVS_KEY_SPLASH_OEM_Y    "oem_name_y"

// Logo blob key
#define NVS_KEY_LOGO            "logo"

// Menu / operational settings keys
#define NVS_KEY_ADDRESS         "address"
#define NVS_KEY_CHG_TO          "chg_to"
#define NVS_KEY_DSC_TO          "dsc_to"
#define NVS_KEY_CAPACITY        "capacity"
#define NVS_KEY_MAX_TEMP        "max_temp"

// -----------------------------------------------------------------------------
// settings_t — the full persistent settings block
// Increment FW_SCHEMA_VERSION in version.h whenever this struct changes.
// Add a migration function in nvs_settings.c for the old schema version.
// -----------------------------------------------------------------------------
typedef struct {
    // --- Schema identity (written/read for verification) ---
    uint8_t  schema_ver;            // Z: schema version this block was written with
    uint8_t  min_compat;            // Y: min compat schema at write time

    // --- OEM / identity ---
    char     oem_name[FW_OEM_NAME_LEN];
    char     oem_model[FW_OEM_MODEL_LEN];
    char     oem_serial[FW_OEM_SERIAL_LEN];
    char     oem_url[FW_OEM_URL_LEN];
    char     hw_ver[FW_HW_VER_LEN];
    char     location[FW_LOCATION_LEN];

    // --- Splash screen ---
    splash_config_t splash;

    // --- Logo (1KB reservation, format TBD) ---
    uint8_t  logo[FW_LOGO_SIZE_BYTES];
    bool     logo_valid;            // false = no logo stored, use text fallback

    // --- Operational / menu settings ---
    uint8_t  address;               // CAN address 0-63
    float    chg_to;                // charge target voltage V
    float    dsc_to;                // discharge cutoff voltage V
    uint16_t capacity_mah;          // battery capacity mAh
    uint8_t  max_temp;              // max battery temperature C

} settings_t;

// -----------------------------------------------------------------------------
// Load result — returned by nvs_settings_load(), consumed by main.c
// -----------------------------------------------------------------------------
typedef enum {
    NVS_LOAD_OK = 0,        // normal boot, all good
    NVS_LOAD_FIRST_BOOT,    // NVS was empty, initialized with defaults
    NVS_LOAD_MIGRATED,      // schema was older, migration ran successfully
    NVS_LOAD_PROMPT,        // incompatible schema — caller must show LEFT/RIGHT
    NVS_LOAD_ERROR,         // NVS hardware/partition error
} nvs_load_result_t;

// Populated by nvs_settings_load() when result is NVS_LOAD_PROMPT
typedef struct {
    uint8_t stored_schema;  // Z found in NVS
    uint8_t fw_schema;      // X from version.h
    uint8_t min_compat;     // Y from version.h
    bool    too_new;        // true = Z > X (NVS from newer firmware)
    bool    too_old;        // true = Z < Y (no migration script)
} nvs_conflict_t;

// -----------------------------------------------------------------------------
// Global skip flag — set when user presses RIGHT on conflict prompt.
// When true: all nvs_settings_save() calls are no-ops.
// Checked by app.c before saving.
// -----------------------------------------------------------------------------
extern bool g_skip_nvs;

// -----------------------------------------------------------------------------
// Default settings — compile-time values applied at first boot or after ERASE
// -----------------------------------------------------------------------------
#define SETTINGS_DEFAULTS { \
    .schema_ver     = FW_SCHEMA_VERSION,        \
    .min_compat     = FW_MIN_COMPAT_SCHEMA,     \
    .oem_name       = FW_OEM_NAME,              \
    .oem_model      = FW_OEM_MODEL,             \
    .oem_serial     = FW_OEM_SERIAL,            \
    .oem_url        = FW_OEM_URL,               \
    .hw_ver         = FW_HW_VERSION,            \
    .location       = FW_MANUFACTURE_LOCATION,  \
    .splash         = SPLASH_DEFAULTS,          \
    .logo           = {0},                      \
    .logo_valid     = false,                    \
    .address        = 0,                        \
    .chg_to         = 4.20f,                    \
    .dsc_to         = 3.70f,                    \
    .capacity_mah   = 10000,                    \
    .max_temp       = 50,                       \
}

// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------

/**
 * @brief  Load settings from NVS into *out.
 *         Runs version comparison and migration if needed.
 *         If result is NVS_LOAD_PROMPT, *conflict is populated.
 *         *out is always valid (defaults on error/first-boot).
 */
nvs_load_result_t nvs_settings_load(settings_t *out, nvs_conflict_t *conflict);

/**
 * @brief  Save full settings block to NVS.
 *         No-op if g_skip_nvs is true.
 */
esp_err_t nvs_settings_save(const settings_t *s);

/**
 * @brief  Save a single float menu item by NVS key.
 *         No-op if g_skip_nvs is true.
 */
esp_err_t nvs_settings_save_float(const char *nvs_key, float val);

/**
 * @brief  Save a single int menu item by NVS key.
 *         No-op if g_skip_nvs is true.
 */
esp_err_t nvs_settings_save_int(const char *nvs_key, int32_t val);

/**
 * @brief  Erase entire NVS namespace and reinitialize with defaults.
 *         Ignores g_skip_nvs (user explicitly requested erase).
 */
esp_err_t nvs_settings_erase_and_init(settings_t *out);

/**
 * @brief  Apply loaded settings into app menu items.
 *         Call after nvs_settings_load() and app_init().
 */

// void nvs_settings_apply(const settings_t *s, struct app_t *app);
void nvs_settings_apply(const settings_t *s, app_t *app);

/**
 * @brief  Map a menu item key string to its NVS key string.
 *         Returns NULL if key has no NVS backing (e.g. "Exec").
 */
const char *nvs_key_for_menu_item(const char *menu_key);
