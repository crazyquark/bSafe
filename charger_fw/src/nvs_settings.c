// =============================================================================
// nvs_settings.c — NVS persistent settings implementation
// =============================================================================
#include "nvs_settings.h"
#include "app.h"
#include "version.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdbool.h>

static const char *TAG = "nvs_settings";

// -----------------------------------------------------------------------------
// Global skip flag
// -----------------------------------------------------------------------------
bool g_skip_nvs = false;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

static esp_err_t _open(nvs_handle_t *h, nvs_open_mode_t mode)
{
    esp_err_t ret = nvs_open(NVS_NAMESPACE, mode, h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void _write_str(nvs_handle_t h, const char *key, const char *val, size_t max_len)
{
    char buf[256]; // safe scratch — largest field is oem_url at 128
    size_t copy = strlen(val);
    if (copy >= max_len) copy = max_len - 1;
    memcpy(buf, val, copy);
    buf[copy] = '\0';
    esp_err_t ret = nvs_set_str(h, key, buf);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "set_str '%s' failed: %s", key, esp_err_to_name(ret));
}

static void _read_str(nvs_handle_t h, const char *key,
                      char *out, size_t out_len, const char *def)
{
    size_t req = out_len;
    esp_err_t ret = nvs_get_str(h, key, out, &req);
    if (ret != ESP_OK) {
        strncpy(out, def, out_len - 1);
        out[out_len - 1] = '\0';
    }
}

static void _write_u8(nvs_handle_t h, const char *key, uint8_t val)
{
    esp_err_t ret = nvs_set_u8(h, key, val);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "set_u8 '%s' failed: %s", key, esp_err_to_name(ret));
}

static uint8_t _read_u8(nvs_handle_t h, const char *key, uint8_t def)
{
    uint8_t v = def;
    nvs_get_u8(h, key, &v);
    return v;
}

static void _write_u16(nvs_handle_t h, const char *key, uint16_t val)
{
    esp_err_t ret = nvs_set_u16(h, key, val);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "set_u16 '%s' failed: %s", key, esp_err_to_name(ret));
}

static uint16_t _read_u16(nvs_handle_t h, const char *key, uint16_t def)
{
    uint16_t v = def;
    nvs_get_u16(h, key, &v);
    return v;
}

static void _write_i16(nvs_handle_t h, const char *key, int16_t val)
{
    esp_err_t ret = nvs_set_i16(h, key, val);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "set_i16 '%s' failed: %s", key, esp_err_to_name(ret));
}

static int16_t _read_i16(nvs_handle_t h, const char *key, int16_t def)
{
    int16_t v = def;
    nvs_get_i16(h, key, &v);
    return v;
}

// Floats stored as fixed-point i32 (value * 10000)
static void _write_float(nvs_handle_t h, const char *key, float val)
{
    int32_t fixed = (int32_t)(val * 10000.0f);
    esp_err_t ret = nvs_set_i32(h, key, fixed);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "set_i32(float) '%s' failed: %s", key, esp_err_to_name(ret));
}

static float _read_float(nvs_handle_t h, const char *key, float def)
{
    int32_t fixed = (int32_t)(def * 10000.0f);
    nvs_get_i32(h, key, &fixed);
    return (float)fixed / 10000.0f;
}

static void _write_bool(nvs_handle_t h, const char *key, bool val)
{
    nvs_set_u8(h, key, val ? 1 : 0);
}

static bool _read_bool(nvs_handle_t h, const char *key, bool def)
{
    uint8_t v = def ? 1 : 0;
    nvs_get_u8(h, key, &v);
    return v != 0;
}

// -----------------------------------------------------------------------------
// Write all settings fields to open NVS handle (no commit)
// -----------------------------------------------------------------------------
static void _write_all(nvs_handle_t h, const settings_t *s)
{
    // Schema version markers
    _write_u8(h,  NVS_KEY_SCHEMA_VER,   s->schema_ver);
    _write_u8(h,  NVS_KEY_MIN_COMPAT,   s->min_compat);

    // OEM / identity
    _write_str(h, NVS_KEY_OEM_NAME,     s->oem_name,   FW_OEM_NAME_LEN);
    _write_str(h, NVS_KEY_OEM_MODEL,    s->oem_model,  FW_OEM_MODEL_LEN);
    _write_str(h, NVS_KEY_OEM_SERIAL,   s->oem_serial, FW_OEM_SERIAL_LEN);
    _write_str(h, NVS_KEY_OEM_URL,      s->oem_url,    FW_OEM_URL_LEN);
    _write_str(h, NVS_KEY_HW_VER,       s->hw_ver,     FW_HW_VER_LEN);
    _write_str(h, NVS_KEY_LOCATION,     s->location,   FW_LOCATION_LEN);

    // Splash
    _write_bool(h, NVS_KEY_SPLASH_EN,   s->splash.show_splash);
    _write_u16(h,  NVS_KEY_SPLASH_MS,   s->splash.splash_duration_ms);
    _write_i16(h,  NVS_KEY_SPLASH_LOGO_X, s->splash.logo_x);
    _write_i16(h,  NVS_KEY_SPLASH_LOGO_Y, s->splash.logo_y);
    _write_u16(h,  NVS_KEY_SPLASH_LOGO_W, s->splash.logo_w);
    _write_u16(h,  NVS_KEY_SPLASH_LOGO_H, s->splash.logo_h);
    _write_i16(h,  NVS_KEY_SPLASH_PROG_X, s->splash.progress_x);
    _write_i16(h,  NVS_KEY_SPLASH_PROG_Y, s->splash.progress_y);
    _write_u16(h,  NVS_KEY_SPLASH_PROG_W, s->splash.progress_w);
    _write_u16(h,  NVS_KEY_SPLASH_PROG_H, s->splash.progress_h);
    _write_bool(h, NVS_KEY_SPLASH_OEM_EN, s->splash.show_oem_name);
    _write_i16(h,  NVS_KEY_SPLASH_OEM_X,  s->splash.oem_name_x);
    _write_i16(h,  NVS_KEY_SPLASH_OEM_Y,  s->splash.oem_name_y);

    // Logo blob — only write if valid to avoid wearing NVS with 1KB writes
    if (s->logo_valid) {
        nvs_set_blob(h, NVS_KEY_LOGO, s->logo, FW_LOGO_SIZE_BYTES);
    }
    _write_bool(h, "logo_valid", s->logo_valid);

    // Operational settings
    _write_u8(h,    NVS_KEY_ADDRESS,    s->address);
    _write_float(h, NVS_KEY_CHG_TO,     s->chg_to);
    _write_float(h, NVS_KEY_DSC_TO,     s->dsc_to);
    _write_u16(h,   NVS_KEY_CAPACITY,   s->capacity_mah);
    _write_u8(h,    NVS_KEY_MAX_TEMP,   s->max_temp);
}

// -----------------------------------------------------------------------------
// Read all settings fields from open NVS handle into *out
// Missing keys fall back to defaults.
// -----------------------------------------------------------------------------
static void _read_all(nvs_handle_t h, settings_t *out)
{
    settings_t def = SETTINGS_DEFAULTS;

    out->schema_ver = _read_u8(h, NVS_KEY_SCHEMA_VER, def.schema_ver);
    out->min_compat = _read_u8(h, NVS_KEY_MIN_COMPAT,  def.min_compat);

    _read_str(h, NVS_KEY_OEM_NAME,  out->oem_name,   FW_OEM_NAME_LEN,   def.oem_name);
    _read_str(h, NVS_KEY_OEM_MODEL, out->oem_model,  FW_OEM_MODEL_LEN,  def.oem_model);
    _read_str(h, NVS_KEY_OEM_SERIAL,out->oem_serial, FW_OEM_SERIAL_LEN, def.oem_serial);
    _read_str(h, NVS_KEY_OEM_URL,   out->oem_url,    FW_OEM_URL_LEN,    def.oem_url);
    _read_str(h, NVS_KEY_HW_VER,    out->hw_ver,     FW_HW_VER_LEN,     def.hw_ver);
    _read_str(h, NVS_KEY_LOCATION,  out->location,   FW_LOCATION_LEN,   def.location);

    out->splash.show_splash         = _read_bool(h, NVS_KEY_SPLASH_EN,     def.splash.show_splash);
    out->splash.splash_duration_ms  = _read_u16(h,  NVS_KEY_SPLASH_MS,     def.splash.splash_duration_ms);
    out->splash.logo_x              = _read_i16(h,  NVS_KEY_SPLASH_LOGO_X, def.splash.logo_x);
    out->splash.logo_y              = _read_i16(h,  NVS_KEY_SPLASH_LOGO_Y, def.splash.logo_y);
    out->splash.logo_w              = _read_u16(h,  NVS_KEY_SPLASH_LOGO_W, def.splash.logo_w);
    out->splash.logo_h              = _read_u16(h,  NVS_KEY_SPLASH_LOGO_H, def.splash.logo_h);
    out->splash.progress_x          = _read_i16(h,  NVS_KEY_SPLASH_PROG_X, def.splash.progress_x);
    out->splash.progress_y          = _read_i16(h,  NVS_KEY_SPLASH_PROG_Y, def.splash.progress_y);
    out->splash.progress_w          = _read_u16(h,  NVS_KEY_SPLASH_PROG_W, def.splash.progress_w);
    out->splash.progress_h          = _read_u16(h,  NVS_KEY_SPLASH_PROG_H, def.splash.progress_h);
    out->splash.show_oem_name       = _read_bool(h, NVS_KEY_SPLASH_OEM_EN, def.splash.show_oem_name);
    out->splash.oem_name_x          = _read_i16(h,  NVS_KEY_SPLASH_OEM_X,  def.splash.oem_name_x);
    out->splash.oem_name_y          = _read_i16(h,  NVS_KEY_SPLASH_OEM_Y,  def.splash.oem_name_y);

    out->logo_valid = _read_bool(h, "logo_valid", false);
    if (out->logo_valid) {
        size_t blob_len = FW_LOGO_SIZE_BYTES;
        esp_err_t ret = nvs_get_blob(h, NVS_KEY_LOGO, out->logo, &blob_len);
        if (ret != ESP_OK) out->logo_valid = false;
    }

    out->address      = _read_u8(h,    NVS_KEY_ADDRESS,  def.address);
    out->chg_to       = _read_float(h, NVS_KEY_CHG_TO,   def.chg_to);
    out->dsc_to       = _read_float(h, NVS_KEY_DSC_TO,   def.dsc_to);
    out->capacity_mah = _read_u16(h,   NVS_KEY_CAPACITY, def.capacity_mah);
    out->max_temp     = _read_u8(h,    NVS_KEY_MAX_TEMP, def.max_temp);
}

// -----------------------------------------------------------------------------
// Migration functions
// Add one function per schema version that needs migration.
// Each function receives the settings loaded from the old schema and updates
// them to the current schema, filling in any new fields with defaults.
//
// Convention: _migrate_from_N migrates schema N → current (FW_SCHEMA_VERSION).
// -----------------------------------------------------------------------------

// Placeholder — no migrations needed at schema v1 (first version).
// When schema v2 is introduced, add _migrate_from_1(settings_t *s) here.

static esp_err_t _run_migration(settings_t *s, uint8_t from_schema)
{
    ESP_LOGI(TAG, "Migrating from schema %u to %u", from_schema, FW_SCHEMA_VERSION);

    // Example structure for future migrations:
    // if (from_schema < 2) { _migrate_from_1(s); }
    // if (from_schema < 3) { _migrate_from_2(s); }

    // No migrations defined yet — just update version markers
    // (safe because FW_MIN_COMPAT_SCHEMA == FW_SCHEMA_VERSION == 1)
    s->schema_ver = FW_SCHEMA_VERSION;
    s->min_compat = FW_MIN_COMPAT_SCHEMA;

    if (from_schema < FW_MIN_COMPAT_SCHEMA || from_schema > FW_SCHEMA_VERSION) {
        // Should never reach here — caller should have caught this
        ESP_LOGE(TAG, "Migration called with unmigrateable schema %u", from_schema);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Migration complete");
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// nvs_settings_load
// -----------------------------------------------------------------------------
nvs_load_result_t nvs_settings_load(settings_t *out, nvs_conflict_t *conflict)
{
    // Initialise NVS flash partition
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issue (%s), erasing", esp_err_to_name(ret));
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        return NVS_LOAD_ERROR;
    }

    nvs_handle_t h;
    ret = _open(&h, NVS_READONLY);

    // --- First boot: namespace doesn't exist yet ---
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "First boot — initialising NVS with defaults");
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        nvs_settings_save(out);  // writes and commits
        return NVS_LOAD_FIRST_BOOT;
    }
    if (ret != ESP_OK) {
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        return NVS_LOAD_ERROR;
    }

    // Read schema version markers from NVS (Z and Y_stored)
    uint8_t z = _read_u8(h, NVS_KEY_SCHEMA_VER, 0xFF);  // schema written to NVS
    uint8_t y_stored = _read_u8(h, NVS_KEY_MIN_COMPAT, 0xFF);
    nvs_close(h);

    uint8_t x = FW_SCHEMA_VERSION;      // current firmware schema
    uint8_t y = FW_MIN_COMPAT_SCHEMA;   // oldest schema we can migrate from

    ESP_LOGI(TAG, "Schema: NVS Z=%u (min_compat stored=%u), FW X=%u Y=%u",
             z, y_stored, x, y);

    // --- Case: Z not found — treat as first boot ---
    if (z == 0xFF) {
        ESP_LOGI(TAG, "Schema version missing — first boot");
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        nvs_settings_save(out);
        return NVS_LOAD_FIRST_BOOT;
    }

    // --- Case: Z > X — NVS from newer firmware, no migration possible ---
    if (z > x) {
        ESP_LOGW(TAG, "NVS schema Z=%u > firmware X=%u (newer firmware wrote this)", z, x);
        if (conflict) {
            conflict->stored_schema = z;
            conflict->fw_schema     = x;
            conflict->min_compat    = y;
            conflict->too_new       = true;
            conflict->too_old       = false;
        }
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        return NVS_LOAD_PROMPT;
    }

    // --- Case: Z < Y — too old, no migration script ---
    if (z < y) {
        ESP_LOGW(TAG, "NVS schema Z=%u < min_compat Y=%u (no migration path)", z, y);
        if (conflict) {
            conflict->stored_schema = z;
            conflict->fw_schema     = x;
            conflict->min_compat    = y;
            conflict->too_new       = false;
            conflict->too_old       = true;
        }
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        return NVS_LOAD_PROMPT;
    }

    // --- Read full settings ---
    ret = _open(&h, NVS_READONLY);
    if (ret != ESP_OK) {
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        return NVS_LOAD_ERROR;
    }
    _read_all(h, out);
    nvs_close(h);

    // --- Case: X == Z — exact match, normal boot ---
    if (z == x) {
        // Check if Y key was present (distinguishes first-boot from normal)
        if (y_stored == 0xFF) {
            // Y was missing — re-write to complete the schema markers
            ESP_LOGI(TAG, "Schema markers incomplete — rewriting");
            out->schema_ver = x;
            out->min_compat = y;
            nvs_settings_save(out);
            return NVS_LOAD_FIRST_BOOT;
        }
        ESP_LOGI(TAG, "Schema match — normal boot");
        return NVS_LOAD_OK;
    }

    // --- Case: X > Z >= Y — migration needed ---
    ESP_LOGI(TAG, "Schema outdated Z=%u, migrating to X=%u", z, x);
    ret = _run_migration(out, z);
    if (ret != ESP_OK) {
        settings_t def = SETTINGS_DEFAULTS;
        *out = def;
        return NVS_LOAD_ERROR;
    }
    nvs_settings_save(out);  // commit migrated settings
    return NVS_LOAD_MIGRATED;
}

// -----------------------------------------------------------------------------
// nvs_settings_save — full block write
// -----------------------------------------------------------------------------
esp_err_t nvs_settings_save(const settings_t *s)
{
    if (g_skip_nvs) {
        ESP_LOGD(TAG, "skip_nvs set — save skipped");
        return ESP_OK;
    }

    nvs_handle_t h;
    esp_err_t ret = _open(&h, NVS_READWRITE);
    if (ret != ESP_OK) return ret;

    _write_all(h, s);

    ret = nvs_commit(h);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(ret));
    nvs_close(h);
    return ret;
}

// -----------------------------------------------------------------------------
// nvs_settings_save_float — single float key update (for menu edits)
// -----------------------------------------------------------------------------
esp_err_t nvs_settings_save_float(const char *nvs_key, float val)
{
    if (g_skip_nvs) return ESP_OK;

    nvs_handle_t h;
    esp_err_t ret = _open(&h, NVS_READWRITE);
    if (ret != ESP_OK) return ret;

    _write_float(h, nvs_key, val);
    ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

// -----------------------------------------------------------------------------
// nvs_settings_save_int — single int32 key update (for menu edits)
// -----------------------------------------------------------------------------
esp_err_t nvs_settings_save_int(const char *nvs_key, int32_t val)
{
    if (g_skip_nvs) return ESP_OK;

    nvs_handle_t h;
    esp_err_t ret = _open(&h, NVS_READWRITE);
    if (ret != ESP_OK) return ret;

    nvs_set_i32(h, nvs_key, val);
    ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

// -----------------------------------------------------------------------------
// nvs_settings_erase_and_init
// -----------------------------------------------------------------------------
esp_err_t nvs_settings_erase_and_init(settings_t *out)
{
    ESP_LOGI(TAG, "Erasing NVS namespace '%s'", NVS_NAMESPACE);

    nvs_handle_t h;
    esp_err_t ret = _open(&h, NVS_READWRITE);
    if (ret == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }

    settings_t def = SETTINGS_DEFAULTS;
    *out = def;

    // Re-open and write defaults
    ret = _open(&h, NVS_READWRITE);
    if (ret != ESP_OK) return ret;
    _write_all(h, out);
    ret = nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "NVS erased and reinitialized with defaults");
    return ret;
}

// -----------------------------------------------------------------------------
// nvs_settings_apply — push loaded settings into app menu items
// -----------------------------------------------------------------------------
void nvs_settings_apply(const settings_t *s, app_t *app)
{
    for (int i = 0; i < app->num_items; i++) {
        menu_item_t *it = &app->items[i];
        if (strcmp(it->key, "Address")  == 0) { it->val = (float)s->address;       continue; }
        if (strcmp(it->key, "ChgTo")    == 0) { it->val = s->chg_to;               continue; }
        if (strcmp(it->key, "DscTo")    == 0) { it->val = s->dsc_to;               continue; }
        if (strcmp(it->key, "Capacity") == 0) { it->val = (float)s->capacity_mah;  continue; }
        if (strcmp(it->key, "MaxTemp")  == 0) { it->val = (float)s->max_temp;      continue; }
    }
}

// -----------------------------------------------------------------------------
// nvs_key_for_menu_item — map menu key → NVS key
// Returns NULL for non-persistent items (e.g. "Exec")
// -----------------------------------------------------------------------------
const char *nvs_key_for_menu_item(const char *menu_key)
{
    if (strcmp(menu_key, "Address")  == 0) return NVS_KEY_ADDRESS;
    if (strcmp(menu_key, "ChgTo")    == 0) return NVS_KEY_CHG_TO;
    if (strcmp(menu_key, "DscTo")    == 0) return NVS_KEY_DSC_TO;
    if (strcmp(menu_key, "Capacity") == 0) return NVS_KEY_CAPACITY;
    if (strcmp(menu_key, "MaxTemp")  == 0) return NVS_KEY_MAX_TEMP;
    return NULL;  // "Exec" and any future non-persistent items
}
