#pragma once
// =============================================================================
// version.h — Compile-time firmware identity, schema versioning, OEM branding
//
// RULES:
//   FW_VERSION_*        bump on every release
//   FW_SCHEMA_VERSION   bump when settings_t layout changes (follows FW_VERSION)
//   FW_MIN_COMPAT_SCHEMA oldest schema version this firmware can migrate FROM
//                        update only when a migration script is added for it
//
// NVS comparison logic (X = FW_SCHEMA_VERSION, Y = FW_MIN_COMPAT_SCHEMA,
//                        Z = schema version stored in NVS):
//   X == Z, Y missing  → first boot, initialize NVS with defaults
//   X == Z, Y present  → normal boot, read/write
//   X > Z >= Y         → compatible, run Z→X migration then write
//   Z < Y              → too old, no migration path → prompt LEFT/RIGHT
//   Z > X              → NVS from newer firmware → prompt LEFT/RIGHT
// =============================================================================

// -----------------------------------------------------------------------------
// Firmware version (major.minor.build)
// -----------------------------------------------------------------------------
#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_BUILD    0

// Human-readable string — generated from the above
#define FW_VERSION_STR      "1.0.0"

// -----------------------------------------------------------------------------
// NVS schema versioning
// Schema version tracks settings_t layout. Increment when fields are added,
// removed, or renamed. Does NOT need to match FW_VERSION exactly.
// -----------------------------------------------------------------------------
#define FW_SCHEMA_VERSION       1       // X: current schema this firmware writes
#define FW_MIN_COMPAT_SCHEMA    1       // Y: oldest schema we have a migration for

// -----------------------------------------------------------------------------
// Production metadata (baked at compile time)
// Update before each production flash batch.
// -----------------------------------------------------------------------------
#define FW_BUILD_DATE       __DATE__    // e.g. "Feb 23 2026"
#define FW_BUILD_TIME       __TIME__    // e.g. "14:30:00"

// Manufacture location — free text, max 32 bytes including null
#define FW_MANUFACTURE_LOCATION     "Bucharest, Romania"

// Hardware version string — identifies compatible PCB revision
// Used to warn if NVS was written by firmware targeting a different HW rev
#define FW_HW_VERSION       "PCB_R1"

// -----------------------------------------------------------------------------
// OEM branding (stored in NVS at first boot, can be overwritten by provisioning)
// These are the compile-time defaults only.
// -----------------------------------------------------------------------------
#define FW_OEM_NAME         "bSafe V1"
#define FW_OEM_MODEL        "bSafe V1"
#define FW_OEM_SERIAL       "" // populated at provisioning time
#define FW_OEM_URL          ""

// Field size limits (including null terminator)
#define FW_OEM_NAME_LEN     32
#define FW_OEM_MODEL_LEN    32
#define FW_OEM_SERIAL_LEN   32
#define FW_OEM_URL_LEN      128
#define FW_LOCATION_LEN     32
#define FW_HW_VER_LEN       16

// -----------------------------------------------------------------------------
// Logo reservation
// 1KB raw buffer — format TBD (monochrome bitmap, RLE, or other).
// Currently unused; splash screen renders text only.
// -----------------------------------------------------------------------------
#define FW_LOGO_SIZE_BYTES  1024

// -----------------------------------------------------------------------------
// Splash screen configuration
// Mirrors a Windows 9x-style boot screen with positioned elements.
// -----------------------------------------------------------------------------
typedef struct {
    bool     show_splash;           // display splash on boot
    uint16_t splash_duration_ms;    // how long to hold splash (0 = until input)

    // Logo position and size on display
    int16_t  logo_x;
    int16_t  logo_y;
    uint16_t logo_w;
    uint16_t logo_h;

    // Progress bar geometry
    int16_t  progress_x;
    int16_t  progress_y;
    uint16_t progress_w;
    uint16_t progress_h;

    // OEM name text position
    bool     show_oem_name;
    int16_t  oem_name_x;
    int16_t  oem_name_y;
} splash_config_t;

// Compile-time splash defaults — stored in NVS at first boot
#define SPLASH_DEFAULTS { \
    .show_splash        = true,  \
    .splash_duration_ms = 2500,  \
    .logo_x             = 0,     \
    .logo_y             = 0,     \
    .logo_w             = 128,   \
    .logo_h             = 32,    \
    .progress_x         = 0,     \
    .progress_y         = 56,    \
    .progress_w         = 128,   \
    .progress_h         = 8,     \
    .show_oem_name      = true,  \
    .oem_name_x         = 0,     \
    .oem_name_y         = 36,    \
}

// -----------------------------------------------------------------------------
// Combined firmware identity struct — stored in flash (const, not NVS)
// Used for logging, CAN telemetry, and NVS version comparison.
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t  major;
    uint8_t  minor;
    uint16_t build;
    uint8_t  schema_version;       // X
    uint8_t  min_compat_schema;    // Y
    const char *version_str;
    const char *build_date;
    const char *build_time;
    const char *hw_version;
    const char *manufacture_location;
    const char *oem_name;
    const char *oem_model;
    const char *oem_url;
} fw_identity_t;

// Defined in main.c as a const — extern here for read access everywhere
extern const fw_identity_t g_fw_identity;
