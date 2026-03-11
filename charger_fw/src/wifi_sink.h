/**
 * @file wifi_sink.h
 * @brief WiFi + TCP transport for bSafe ESP32-C3 (ESP-IDF 5.2+)
 *
 * Two-phase API:
 *   wifi_sink_preinit()  — call at boot (no radio, safe on 3V3 only)
 *   wifi_sink_connect()  — call when entering WiFi page (starts radio + tasks)
 *
 * Device mDNS advertisement (started inside wifi_sink_connect):
 *   hostname : bsafeXXXXX.local  (XXXXX = last 3 MAC octets, hex)
 *   service  : _bsafe._tcp.local  port=tcp_port  TXT addr=<address>
 *
 * Pi discovery:
 *   Resolves pi_hostname.local via mdns_query_a().
 *   Falls back to WIFI_SINK_PI_FALLBACK_IP (softAP gateway) if mDNS fails.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ── Tunables ────────────────────────────────────────────────────────────── */

#define WIFI_SINK_MAX_NETWORKS         8
#define WIFI_SINK_SSID_LEN             33   /* max 32 + NUL */
#define WIFI_SINK_PASS_LEN             65   /* max 64 + NUL */
#define WIFI_SINK_HOSTNAME_LEN         48   /* pi_hostname field */

#define WIFI_SINK_CONNECT_TIMEOUT_MS   8000  /* WiFi association timeout      */
#define WIFI_SINK_RECONNECT_DELAY_MS   2000  /* pause before retry            */
#define WIFI_SINK_TCP_TIMEOUT_MS       5000  /* connect() + send() timeout ms */

/* TCP keepalive — detects silently dead connections (NAT timeout, cable pull)
 * without the false-positives that a short SO_RCVTIMEO causes on idle links. */
#define WIFI_SINK_KEEPALIVE_IDLE_S     10   /* seconds idle before first probe */
#define WIFI_SINK_KEEPALIVE_INTVL_S     5   /* seconds between probes          */
#define WIFI_SINK_KEEPALIVE_COUNT       3   /* probes before declaring dead     */
#define WIFI_SINK_MDNS_RETRIES         3    /* attempts per tcp_connect() call */

/* Fallback IP when mDNS resolution fails — softAP gateway */
#define WIFI_SINK_PI_FALLBACK_IP       "192.168.1.130"

/* Default network seeded on first boot */
/* Default networks are defined in config.h as WIFI_DEFAULT_NETWORKS.
 * wifi_sink_preinit() seeds them on first boot (empty NVS).             */

/* NVS namespace (separate from main settings namespace) */
#define WIFI_SINK_NVS_NS               "wifi_sink"
#define WIFI_SINK_NVS_COUNT            "net_count"
#define WIFI_SINK_NVS_LAST             "net_last"

/* ── Frame type constants ────────────────────────────────────────────────── */

#define FT_OP_STATUS    0x00   /* device → host: OperationalStatus (2 Hz) */
#define FT_TELEM        0x01   /* device → host: TelemetryStatus (on req) */
#define FT_IR           0x02   /* device → host: IrStatus (on req) */
#define FT_IDENTITY     0x03   /* device → host: IdentityStatus (on req) */

#define FT_WIFI_CFG_PUSH  0x10 /* host → device: push network entry */
#define FT_WIFI_CFG_ACK   0x11 /* device → host: ack */
#define FT_WIFI_NET_RPT   0x12 /* device → host: one known network */
#define FT_WIFI_NET_REQ   0x13 /* host → device: request full network list */

#define FT_CMD          0x80   /* host → device: mode/request command */

/* ── Types ───────────────────────────────────────────────────────────────── */

typedef enum {
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_WIFI_CONNECTING,
    WIFI_STATE_TCP_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_RECONNECTING,
} wifi_sink_state_t;

/** CMD frame payload — mirrors CAN CMD wire format (big-endian multi-byte) */
typedef struct __attribute__((packed)) {
    uint8_t  mode;
    uint8_t  dsc_duty_pct;
    uint16_t dsc_pulse_ms;       /* big-endian */
    uint32_t settle_minutes;     /* big-endian */
    uint8_t  flags;              /* bits[2:0]=req_frame_type, bit[3]=request_flag */
    bool     req_immediate_vbat; /* derived from flags by dispatch */
    uint8_t  req_frame_type;     /* derived from flags by dispatch */
    uint8_t  precharge_thresh;
} wifi_cmd_t;

typedef struct {
    char    ssid[WIFI_SINK_SSID_LEN];
    char    password[WIFI_SINK_PASS_LEN];
    uint8_t priority;   /* higher = preferred; 100 reserved for AP network */
} wifi_network_t;

/**
 * wifi_sink_config_t — passed to wifi_sink_connect().
 *
 * pi_hostname: bare hostname WITHOUT .local suffix (e.g. "bsafe-pi").
 *              wifi_sink resolves it via mDNS; falls back to softAP gateway.
 */
typedef struct {
    uint8_t  address;                             /* bSafe bus address 0-63 */
    uint16_t tcp_port;                            /* Pi server port (default 7000) */
    char     pi_hostname[WIFI_SINK_HOSTNAME_LEN]; /* e.g. "bsafe-pi" */

    /** Called from RX task when a CMD frame arrives. Must be ISR-safe. */
    void (*on_cmd)(const wifi_cmd_t *cmd, void *ctx);

    /** Called when WiFi/TCP state changes. May be NULL. */
    void (*on_state)(wifi_sink_state_t state, const char *ssid, void *ctx);

    void *ctx;
} wifi_sink_config_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * wifi_sink_preinit — Phase 1: create mutexes, load NVS networks.
 * Call once at boot from main.c, before app_init().
 * Does NOT touch the WiFi radio.
 */
esp_err_t wifi_sink_preinit(void);

/**
 * wifi_sink_connect — Phase 2: init radio, start mDNS, start connect_task.
 * Call when the user enters PAGE_WIFI.  Idempotent after first call.
 * wifi_sink_preinit() must have been called first.
 */
esp_err_t wifi_sink_connect(const wifi_sink_config_t *cfg);

/** Legacy combined init — preinit + connect in one call. */
esp_err_t wifi_sink_init(const wifi_sink_config_t *cfg);

/**
 * wifi_sink_send — Send a framed payload over the active TCP connection.
 * Thread-safe (protected by send mutex).
 * Returns ESP_ERR_INVALID_STATE if not connected, ESP_ERR_TIMEOUT if mutex busy.
 */
esp_err_t wifi_sink_send(uint8_t frame_type, const uint8_t *payload, uint16_t len);

wifi_sink_state_t wifi_sink_get_state(void);
bool              wifi_sink_connected(void);
const char       *wifi_sink_current_ssid(void);
const char       *wifi_sink_own_ip(void);          /* own STA IP, "" until DHCP */
const char       *wifi_sink_pi_ip(void);           /* Pi's resolved IP, "" until mDNS */
void              wifi_sink_try_progress(uint8_t *idx, uint8_t *total); /* 1-based */

/** Add or update a network in RAM + NVS. Thread-safe. */
esp_err_t wifi_sink_add_network(const char *ssid, const char *password,
                                uint8_t priority);
uint8_t   wifi_sink_network_count(void);
esp_err_t wifi_sink_get_network(uint8_t idx, wifi_network_t *out);
