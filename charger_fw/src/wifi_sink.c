/**
 * @file wifi_sink.c
 * @brief WiFi + TCP transport for bSafe ESP32-C3 (ESP-IDF 5.2+)
 *
 * Architecture:
 *   connect_task  — manages WiFi association + TCP connect, cycles networks
 *   rx_task       — reads TCP stream, drains frames, dispatches callbacks
 *   send mutex    — protects TCP write from app_task and rx_task
 *
 * Network cycle (connect_task):
 *   1. Load NVS list, sort by priority, put last-successful first
 *   2. For each network: esp_wifi_connect(), wait WIFI_SINK_CONNECT_TIMEOUT_MS
 *   3. On IP acquired: tcp_connect(), start rx_task
 *   4. On TCP disconnect or WiFi disconnect: kill rx_task, back to step 2
 */

#include "wifi_sink.h"
#include "config.h"
#include "mdns.h"

#include <string.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "wifi_sink";

/* ── Event group bits ───────────────────────────────────────────────────── */
#define BIT_WIFI_CONNECTED  BIT0
#define BIT_WIFI_FAIL       BIT1
#define BIT_TCP_CONNECTED   BIT2
#define BIT_TCP_FAIL        BIT3
#define BIT_DISCONNECT      BIT4   /* signal connect_task to cycle */

/* ── Internal state ─────────────────────────────────────────────────────── */
static wifi_sink_config_t s_cfg;
static wifi_sink_state_t  s_state      = WIFI_STATE_IDLE;
static char               s_cur_ssid[WIFI_SINK_SSID_LEN] = {0};
static int                s_tcp_fd     = -1;
static SemaphoreHandle_t  s_send_mutex = NULL;
static EventGroupHandle_t s_eg         = NULL;
static TaskHandle_t       s_rx_task    = NULL;
static bool               s_wifi_started    = false;
static char               s_pi_resolved_ip[16] = {0};
static char               s_own_ip[16]         = {0};

/* NVS network list (loaded into RAM at init) */
static wifi_network_t s_networks[WIFI_SINK_MAX_NETWORKS];
static uint8_t        s_net_count   = 0;
static SemaphoreHandle_t s_net_mutex = NULL;
static volatile uint8_t s_try_idx   = 0;   /* 1-based current attempt, 0=idle */
static volatile uint8_t s_try_total = 0;   /* total networks in this cycle    */   /* protects s_networks / s_net_count */

/* TCP RX buffer */
#define RX_BUF_SIZE 1024
static uint8_t s_rx_buf[RX_BUF_SIZE];
static int     s_rx_pos = 0;

/* ── Forward declarations ───────────────────────────────────────────────── */
static void connect_task(void *arg);
static void rx_task(void *arg);
static void dispatch_frame(uint8_t ftype, uint8_t address,
                           const uint8_t *payload, uint16_t len);
static void send_net_report(uint8_t idx, const char *ssid);
static void send_cfg_ack(uint8_t idx, uint8_t result);
static esp_err_t tcp_connect(void);
static void tcp_close(void);
static esp_err_t nvs_load_networks(void);
static esp_err_t nvs_save_networks(void);
static void set_state(wifi_sink_state_t st, const char *ssid);

/* ── WiFi event handler ─────────────────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected");
        xEventGroupSetBits(s_eg, BIT_WIFI_FAIL | BIT_DISCONNECT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_own_ip, sizeof(s_own_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", s_own_ip);
        xEventGroupSetBits(s_eg, BIT_WIFI_CONNECTED);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * wifi_sink_preinit — Phase 1, call at boot.
 *
 * Creates the send/net mutexes and event group, loads stored networks from
 * NVS, and seeds the default network list if NVS is empty.  Does NOT touch
 * the WiFi radio, event loop, or start any tasks — safe to call while running
 * on 3V3-only power (BQ25895 absent / 12 V not present).
 */
esp_err_t wifi_sink_preinit(void)
{
    s_send_mutex = xSemaphoreCreateMutex();
    s_net_mutex  = xSemaphoreCreateMutex();
    s_eg         = xEventGroupCreate();
    if (!s_send_mutex || !s_net_mutex || !s_eg) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nvs_load_networks();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No NVS networks (first boot?)");
    }
    ESP_LOGI(TAG, "preinit: %d network(s) loaded from NVS", s_net_count);

    if (s_net_count == 0) {
        typedef struct { const char *ssid; const char *pass; uint8_t pri; } _def_t;
        static const _def_t _defs[] = WIFI_DEFAULT_NETWORKS;
        for (int _i = 0; _i < (int)(sizeof(_defs)/sizeof(_defs[0])); _i++)
            wifi_sink_add_network(_defs[_i].ssid, _defs[_i].pass, _defs[_i].pri);
    }

    return ESP_OK;
}

/**
 * wifi_sink_connect — Phase 2, call when user enters the WiFi sink page.
 *
 * Initialises the WiFi driver, default event loop, and starts connect_task.
 * Idempotent: safe to call multiple times (subsequent calls are no-ops).
 * wifi_sink_preinit() MUST have been called first.
 */
esp_err_t wifi_sink_connect(const wifi_sink_config_t *cfg)
{
    if (s_wifi_started) {
        ESP_LOGD(TAG, "wifi_sink_connect: already started, ignoring");
        return ESP_OK;
    }
    if (!s_send_mutex || !s_net_mutex || !s_eg) {
        ESP_LOGE(TAG, "wifi_sink_connect: call wifi_sink_preinit() first");
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(&s_cfg, cfg, sizeof(wifi_sink_config_t));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_started = true;
    xTaskCreate(connect_task, "wifi_connect", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "wifi_sink_connect started, address=%d pi=%s:%d",
             cfg->address, cfg->pi_hostname, cfg->tcp_port);
    return ESP_OK;
}

/** Legacy combined init — preinit + connect in one call. */
esp_err_t wifi_sink_init(const wifi_sink_config_t *cfg)
{
    esp_err_t err = wifi_sink_preinit();
    if (err != ESP_OK) return err;
    return wifi_sink_connect(cfg);
}

esp_err_t wifi_sink_send(uint8_t frame_type, const uint8_t *payload, uint16_t len)
{
    if (s_tcp_fd < 0) return ESP_ERR_INVALID_STATE;

    /* Frame: [length:2LE][frame_type:1][address:1][payload] */
    uint8_t hdr[4];
    hdr[0] = (uint8_t)(len & 0xFF);
    hdr[1] = (uint8_t)(len >> 8);
    hdr[2] = frame_type;
    hdr[3] = s_cfg.address;

    if (xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    int fd = s_tcp_fd;
    bool ok = true;
    if (fd >= 0) {
        ok = (send(fd, hdr, sizeof(hdr), MSG_NOSIGNAL) == sizeof(hdr));
        if (ok && len > 0) {
            ok = (send(fd, payload, len, MSG_NOSIGNAL) == len);
        }
    } else {
        ok = false;
    }
    xSemaphoreGive(s_send_mutex);

    if (!ok) {
        ESP_LOGW(TAG, "send() failed — triggering reconnect");
        xEventGroupSetBits(s_eg, BIT_DISCONNECT);
        return ESP_FAIL;
    }
    return ESP_OK;
}

wifi_sink_state_t wifi_sink_get_state(void) { return s_state; }
bool              wifi_sink_connected(void)  { return s_tcp_fd >= 0; }
const char       *wifi_sink_current_ssid(void) { return s_cur_ssid; }
const char       *wifi_sink_own_ip(void)       { return s_own_ip; }
const char       *wifi_sink_pi_ip(void)        { return s_pi_resolved_ip; }
void              wifi_sink_try_progress(uint8_t *idx, uint8_t *total)
{
    if (idx)   *idx   = s_try_idx;
    if (total) *total = s_try_total;
}

esp_err_t wifi_sink_add_network(const char *ssid, const char *password,
                                uint8_t priority)
{
    if (!ssid || strlen(ssid) == 0) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_net_mutex, portMAX_DELAY);

    /* Update existing entry if SSID matches */
    for (uint8_t i = 0; i < s_net_count; i++) {
        if (strcmp(s_networks[i].ssid, ssid) == 0) {
            strlcpy(s_networks[i].password, password, WIFI_SINK_PASS_LEN);
            s_networks[i].priority = priority;
            xSemaphoreGive(s_net_mutex);
            return nvs_save_networks();
        }
    }
    /* Add new entry */
    if (s_net_count >= WIFI_SINK_MAX_NETWORKS) {
        xSemaphoreGive(s_net_mutex);
        return ESP_ERR_NO_MEM;
    }
    strlcpy(s_networks[s_net_count].ssid,     ssid,     WIFI_SINK_SSID_LEN);
    strlcpy(s_networks[s_net_count].password, password, WIFI_SINK_PASS_LEN);
    s_networks[s_net_count].priority = priority;
    s_net_count++;
    xSemaphoreGive(s_net_mutex);
    ESP_LOGI(TAG, "Network saved: '%s' (priority=%d)", ssid, priority);
    return nvs_save_networks();
}

uint8_t wifi_sink_network_count(void) { return s_net_count; }

esp_err_t wifi_sink_get_network(uint8_t idx, wifi_network_t *out)
{
    if (idx >= s_net_count || !out) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_net_mutex, portMAX_DELAY);
    memcpy(out, &s_networks[idx], sizeof(wifi_network_t));
    xSemaphoreGive(s_net_mutex);
    return ESP_OK;
}

/* ── NVS helpers ────────────────────────────────────────────────────────── */

static esp_err_t nvs_load_networks(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_SINK_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    uint8_t count = 0;
    nvs_get_u8(h, WIFI_SINK_NVS_COUNT, &count);
    count = count > WIFI_SINK_MAX_NETWORKS ? WIFI_SINK_MAX_NETWORKS : count;

    for (uint8_t i = 0; i < count; i++) {
        char key[16];
        size_t len;

        snprintf(key, sizeof(key), "ssid%d", i);
        len = WIFI_SINK_SSID_LEN;
        nvs_get_str(h, key, s_networks[i].ssid, &len);

        snprintf(key, sizeof(key), "pass%d", i);
        len = WIFI_SINK_PASS_LEN;
        nvs_get_str(h, key, s_networks[i].password, &len);

        snprintf(key, sizeof(key), "prio%d", i);
        uint8_t prio = 50;
        nvs_get_u8(h, key, &prio);
        s_networks[i].priority = prio;
    }
    s_net_count = count;
    nvs_close(h);
    return ESP_OK;
}

static esp_err_t nvs_save_networks(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_SINK_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    xSemaphoreTake(s_net_mutex, portMAX_DELAY);
    nvs_set_u8(h, WIFI_SINK_NVS_COUNT, s_net_count);
    for (uint8_t i = 0; i < s_net_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ssid%d", i);
        nvs_set_str(h, key, s_networks[i].ssid);
        snprintf(key, sizeof(key), "pass%d", i);
        nvs_set_str(h, key, s_networks[i].password);
        snprintf(key, sizeof(key), "prio%d", i);
        nvs_set_u8(h, key, s_networks[i].priority);
    }
    xSemaphoreGive(s_net_mutex);

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* ── State helper ───────────────────────────────────────────────────────── */

static void set_state(wifi_sink_state_t st, const char *ssid)
{
    s_state = st;
    if (ssid) strlcpy(s_cur_ssid, ssid, WIFI_SINK_SSID_LEN);
    if (s_cfg.on_state) s_cfg.on_state(st, s_cur_ssid, s_cfg.ctx);
}

/* ── Connect task ───────────────────────────────────────────────────────── */

/**
 * Builds a sorted network list: last-successful first, then rest by priority.
 * Caller must hold s_net_mutex or call before tasks start.
 */
static uint8_t build_try_order(uint8_t *order_out)
{
    uint8_t last = 0xFF;
    nvs_handle_t h;
    if (nvs_open(WIFI_SINK_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, WIFI_SINK_NVS_LAST, &last);
        nvs_close(h);
    }

    uint8_t count = 0;
    /* Last-successful first */
    if (last < s_net_count) {
        order_out[count++] = last;
    }
    /* Remaining in priority order (simple insertion sort) */
    for (uint8_t i = 0; i < s_net_count; i++) {
        if (i == last) continue;
        order_out[count++] = i;
    }
    return count;
}

static void connect_task(void *arg)
{
    for (;;) {
        xSemaphoreTake(s_net_mutex, portMAX_DELAY);
        uint8_t order[WIFI_SINK_MAX_NETWORKS];
        uint8_t n = build_try_order(order);
        xSemaphoreGive(s_net_mutex);

        if (n == 0) {
            s_try_idx = 0; s_try_total = 0;
            ESP_LOGW(TAG, "No networks configured — waiting 10s");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        s_try_total = n;

        for (uint8_t oi = 0; oi < n; oi++) {
            s_try_idx = oi + 1;   /* 1-based */
            uint8_t idx = order[oi];
            wifi_network_t net;
            wifi_sink_get_network(idx, &net);

            ESP_LOGI(TAG, "Trying network [%d]: '%s'", idx, net.ssid);
            set_state(WIFI_STATE_WIFI_CONNECTING, net.ssid);

            /* Configure and connect */
            wifi_config_t wcfg = {0};
            strlcpy((char *)wcfg.sta.ssid,     net.ssid,     sizeof(wcfg.sta.ssid));
            strlcpy((char *)wcfg.sta.password,  net.password, sizeof(wcfg.sta.password));
            wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

            /* Disconnect first, then wait for the resulting STA_DISCONNECTED
             * event to fire and set BIT_WIFI_FAIL, THEN clear — so the clear
             * always wins and the stale event can't poison our wait below. */
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(150));
            xEventGroupClearBits(s_eg,
                BIT_WIFI_CONNECTED | BIT_WIFI_FAIL | BIT_DISCONNECT);
            esp_wifi_set_config(WIFI_IF_STA, &wcfg);
            esp_wifi_connect();

            EventBits_t bits = xEventGroupWaitBits(
                s_eg, BIT_WIFI_CONNECTED | BIT_WIFI_FAIL, pdTRUE, pdFALSE,
                pdMS_TO_TICKS(WIFI_SINK_CONNECT_TIMEOUT_MS));

            if (!(bits & BIT_WIFI_CONNECTED)) {
                ESP_LOGW(TAG, "'%s' failed", net.ssid);
                vTaskDelay(pdMS_TO_TICKS(WIFI_SINK_RECONNECT_DELAY_MS));
                continue;
            }

            /* WiFi connected — try TCP */
            set_state(WIFI_STATE_TCP_CONNECTING, net.ssid);
            if (tcp_connect() != ESP_OK) {
                ESP_LOGW(TAG, "TCP connect to %s:%d failed",
                         s_cfg.pi_hostname, s_cfg.tcp_port);
                esp_wifi_disconnect();
                vTaskDelay(pdMS_TO_TICKS(WIFI_SINK_RECONNECT_DELAY_MS));
                continue;
            }

            /* Save this as last-successful */
            nvs_handle_t h;
            if (nvs_open(WIFI_SINK_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_u8(h, WIFI_SINK_NVS_LAST, idx);
                nvs_commit(h);
                nvs_close(h);
            }

            set_state(WIFI_STATE_CONNECTED, net.ssid);
            ESP_LOGI(TAG, "Connected: '%s' → %s:%d", net.ssid,
                     s_pi_resolved_ip[0] ? s_pi_resolved_ip : s_cfg.pi_hostname,
                     s_cfg.tcp_port);

            /* Clear stale disconnect bit BEFORE spawning rx_task —
             * if we cleared it after, rx_task could fire and set it
             * before the clear, causing us to wait here forever. */
            xEventGroupClearBits(s_eg, BIT_DISCONNECT);

            /* Start RX task */
            xTaskCreate(rx_task, "wifi_rx", 4096, NULL, 6, &s_rx_task);

            /* Wait for disconnect signal */
            xEventGroupWaitBits(s_eg, BIT_DISCONNECT, pdTRUE, pdFALSE, portMAX_DELAY);

            /* Tear down */
            set_state(WIFI_STATE_RECONNECTING, NULL);
            /* rx_task already called vTaskDelete(NULL) before setting BIT_DISCONNECT —
             * do NOT call vTaskDelete again on the same handle (double-delete → crash).
             * Just clear the handle so it won't be touched again. */
            s_rx_task = NULL;
            tcp_close();           /* close socket before WiFi stack tears down */
            esp_wifi_disconnect();
            s_try_idx = 0; s_try_total = 0;  /* reset progress display during delay */
            vTaskDelay(pdMS_TO_TICKS(WIFI_SINK_RECONNECT_DELAY_MS));
            break;   /* restart outer loop from beginning to re-sort */
        }
        /* ── All networks exhausted without a successful connection ──────
         * Power-cycle the WiFi radio to clear any stuck association state,
         * then wait before the next round of attempts.               */
        ESP_LOGW(TAG, "All networks failed — radio power cycle");
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_start();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ── TCP helpers ────────────────────────────────────────────────────────── */

static esp_err_t tcp_connect(void)
{
    /* Resolve pi_hostname via mDNS; fall back to WIFI_SINK_PI_FALLBACK_IP */
    {
        char _fqdn[WIFI_SINK_HOSTNAME_LEN + 7];
        snprintf(_fqdn, sizeof(_fqdn), "%s.local", s_cfg.pi_hostname);
        esp_ip4_addr_t _res = {0};
        bool _ok = false;
        for (int _t = 0; _t < WIFI_SINK_MDNS_RETRIES; _t++) {
            if (mdns_query_a(_fqdn, 2000, &_res) == ESP_OK) { _ok = true; break; }
            ESP_LOGW(TAG, "mDNS query attempt %d/%d for %s failed: ESP_ERR_NOT_FOUND",
                     _t + 1, WIFI_SINK_MDNS_RETRIES, _fqdn);
        }
        if (_ok)
            snprintf(s_pi_resolved_ip, sizeof(s_pi_resolved_ip), IPSTR, IP2STR(&_res));
        else {
            ESP_LOGW(TAG, "mDNS resolve '%s' failed — using fallback %s",
                     _fqdn, WIFI_SINK_PI_FALLBACK_IP);
            strlcpy(s_pi_resolved_ip, WIFI_SINK_PI_FALLBACK_IP, sizeof(s_pi_resolved_ip));
        }
    }
    struct sockaddr_in sa = {
        .sin_family = AF_INET,
        .sin_port   = htons(s_cfg.tcp_port),
    };
    if (inet_pton(AF_INET, s_pi_resolved_ip, &sa.sin_addr) != 1) {
        ESP_LOGE(TAG, "Bad resolved IP: %s", s_pi_resolved_ip);
        return ESP_ERR_INVALID_ARG;
    }

    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return ESP_FAIL;

    struct timeval tv = { .tv_sec = WIFI_SINK_TCP_TIMEOUT_MS / 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        close(fd);
        return ESP_FAIL;
    }
    s_tcp_fd = fd;
    return ESP_OK;
}

static void tcp_close(void)
{
    if (xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        if (s_tcp_fd >= 0) {
            close(s_tcp_fd);
            s_tcp_fd = -1;
        }
        xSemaphoreGive(s_send_mutex);
    }
}

/* ── RX task ────────────────────────────────────────────────────────────── */

static void rx_task(void *arg)
{
    s_rx_pos = 0;
    for (;;) {
        int fd = s_tcp_fd;
        if (fd < 0) break;

        int n = recv(fd, s_rx_buf + s_rx_pos, sizeof(s_rx_buf) - s_rx_pos, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* SO_RCVTIMEO fired — connection still alive, keep looping */
                continue;
            }
            ESP_LOGW(TAG, "recv() returned %d (errno=%d) — disconnecting", n, errno);
            xEventGroupSetBits(s_eg, BIT_DISCONNECT);
            break;
        }
        if (n == 0) {
            ESP_LOGW(TAG, "recv() returned 0 — server closed connection");
            xEventGroupSetBits(s_eg, BIT_DISCONNECT);
            break;
        }
        s_rx_pos += n;

        /* Drain complete frames from buffer */
        int consumed = 0;
        while (s_rx_pos - consumed >= 4) {
            uint16_t length    = (uint16_t)s_rx_buf[consumed]
                               | ((uint16_t)s_rx_buf[consumed + 1] << 8);
            uint8_t  frame_type = s_rx_buf[consumed + 2];
            uint8_t  address    = s_rx_buf[consumed + 3];
            int total = 4 + length;
            if (s_rx_pos - consumed < total) break;

            dispatch_frame(frame_type, address,
                           s_rx_buf + consumed + 4, length);
            consumed += total;
        }

        /* Shift unconsumed bytes to front */
        if (consumed > 0 && consumed < s_rx_pos) {
            memmove(s_rx_buf, s_rx_buf + consumed, s_rx_pos - consumed);
        }
        s_rx_pos -= consumed;
        if (s_rx_pos < 0) s_rx_pos = 0;
    }
    vTaskDelete(NULL);
}

/* ── Frame dispatcher ───────────────────────────────────────────────────── */

static void dispatch_frame(uint8_t ftype, uint8_t address,
                           const uint8_t *payload, uint16_t len)
{
    switch (ftype) {

    case FT_CMD:
        if (len >= sizeof(wifi_cmd_t) && s_cfg.on_cmd) {
            wifi_cmd_t cmd;
            memcpy(&cmd, payload, sizeof(wifi_cmd_t));
            /* Convert big-endian fields */
            cmd.dsc_pulse_ms   = ntohs(cmd.dsc_pulse_ms);
            cmd.settle_minutes = ntohl(cmd.settle_minutes);
            s_cfg.on_cmd(&cmd, s_cfg.ctx);
        }
        break;

    case FT_WIFI_NET_REQ:
        /* Pi requests our full network list — reply with one REPORT per entry */
        ESP_LOGI(TAG, "NET_REQ received — reporting %d networks", s_net_count);
        for (uint8_t i = 0; i < s_net_count; i++) {
            send_net_report(i, s_networks[i].ssid);
        }
        break;

    case FT_WIFI_CFG_PUSH:
        /* Push: [idx:1][ssid:33][pass:65] = 99 bytes */
        if (len >= 99) {
            uint8_t idx  = payload[0];
            char ssid[WIFI_SINK_SSID_LEN] = {0};
            char pass[WIFI_SINK_PASS_LEN] = {0};
            memcpy(ssid, payload + 1,  32); ssid[32] = 0;
            memcpy(pass, payload + 34, 64); pass[64] = 0;

            ESP_LOGI(TAG, "CFG_PUSH idx=%d ssid='%s'", idx, ssid);
            esp_err_t err = wifi_sink_add_network(ssid, pass, 50);
            send_cfg_ack(idx, err == ESP_OK ? 0 : 1);
        }
        break;

    default:
        ESP_LOGD(TAG, "Unhandled frame type 0x%02X", ftype);
        break;
    }
}

static void send_net_report(uint8_t idx, const char *ssid)
{
    /* payload: [idx:1][ssid:33 null-padded] */
    uint8_t buf[34] = {0};
    buf[0] = idx;
    strlcpy((char *)buf + 1, ssid, 33);
    wifi_sink_send(FT_WIFI_NET_RPT, buf, sizeof(buf));
}

static void send_cfg_ack(uint8_t idx, uint8_t result)
{
    uint8_t buf[2] = { idx, result };
    wifi_sink_send(FT_WIFI_CFG_ACK, buf, sizeof(buf));
}