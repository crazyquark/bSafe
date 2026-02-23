// =============================================================================
// board.c — hardware abstraction (port of board.py)
// =============================================================================
#include "board.h"
#include "config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "board";

// ---------------------------------------------------------------------------
// Internal: I2C bus scan — returns true if addr is present
// ---------------------------------------------------------------------------
static bool _i2c_probe(board_t *b, uint8_t addr)
{
    // i2c_master_probe() is available from IDF 5.1+
    esp_err_t ret = i2c_master_probe(b->i2c_bus, addr, 50);
    return ret == ESP_OK;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

esp_err_t board_init_i2c(board_t *b)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port         = I2C_PORT_NUM,
        .sda_io_num       = I2C_SDA_GPIO,
        .scl_io_num       = I2C_SCL_GPIO,
        .clk_source       = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  // use external pull-ups
    };

    esp_err_t ret = i2c_new_master_bus(&cfg, &b->i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C bus initialised (SDA=%d SCL=%d @ %d Hz)",
                 I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);
    }
    return ret;
}

esp_err_t board_init_expander(board_t *b)
{
    b->has_exp = false;

    if (!_i2c_probe(b, TCA9535_ADDR)) {
        ESP_LOGW(TAG, "TCA9535 not found at 0x%02X", TCA9535_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t ret = tca9535_init(b->i2c_bus, TCA9535_ADDR, &b->exp);
    if (ret != ESP_OK) goto fail;

    ret = tca9535_configure(&b->exp, TCA_DIR0, TCA_DIR1);
    if (ret != ESP_OK) goto fail;

    // Safe default output states
    tca9535_write_pin(&b->exp, P_CE,  true);   // /CE HIGH = disabled
    tca9535_write_pin(&b->exp, P_QON, false);
    tca9535_write_pin(&b->exp, P_FAN, false);

    // Prime input cache
    tca9535_read_all(&b->exp, &b->in0, &b->in1);

    b->has_exp = true;
    ESP_LOGI(TAG, "TCA9535 OK (in0=0x%02X in1=0x%02X)", b->in0, b->in1);
    return ESP_OK;

fail:
    ESP_LOGW(TAG, "TCA9535 init failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t board_init_ina226(board_t *b, float max_current_a)
{
    b->has_ina_batt = false;
    b->has_ina_sys  = false;

    // Battery-side INA226
    if (_i2c_probe(b, INA226_ADDR_BATT)) {
        esp_err_t ret = ina226_init(b->i2c_bus, INA226_ADDR_BATT,
                                    SHUNT_RESISTANCE_OHMS, &b->ina_batt);
        if (ret == ESP_OK) ret = ina226_reset(&b->ina_batt);
        if (ret == ESP_OK) ret = ina226_configure(&b->ina_batt,
                                                   INA226_AVG_128,
                                                   INA226_CT_332US,
                                                   INA226_CT_332US,
                                                   INA226_MODE_SHUNT_BUS_CONT);
        if (ret == ESP_OK) ret = ina226_calibrate_for_max_current(&b->ina_batt, max_current_a);

        if (ret == ESP_OK) {
            b->has_ina_batt = true;
            ESP_LOGI(TAG, "INA226 batt OK");
        } else {
            ESP_LOGW(TAG, "INA226 batt init failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "INA226 batt not found at 0x%02X", INA226_ADDR_BATT);
    }

    // System-side INA226
    if (_i2c_probe(b, INA226_ADDR_SYS)) {
        esp_err_t ret = ina226_init(b->i2c_bus, INA226_ADDR_SYS,
                                    SHUNT_RESISTANCE_OHMS, &b->ina_sys);
        if (ret == ESP_OK) ret = ina226_reset(&b->ina_sys);
        if (ret == ESP_OK) ret = ina226_configure(&b->ina_sys,
                                                   INA226_AVG_128,
                                                   INA226_CT_332US,
                                                   INA226_CT_332US,
                                                   INA226_MODE_SHUNT_BUS_CONT);
        if (ret == ESP_OK) ret = ina226_calibrate_for_max_current(&b->ina_sys, max_current_a);

        if (ret == ESP_OK) {
            b->has_ina_sys = true;
            ESP_LOGI(TAG, "INA226 sys OK");
        } else {
            ESP_LOGW(TAG, "INA226 sys init failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "INA226 sys not found at 0x%02X", INA226_ADDR_SYS);
    }

    return ESP_OK;  // partial presence is OK
}

esp_err_t board_init_bq25895(board_t *b)
{
    b->has_bq = false;

    if (!_i2c_probe(b, BQ25895_ADDR)) {
        ESP_LOGW(TAG, "BQ25895 not found at 0x%02X", BQ25895_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t ret = bq25895_init(b->i2c_bus, BQ25895_ADDR, &b->bq);
    if (ret != ESP_OK) goto fail;

    // Safe defaults: charge disabled, not in HiZ
    bq25895_set_charge_enable(&b->bq, false);
    bq25895_set_hiz(&b->bq, false);

    b->has_bq = true;
    ESP_LOGI(TAG, "BQ25895 OK");
    return ESP_OK;

fail:
    ESP_LOGW(TAG, "BQ25895 init failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t board_init_twai(board_t *b)
{
    b->has_twai = false;

    // General config
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_GPIO,
        (gpio_num_t)TWAI_RX_GPIO,
        TWAI_MODE_NO_ACK          // <-- was TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    g_config.alerts_enabled = TWAI_ALERT_BUS_ERROR
                             | TWAI_ALERT_BUS_OFF
                             | TWAI_ALERT_ERR_PASS
                             | TWAI_ALERT_RX_QUEUE_FULL;

    // Timing — use preset macro for common bitrates
    // Presets: TWAI_TIMING_CONFIG_25KBITS() up to TWAI_TIMING_CONFIG_1MBITS()
#if   TWAI_BITRATE == 125000
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
#elif TWAI_BITRATE == 250000
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
#elif TWAI_BITRATE == 500000
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#elif TWAI_BITRATE == 1000000
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
#else
    #error "Unsupported TWAI_BITRATE — add case or use custom timing"
#endif

    // Accept all frames; apply filters later when message IDs are defined
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }

    b->has_twai = true;
    ESP_LOGI(TAG, "TWAI OK (TX=%d RX=%d @ %d bps)",
             TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_BITRATE);
    return ESP_OK;
}

esp_err_t board_init_all(board_t *b, float max_current_a)
{
    memset(b, 0, sizeof(*b));

    esp_err_t ret = board_init_i2c(b);
    if (ret != ESP_OK) return ret;  // I2C is mandatory

    board_init_ssd1306(b);
    board_init_expander(b);         // optional — log WARN on absent
    board_init_ina226(b, max_current_a);
    board_init_bq25895(b);
    board_init_twai(b);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Control helpers
// ---------------------------------------------------------------------------

esp_err_t board_set_ce_enabled(board_t *b, bool enabled)
{
    if (!b->has_exp) return ESP_OK;
    // /CE is active-low: enabled → drive LOW, disabled → drive HIGH
    return tca9535_write_pin(&b->exp, P_CE, !enabled);
}

esp_err_t board_set_qon(board_t *b, bool enabled)
{
    if (!b->has_exp) return ESP_OK;
    return tca9535_write_pin(&b->exp, P_QON, enabled);
}

esp_err_t board_set_fan(board_t *b, bool enabled)
{
    if (!b->has_exp) return ESP_OK;
    return tca9535_write_pin(&b->exp, P_FAN, enabled);
}

// ---------------------------------------------------------------------------
// TWAI helpers
// ---------------------------------------------------------------------------

esp_err_t board_twai_transmit(board_t *b, uint32_t id,
                               const uint8_t *data, uint8_t len,
                               uint32_t timeout_ms)
{
    if (!b->has_twai) return ESP_ERR_INVALID_STATE;

    twai_message_t msg = {
        .identifier = id,
        .data_length_code = len,
        .extd = 0,  // standard 11-bit frame
        .rtr  = 0,
    };
    if (len > 8) len = 8;
    for (int i = 0; i < len; i++) msg.data[i] = data[i];

    return twai_transmit(&msg, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t board_twai_receive(board_t *b, twai_message_t *msg, uint32_t timeout_ms)
{
    if (!b->has_twai) return ESP_ERR_INVALID_STATE;
    return twai_receive(msg, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t board_twai_get_status(board_t *b, twai_status_info_t *info)
{
    if (!b->has_twai) return ESP_ERR_INVALID_STATE;
    return twai_get_status_info(info);
}

esp_err_t board_init_ssd1306(board_t *b)
{
    b->has_lcd = false;

    if (!_i2c_probe(b, OLED_ADDR)) {
        ESP_LOGW(TAG, "SSD1306 not found at 0x%02X", OLED_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    // Use v2 API — matches i2c_master_bus_handle_t (new driver)
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr            = OLED_ADDR,
        .scl_speed_hz        = 400000,   // valid in v2 only
        .control_phase_bytes = 1,
        .dc_bit_offset       = 6,        // SSD1306: D/C# is bit 6 of control byte
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
    };

    esp_err_t ret = esp_lcd_new_panel_io_i2c_v2(
        b->i2c_bus,   // i2c_master_bus_handle_t — no cast needed
        &io_cfg,
        &b->lcd_io
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD IO init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };

    esp_lcd_panel_ssd1306_config_t ssd1306_cfg = {
        .height = 64,
    };
    panel_cfg.vendor_config = &ssd1306_cfg;

    ret = esp_lcd_new_panel_ssd1306(b->lcd_io, &panel_cfg, &b->lcd_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 panel init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(b->lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(b->lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(b->lcd_panel, true));

    b->has_lcd = true;
    ESP_LOGI(TAG, "SSD1306 OK");
    return ESP_OK;
}