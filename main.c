// =============================================================================
// main.c — application entry point
// Demonstrates: I2C bus scan + board init + TWAI receive loop
// =============================================================================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "display.h"
#include "ui_demo.h"
#include "config.h"
#include "board.h"
#include "display.h"
#include "ui_demo.h"
#include "esp_timer.h"

static const char *TAG = "main";
static display_t   disp;
static ui_state_t  ui;

// ---------------------------------------------------------------------------
// I2C bus scan helper — mirrors MicroPython's i2c.scan()
// ---------------------------------------------------------------------------
static void do_i2c_scan(board_t *b)
{
    ESP_LOGI(TAG, "---- I2C scan (0x08..0x77) ----");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(b->i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  Found: 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "  %d device(s) found", found);
    ESP_LOGI(TAG, "--------------------------------");
}

// ---------------------------------------------------------------------------
// TWAI receive task — runs concurrently with main logic
// ---------------------------------------------------------------------------
static void twai_rx_task(void *arg)
{
    board_t *b = (board_t *)arg;
    ESP_LOGI(TAG, "TWAI RX task started");

    while (1) {
        twai_message_t msg;
        esp_err_t ret = board_twai_receive(b, &msg, 100);

        if (ret == ESP_OK) {
            // Got a frame
            ESP_LOGI(TAG, "CAN RX  ID=0x%03" PRIx32 "  DLC=%d  data:",
                     msg.identifier, msg.data_length_code);
            for (int i = 0; i < msg.data_length_code; i++) {
                printf("  [%d]=0x%02X", i, msg.data[i]);
            }
            printf("\n");

        } else if (ret == ESP_ERR_TIMEOUT) {
            // Nothing received — check for bus alerts
            uint32_t alerts;
            if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) {
                if (alerts & TWAI_ALERT_BUS_OFF) {
                    ESP_LOGE(TAG, "CAN BUS OFF — attempting recovery");
                    twai_initiate_recovery();
                }
                if (alerts & TWAI_ALERT_BUS_ERROR) {
                    ESP_LOGW(TAG, "CAN bus error (alerts=0x%" PRIx32 ")", alerts);
                }
                if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                    ESP_LOGW(TAG, "CAN RX queue full — frames dropped");
                }
            }
        } else {
            ESP_LOGE(TAG, "TWAI receive error: %s", esp_err_to_name(ret));
        }
    }
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "=== charger_fw starting ===");

    // Configure BOOT button GPIO (OK button, active-low)
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << GPIO_BTN_OK,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    // Initialise board (I2C + all optional peripherals + TWAI)
    static board_t board;
    ESP_ERROR_CHECK(board_init_all(&board, 5.0f));  // 5A max current

    // Init display (board_init_ssd1306 must have run inside board_init_all)
    if (board.has_lcd) {
        display_init(&disp, board.lcd_panel);
        ui_init(&ui, &disp);
        ESP_LOGI(TAG, "Display ready");
    }

    // Show what we found on the bus
    do_i2c_scan(&board);

    ESP_LOGI(TAG, "Board status:");
    ESP_LOGI(TAG, "  TCA9535 expander : %s", board.has_exp      ? "OK" : "absent");
    ESP_LOGI(TAG, "  INA226 batt      : %s", board.has_ina_batt ? "OK" : "absent");
    ESP_LOGI(TAG, "  INA226 sys       : %s", board.has_ina_sys  ? "OK" : "absent");
    ESP_LOGI(TAG, "  BQ25895 charger  : %s", board.has_bq       ? "OK" : "absent");
    ESP_LOGI(TAG, "  TWAI/CAN         : %s", board.has_twai     ? "OK" : "absent");

    // Start TWAI RX task
    if (board.has_twai) {
        xTaskCreate(twai_rx_task, "twai_rx", 4096, &board, 5, NULL);
    }

    // ---------------------------------------------------------------------------
    // Main loop — read sensors, poll buttons, transmit periodic CAN heartbeat
    // ---------------------------------------------------------------------------
    uint32_t tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        tick++;

        // --- Button edges ---
        board_btn_edges_t btn = board_poll_buttons(&board);
        if (board.has_lcd) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (btn.right || btn.ok) {
                // Advance to next screen immediately on button press
                int next = (ui.screen + 1) % UI_SCREEN_COUNT;
                ui_force_screen(&ui, next, now_ms);
            }
        }
        if (btn.menu)  ESP_LOGI(TAG, "BTN: MENU");
        if (btn.left)  ESP_LOGI(TAG, "BTN: LEFT");
        if (btn.right) ESP_LOGI(TAG, "BTN: RIGHT");
        if (btn.ok)    ESP_LOGI(TAG, "BTN: OK");

        // --- Read sensors every 1 s ---
        if (tick % 10 == 0) {
            ui_data_t uid = {
                .chrg_label = "not_charging",
                .power_good = false,
                .fan_on     = false,
                .ce_enabled = false,
                .qon_on     = false,
                .can_ok     = board.has_twai,
                .uptime_s   = tick / 10,
            };

            if (board.has_ina_batt) {
                ina226_reading_t r;
                if (ina226_read_all(&board.ina_batt, &r) == ESP_OK) {
                    uid.vbat = r.bus_v;
                    uid.ibat = r.current_a;
                    uid.pbat = r.power_w;
                    ESP_LOGI(TAG, "INA_BATT  Vbus=%.3fV  I=%.3fA  P=%.2fW",
                             r.bus_v, r.current_a, r.power_w);
                }
            }

            if (board.has_ina_sys) {
                ina226_reading_t r;
                if (ina226_read_all(&board.ina_sys, &r) == ESP_OK) {
                    uid.vsys = r.bus_v;
                    ESP_LOGI(TAG, "INA_SYS   Vbus=%.3fV  I=%.3fA  P=%.2fW",
                             r.bus_v, r.current_a, r.power_w);
                }
            }

            if (board.has_bq) {
                bq25895_status_t st;
                float vbus = 0.0f;
                bq25895_read_status(&board.bq, &st);
                bq25895_read_vbus_v(&board.bq, &vbus);
                uid.vbus        = vbus;
                uid.power_good  = st.power_good;
                uid.chrg_label  = bq25895_chrg_stat_label(st.chrg_stat);
                // Rough charge % from vbat (3.0V=0%, 4.2V=100% for LCO)
                uid.charge_pct  = (int)(((uid.vbat - 3.0f) / 1.2f) * 100.0f);
                if (uid.charge_pct < 0)   uid.charge_pct = 0;
                if (uid.charge_pct > 100) uid.charge_pct = 100;
                bq25895_watchdog_kick(&board.bq);
            }

            // Update UI
            if (board.has_lcd) {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                ui_update(&ui, &uid, now_ms);
            }

            // CAN heartbeat
            if (board.has_twai) {
                uint8_t payload[4] = {
                    (tick >> 24) & 0xFF, (tick >> 16) & 0xFF,
                    (tick >>  8) & 0xFF, (tick >>  0) & 0xFF,
                };
                if (board_twai_transmit(&board, 0x100, payload, 4, 10) != ESP_OK) {
                    ESP_LOGW(TAG, "CAN TX failed");
                }
            }
        }
    }
}
