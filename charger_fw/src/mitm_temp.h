#pragma once
// =============================================================================
// mitm_temp.h — Temperature differential diagnostic
// =============================================================================
#include "board.h"
#include "display.h"
#include "esp_adc/adc_oneshot.h"

/**
 * @brief Blocking diagnostic loop — call from app_main() instead of app_run().
 *
 * Initialises AHT20 (0x38) and BMP280 (0x76) internally.
 * ADC_UNIT_1 can only be instantiated once — pass the handle already created
 * by app_init() (via _ext_adc_init) rather than creating a second one.
 *
 * To expose the handle, in app.c make s_adc non-static (or add it to app_t):
 *
 *   // app.c — change:
 *   static adc_oneshot_unit_handle_t s_adc = NULL;
 *   // to:
 *   adc_oneshot_unit_handle_t s_adc = NULL;
 *
 *   // app.h — add extern declaration:
 *   extern adc_oneshot_unit_handle_t s_adc;
 *
 * Usage in main.c:
 *
 *   #include "mitm_temp.h"
 *   #include "app.h"   // for extern s_adc
 *
 *   app_init(&app, &board, &disp);   // this calls _ext_adc_init() → s_adc valid
 *   mitm_temp_run(&board, &disp, s_adc);   // replaces app_run() while debugging
 */
void mitm_temp_run(board_t *board, display_t *disp, adc_oneshot_unit_handle_t adc);
