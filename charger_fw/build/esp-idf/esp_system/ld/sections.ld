/* Automatically generated file; DO NOT EDIT */
/* Espressif IoT Development Framework Linker Script */
/* Generated from: C:\Users\Paul\Documents\GitHub\bSafe\charger_fw\build\esp-idf\esp_system\ld\sections.ld.in */

/*

 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD

 *

 * SPDX-License-Identifier: Apache-2.0

 */
/*

 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD

 *

 * SPDX-License-Identifier: Apache-2.0

 */
/*

 * Automatically generated file. DO NOT EDIT.

 * Espressif IoT Development Framework (ESP-IDF) 5.5.2 Configuration Header

 */
       
/* List of deprecated options */
/* CPU instruction prefetch padding size for flash mmap scenario */
/* Copy from esp_secure_boot.h */
/*

 * PMP region granularity size

 * Software may determine the PMP granularity by writing zero to pmp0cfg, then writing all ones

 * to pmpaddr0, then reading back pmpaddr0. If G is the index of the least-significant bit set,

 * the PMP granularity is 2^G+2 bytes.

 */
/* CPU instruction prefetch padding size for memory protection scenario */
/* Memory alignment size for PMS */
    /* rtc timer data (s_rtc_timer_retain_mem, see esp_clk.c files). For rtc_timer_data_in_rtc_mem section. */
/* Default entry point */
ENTRY(call_start_cpu0);
SECTIONS
{
  /**

   * RTC fast memory holds RTC wake stub code,

   * including from any source file named rtc_wake_stub*.c

   */
  .rtc.text :
  {
    
 . = ALIGN(4); 
 _rtc_fast_start = ABSOLUTE(.);
    HIDDEN(_rtc_code_start = .);
    *(.rtc.literal .rtc.text .rtc.text.*)
    *rtc_wake_stub*.*(.text .text.*)
    *(.rtc_text_end_test)
    HIDDEN(_rtc_code_end = .);
    /* Padding for possible CPU prefetch + 4B alignment for PMS split lines. */
    . = ((_rtc_code_end - _rtc_code_start) == 0) ?
        ALIGN(0) : 16 + ALIGN(4);
    _rtc_text_end = ABSOLUTE(.);
  } > rtc_iram_seg
  /**

   * This section located in RTC FAST Memory area.

   * It holds data marked with RTC_FAST_ATTR attribute.

   * See the file "esp_attr.h" for more information.

   */
  .rtc.force_fast :
  {
    
 . = ALIGN(4); 
 _rtc_force_fast_start = ABSOLUTE(.);
    _coredump_rtc_fast_start = ABSOLUTE(.);
    *(.rtc.fast.coredump .rtc.fast.coredump.*)
    _coredump_rtc_fast_end = ABSOLUTE(.);
    *(.rtc.force_fast .rtc.force_fast.*)
    
 . = ALIGN(4); 
 _rtc_force_fast_end = ABSOLUTE(.);
  } > rtc_data_seg
  /**

   * RTC data section holds RTC wake stub

   * data/rodata, including from any source file

   * named rtc_wake_stub*.c and the data marked with

   * RTC_DATA_ATTR, RTC_RODATA_ATTR attributes.

   */
  .rtc.data :
  {
    _rtc_data_start = ABSOLUTE(.);
    _coredump_rtc_start = ABSOLUTE(.);
    *(.rtc.coredump .rtc.coredump.*)
    _coredump_rtc_end = ABSOLUTE(.);
    *(.rtc.data .rtc.data.*)
    *(.rtc.rodata .rtc.rodata.*)
    *rtc_wake_stub*.*(.data .rodata .data.* .rodata.* .srodata.*)
    _rtc_data_end = ABSOLUTE(.);
  } > rtc_data_location
  /* RTC bss, from any source file named rtc_wake_stub*.c */
  .rtc.bss (NOLOAD) :
  {
    _rtc_bss_start = ABSOLUTE(.);
    *rtc_wake_stub*.*(.bss .bss.* .sbss .sbss.*)
    *rtc_wake_stub*.*(COMMON)
    *(.rtc.bss)
    _rtc_bss_end = ABSOLUTE(.);
  } > rtc_data_location
  /**

   * This section holds data that should not be initialized at power up

   * and will be retained during deep sleep.

   * User data marked with RTC_NOINIT_ATTR will be placed

   * into this section. See the file "esp_attr.h" for more information.

   */
  .rtc_noinit (NOLOAD):
  {
    
 . = ALIGN(4); 
 _rtc_noinit_start = ABSOLUTE(.);
    *(.rtc_noinit .rtc_noinit.*)
    
 . = ALIGN(4); 
 _rtc_noinit_end = ABSOLUTE(.);
  } > rtc_data_location
  /**

   * This section located in RTC SLOW Memory area.

   * It holds data marked with RTC_SLOW_ATTR attribute.

   * See the file "esp_attr.h" for more information.

   */
  .rtc.force_slow :
  {
    
 . = ALIGN(4); 
 _rtc_force_slow_start = ABSOLUTE(.);
    *(.rtc.force_slow .rtc.force_slow.*)
    
 . = ALIGN(4); 
 _rtc_force_slow_end = ABSOLUTE(.);
  } > rtc_slow_seg
  /**

   * This section holds RTC data that should have fixed addresses.

   * The data are not initialized at power-up and are retained during deep

   * sleep.

   */
  .rtc_reserved (NOLOAD):
  {
    
 . = ALIGN(4); 
 _rtc_reserved_start = ABSOLUTE(.);
    /**

     * New data can only be added here to ensure existing data are not moved.

     * Because data have adhered to the end of the segment and code is relied

     * on it.

     * >> put new data here <<

     */
    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    _rtc_reserved_end = ABSOLUTE(.);
  } > rtc_reserved_seg
  _rtc_reserved_length = _rtc_reserved_end - _rtc_reserved_start;
  ASSERT((_rtc_reserved_length <= LENGTH(rtc_reserved_seg)),
          "RTC reserved segment data does not fit.")
  /* Get size of rtc slow data based on rtc_data_location alias */
  _rtc_slow_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_slow_end - _rtc_data_start)
                        : (_rtc_force_slow_end - _rtc_force_slow_start);
  _rtc_fast_length = (ORIGIN(rtc_slow_seg) == ORIGIN(rtc_data_location))
                        ? (_rtc_force_fast_end - _rtc_fast_start)
                        : (_rtc_noinit_end - _rtc_fast_start);
  ASSERT((_rtc_slow_length <= LENGTH(rtc_slow_seg)),
          "RTC_SLOW segment data does not fit.")
  ASSERT((_rtc_fast_length <= LENGTH(rtc_data_seg)),
          "RTC_FAST segment data does not fit.")
  .iram0.text :
  {
    _iram_start = ABSOLUTE(.);
    /* Vectors go to start of IRAM */
    ASSERT(ABSOLUTE(.) % 0x100 == 0, "vector address must be 256 byte aligned");
    KEEP(*(.exception_vectors_table.text));
    KEEP(*(.exception_vectors.text));
    /* Code marked as running out of IRAM */
    _iram_text_start = ABSOLUTE(.);
    *(.iram1 .iram1.*)
    *libapp_trace.a:app_trace.*(.literal .literal.* .text .text.*)
    *libapp_trace.a:app_trace_util.*(.literal .literal.* .text .text.*)
    *libapp_trace.a:port_uart.*(.literal .literal.* .text .text.*)
    *libclang_rt.builtins.a:_divsf3.*(.literal .literal.* .text .text.*)
    *libclang_rt.builtins.a:restore.*(.literal .literal.* .text .text.*)
    *libclang_rt.builtins.a:save.*(.literal .literal.* .text .text.*)
    *libdriver.a:twai.*(.literal.twai_alert_handler .text.twai_alert_handler)
    *libdriver.a:twai.*(.literal.twai_handle_rx_buffer_frames .text.twai_handle_rx_buffer_frames)
    *libdriver.a:twai.*(.literal.twai_handle_tx_buffer_frame .text.twai_handle_tx_buffer_frame)
    *libdriver.a:twai.*(.literal.twai_intr_handler_main .text.twai_intr_handler_main)
    *libesp_adc.a:adc_oneshot.*(.literal.adc_oneshot_read_isr .text.adc_oneshot_read_isr)
    *libesp_driver_gpio.a:gpio.*(.literal.gpio_get_level .text.gpio_get_level)
    *libesp_driver_gpio.a:gpio.*(.literal.gpio_intr_disable .text.gpio_intr_disable)
    *libesp_driver_gpio.a:gpio.*(.literal.gpio_set_level .text.gpio_set_level)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_default_isr .text.gptimer_default_isr)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_get_captured_count .text.gptimer_get_captured_count)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_get_raw_count .text.gptimer_get_raw_count)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_set_alarm_action .text.gptimer_set_alarm_action)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_set_raw_count .text.gptimer_set_raw_count)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_start .text.gptimer_start)
    *libesp_driver_gptimer.a:gptimer.*(.literal.gptimer_stop .text.gptimer_stop)
    *libesp_driver_i2c.a:i2c_master.*(.literal.i2c_master_isr_handler_default .text.i2c_master_isr_handler_default)
    *libesp_driver_ledc.a:ledc.*(.literal._ledc_update_duty .text._ledc_update_duty)
    *libesp_driver_ledc.a:ledc.*(.literal.ledc_stop .text.ledc_stop)
    *libesp_driver_ledc.a:ledc.*(.literal.ledc_update_duty .text.ledc_update_duty)
    *libesp_driver_rmt.a:rmt_encoder.*(.literal.rmt_encoder_reset .text.rmt_encoder_reset)
    *libesp_driver_rmt.a:rmt_rx.*(.literal.rmt_isr_handle_rx_done .text.rmt_isr_handle_rx_done)
    *libesp_driver_rmt.a:rmt_rx.*(.literal.rmt_isr_handle_rx_threshold .text.rmt_isr_handle_rx_threshold)
    *libesp_driver_rmt.a:rmt_rx.*(.literal.rmt_receive .text.rmt_receive)
    *libesp_driver_rmt.a:rmt_rx.*(.literal.rmt_rx_default_isr .text.rmt_rx_default_isr)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_encode_check_result .text.rmt_encode_check_result)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_isr_handle_tx_done .text.rmt_isr_handle_tx_done)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_isr_handle_tx_loop_end .text.rmt_isr_handle_tx_loop_end)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_isr_handle_tx_threshold .text.rmt_isr_handle_tx_threshold)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_tx_default_isr .text.rmt_tx_default_isr)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_tx_do_transaction .text.rmt_tx_do_transaction)
    *libesp_driver_rmt.a:rmt_tx.*(.literal.rmt_tx_mark_eof .text.rmt_tx_mark_eof)
    *libesp_driver_sdm.a:sdm.*(.literal.sdm_channel_set_pulse_density .text.sdm_channel_set_pulse_density)
    *libesp_driver_twai.a:esp_twai.*(.literal.twai_node_receive_from_isr .text.twai_node_receive_from_isr)
    *libesp_driver_twai.a:esp_twai.*(.literal.twai_node_transmit .text.twai_node_transmit)
    *libesp_driver_twai.a:esp_twai_onchip.*(.literal._node_isr_main .text._node_isr_main)
    *libesp_driver_twai.a:esp_twai_onchip.*(.literal._node_parse_rx .text._node_parse_rx)
    *libesp_driver_twai.a:esp_twai_onchip.*(.literal._node_queue_tx .text._node_queue_tx)
    *libesp_driver_twai.a:esp_twai_onchip.*(.literal._node_start_trans .text._node_start_trans)
    *libesp_driver_uart.a:uart_vfs.*(.literal.select_notif_callback_isr .text.select_notif_callback_isr)
    *libesp_driver_usb_serial_jtag.a:usb_serial_jtag_vfs.*(.literal.select_notif_callback_isr .text.select_notif_callback_isr)
    *libesp_event.a:default_event_loop.*(.literal.esp_event_isr_post .text.esp_event_isr_post)
    *libesp_event.a:esp_event.*(.literal.esp_event_isr_post_to .text.esp_event_isr_post_to)
    *libesp_hw_support.a:clk_utils.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_compare_and_set .text.esp_cpu_compare_and_set)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_reset .text.esp_cpu_reset)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_stall .text.esp_cpu_stall)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_unstall .text.esp_cpu_unstall)
    *libesp_hw_support.a:cpu.*(.literal.esp_cpu_wait_for_intr .text.esp_cpu_wait_for_intr)
    *libesp_hw_support.a:esp_clk_tree.*(.literal.esp_clk_tree_enable_src .text.esp_clk_tree_enable_src)
    *libesp_hw_support.a:esp_memory_utils.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:gdma.*(.literal.gdma_append .text.gdma_append)
    *libesp_hw_support.a:gdma.*(.literal.gdma_default_rx_isr .text.gdma_default_rx_isr)
    *libesp_hw_support.a:gdma.*(.literal.gdma_default_tx_isr .text.gdma_default_tx_isr)
    *libesp_hw_support.a:gdma.*(.literal.gdma_reset .text.gdma_reset)
    *libesp_hw_support.a:gdma.*(.literal.gdma_start .text.gdma_start)
    *libesp_hw_support.a:gdma.*(.literal.gdma_stop .text.gdma_stop)
    *libesp_hw_support.a:mspi_timing_tuning.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_module_reset .text.periph_module_reset)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_rcc_acquire_enter .text.periph_rcc_acquire_enter)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_rcc_acquire_exit .text.periph_rcc_acquire_exit)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_rcc_enter .text.periph_rcc_enter)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_rcc_exit .text.periph_rcc_exit)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_rcc_release_enter .text.periph_rcc_release_enter)
    *libesp_hw_support.a:periph_ctrl.*(.literal.periph_rcc_release_exit .text.periph_rcc_release_exit)
    *libesp_hw_support.a:periph_ctrl.*(.literal.wifi_module_disable .text.wifi_module_disable)
    *libesp_hw_support.a:periph_ctrl.*(.literal.wifi_module_enable .text.wifi_module_enable)
    *libesp_hw_support.a:regi2c_ctrl.*(.literal.regi2c_ctrl_read_reg .text.regi2c_ctrl_read_reg)
    *libesp_hw_support.a:regi2c_ctrl.*(.literal.regi2c_ctrl_read_reg_mask .text.regi2c_ctrl_read_reg_mask)
    *libesp_hw_support.a:regi2c_ctrl.*(.literal.regi2c_ctrl_write_reg .text.regi2c_ctrl_write_reg)
    *libesp_hw_support.a:regi2c_ctrl.*(.literal.regi2c_ctrl_write_reg_mask .text.regi2c_ctrl_write_reg_mask)
    *libesp_hw_support.a:regi2c_ctrl.*(.literal.regi2c_enter_critical .text.regi2c_enter_critical)
    *libesp_hw_support.a:regi2c_ctrl.*(.literal.regi2c_exit_critical .text.regi2c_exit_critical)
    *libesp_hw_support.a:rtc_clk.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:rtc_sleep.*(.literal.rtc_sleep_pu .text.rtc_sleep_pu)
    *libesp_hw_support.a:rtc_sleep.*(.literal.rtc_sleep_start .text.rtc_sleep_start)
    *libesp_hw_support.a:rtc_time.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:sar_periph_ctrl.*(.literal .literal.* .text .text.*)
    *libesp_hw_support.a:systimer.*(.literal .literal.* .text .text.*)
    *libesp_mm.a:esp_cache_msync.*(.literal .literal.* .text .text.*)
    *libesp_mm.a:esp_cache_utils.*(.literal .literal.* .text .text.*)
    *libesp_ringbuf.a:(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_print.*(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_spiflash.*(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_sys.*(.literal .literal.* .text .text.*)
    *libesp_rom.a:esp_rom_systimer.*(.literal .literal.* .text .text.*)
    *libesp_system.a:esp_err.*(.literal .literal.* .text .text.*)
    *libesp_system.a:esp_system_chip.*(.literal.esp_system_abort .text.esp_system_abort)
    *libesp_system.a:freertos_hooks.*(.literal.esp_vApplicationTickHook .text.esp_vApplicationTickHook)
    *libesp_system.a:image_process.*(.literal .literal.* .text .text.*)
    *libesp_system.a:panic.*(.literal.panic_abort .text.panic_abort)
    *libesp_system.a:reset_reason.*(.literal.esp_reset_reason_set_hint .text.esp_reset_reason_set_hint)
    *libesp_system.a:system_internal.*(.literal.esp_restart_noos .text.esp_restart_noos)
    *libesp_system.a:system_internal.*(.literal.esp_system_reset_modules_on_exit .text.esp_system_reset_modules_on_exit)
    *libesp_system.a:system_time.*(.literal.esp_system_get_time .text.esp_system_get_time)
    *libesp_system.a:system_time.*(.literal.esp_system_get_time_resolution .text.esp_system_get_time_resolution)
    *libesp_system.a:ubsan.*(.literal .literal.* .text .text.*)
    *libesp_wifi.a:esp_adapter.*(.literal.coex_pti_get_wrapper .text.coex_pti_get_wrapper)
    *libesp_wifi.a:wifi_netif.*(.literal.wifi_sta_receive .text.wifi_sta_receive)
    *libesp_wifi.a:wifi_netif.*(.literal.wifi_transmit_wrap .text.wifi_transmit_wrap)
    *libfreertos.a:(EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:idf_additions.* *libfreertos.a:idf_additions_event_groups.* *libfreertos.a:tasks.*) .literal EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:idf_additions.* *libfreertos.a:idf_additions_event_groups.* *libfreertos.a:tasks.*) .literal.* EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:idf_additions.* *libfreertos.a:idf_additions_event_groups.* *libfreertos.a:tasks.*) .text EXCLUDE_FILE(*libfreertos.a:app_startup.* *libfreertos.a:freertos_compatibility.* *libfreertos.a:idf_additions.* *libfreertos.a:idf_additions_event_groups.* *libfreertos.a:tasks.*) .text.*)
    *libfreertos.a:tasks.*(.text .text.__getreent .text.eTaskGetState .text.pcTaskGetName .text.prvAddCurrentTaskToDelayedList .text.prvAddNewTaskToReadyList .text.prvCheckTasksWaitingTermination .text.prvCreateIdleTasks .text.prvDeleteTCB .text.prvIdleTask .text.prvInitialiseNewTask .text.prvInitialiseTaskLists .text.prvResetNextTaskUnblockTime .text.prvSearchForNameWithinSingleList .text.prvTaskCheckFreeStackSpace .text.prvTaskIsTaskSuspended .text.prvTaskPriorityRaise .text.prvTaskPriorityRestore .text.pvTaskGetCurrentTCBForCore .text.pvTaskGetThreadLocalStoragePointer .text.pvTaskIncrementMutexHeldCount .text.pxTaskGetStackStart .text.ulTaskGenericNotifyTake .text.ulTaskGenericNotifyValueClear .text.uxTaskGetNumberOfTasks .text.uxTaskGetStackHighWaterMark .text.uxTaskGetStackHighWaterMark2 .text.uxTaskPriorityGet .text.uxTaskPriorityGetFromISR .text.uxTaskResetEventItemValue .text.vTaskDelay .text.vTaskDelete .text.vTaskEndScheduler .text.vTaskGenericNotifyGiveFromISR .text.vTaskGetSnapshot .text.vTaskInternalSetTimeOutState .text.vTaskMissedYield .text.vTaskPlaceOnEventList .text.vTaskPlaceOnEventListRestricted .text.vTaskPlaceOnUnorderedEventList .text.vTaskPriorityDisinheritAfterTimeout .text.vTaskPrioritySet .text.vTaskRemoveFromUnorderedEventList .text.vTaskResume .text.vTaskSetThreadLocalStoragePointer .text.vTaskSetThreadLocalStoragePointerAndDelCallback .text.vTaskSetTimeOutState .text.vTaskStartScheduler .text.vTaskSuspend .text.vTaskSuspendAll .text.vTaskSwitchContext .text.xTaskAbortDelay .text.xTaskCatchUpTicks .text.xTaskCheckForTimeOut .text.xTaskCreatePinnedToCore .text.xTaskCreateStaticPinnedToCore .text.xTaskDelayUntil .text.xTaskGenericNotify .text.xTaskGenericNotifyFromISR .text.xTaskGenericNotifyStateClear .text.xTaskGenericNotifyWait .text.xTaskGetCoreID .text.xTaskGetCurrentTaskHandle .text.xTaskGetCurrentTaskHandleForCore .text.xTaskGetHandle .text.xTaskGetIdleTaskHandle .text.xTaskGetIdleTaskHandleForCore .text.xTaskGetSchedulerState .text.xTaskGetStaticBuffers .text.xTaskGetTickCount .text.xTaskGetTickCountFromISR .text.xTaskIncrementTick .text.xTaskPriorityDisinherit .text.xTaskPriorityInherit .text.xTaskRemoveFromEventList .text.xTaskResumeAll .text.xTaskResumeFromISR .text.xTimerCreateTimerTask)
    *libgcc.a:_divsf3.*(.literal .literal.* .text .text.*)
    *libgcc.a:lib2funcs.*(.literal .literal.* .text .text.*)
    *libgcc.a:save-restore.*(.literal .literal.* .text .text.*)
    *libgcov.a:(.literal .literal.* .text .text.*)
    *libhal.a:adc_hal_common.*(.literal.adc_hal_arbiter_config .text.adc_hal_arbiter_config)
    *libhal.a:adc_hal_common.*(.literal.adc_hal_calibration_init .text.adc_hal_calibration_init)
    *libhal.a:adc_hal_common.*(.literal.adc_hal_set_calibration_param .text.adc_hal_set_calibration_param)
    *libhal.a:adc_hal_common.*(.literal.adc_hal_set_controller .text.adc_hal_set_controller)
    *libhal.a:adc_hal_common.*(.literal.get_controller .text.get_controller)
    *libhal.a:adc_oneshot_hal.*(.literal .literal.* .text .text.*)
    *libhal.a:cache_hal.*(.literal .literal.* .text .text.*)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_append .text.gdma_ahb_hal_append)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_clear_intr .text.gdma_ahb_hal_clear_intr)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_get_eof_desc_addr .text.gdma_ahb_hal_get_eof_desc_addr)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_read_intr_status .text.gdma_ahb_hal_read_intr_status)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_reset .text.gdma_ahb_hal_reset)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_start_with_desc .text.gdma_ahb_hal_start_with_desc)
    *libhal.a:gdma_hal_ahb_v1.*(.literal.gdma_ahb_hal_stop .text.gdma_ahb_hal_stop)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_append .text.gdma_hal_append)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_clear_intr .text.gdma_hal_clear_intr)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_get_eof_desc_addr .text.gdma_hal_get_eof_desc_addr)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_read_intr_status .text.gdma_hal_read_intr_status)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_reset .text.gdma_hal_reset)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_start_with_desc .text.gdma_hal_start_with_desc)
    *libhal.a:gdma_hal_top.*(.literal.gdma_hal_stop .text.gdma_hal_stop)
    *libhal.a:gpio_hal.*(.literal.gpio_hal_intr_disable .text.gpio_hal_intr_disable)
    *libhal.a:gpio_hal.*(.literal.gpio_hal_isolate_in_sleep .text.gpio_hal_isolate_in_sleep)
    *libhal.a:i2c_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:ledc_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:mmu_hal.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_encrypt_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_hal_gpspi.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:spi_slave_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:systimer_hal.*(.literal .literal.* .text .text.*)
    *libhal.a:timer_hal.*(.literal.timer_hal_capture_and_get_counter_value .text.timer_hal_capture_and_get_counter_value)
    *libhal.a:timer_hal.*(.literal.timer_hal_set_counter_value .text.timer_hal_set_counter_value)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_check_state_flags .text.twai_hal_check_state_flags)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_decode_interrupt .text.twai_hal_decode_interrupt)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_format_frame .text.twai_hal_format_frame)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_get_events .text.twai_hal_get_events)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_get_rx_msg_count .text.twai_hal_get_rx_msg_count)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_parse_frame .text.twai_hal_parse_frame)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_read_rx_fifo .text.twai_hal_read_rx_fifo)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_set_tx_buffer_and_transmit .text.twai_hal_set_tx_buffer_and_transmit)
    *libhal.a:twai_hal_v1.*(.literal.twai_hal_soft_filter_check_msg .text.twai_hal_soft_filter_check_msg)
    *libhal.a:uart_hal_iram.*(.literal .literal.* .text .text.*)
    *libhal.a:wdt_hal_iram.*(.literal .literal.* .text .text.*)
    *libheap.a:multi_heap.*(.literal.assert_valid_block .text.assert_valid_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_impl .text.multi_heap_aligned_alloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_impl_offs .text.multi_heap_aligned_alloc_impl_offs)
    *libheap.a:multi_heap.*(.literal.multi_heap_aligned_alloc_offs .text.multi_heap_aligned_alloc_offs)
    *libheap.a:multi_heap.*(.literal.multi_heap_free_impl .text.multi_heap_free_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_allocated_size_impl .text.multi_heap_get_allocated_size_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_block_address_impl .text.multi_heap_get_block_address_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_first_block .text.multi_heap_get_first_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_full_block_size .text.multi_heap_get_full_block_size)
    *libheap.a:multi_heap.*(.literal.multi_heap_get_next_block .text.multi_heap_get_next_block)
    *libheap.a:multi_heap.*(.literal.multi_heap_internal_lock .text.multi_heap_internal_lock)
    *libheap.a:multi_heap.*(.literal.multi_heap_internal_unlock .text.multi_heap_internal_unlock)
    *libheap.a:multi_heap.*(.literal.multi_heap_is_free .text.multi_heap_is_free)
    *libheap.a:multi_heap.*(.literal.multi_heap_malloc_impl .text.multi_heap_malloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_realloc_impl .text.multi_heap_realloc_impl)
    *libheap.a:multi_heap.*(.literal.multi_heap_set_lock .text.multi_heap_set_lock)
    *libheap.a:tlsf.*(.literal.tlsf_alloc_overhead .text.tlsf_alloc_overhead)
    *libheap.a:tlsf.*(.literal.tlsf_block_size .text.tlsf_block_size)
    *libheap.a:tlsf.*(.literal.tlsf_free .text.tlsf_free)
    *libheap.a:tlsf.*(.literal.tlsf_get_pool .text.tlsf_get_pool)
    *libheap.a:tlsf.*(.literal.tlsf_malloc .text.tlsf_malloc)
    *libheap.a:tlsf.*(.literal.tlsf_memalign .text.tlsf_memalign)
    *libheap.a:tlsf.*(.literal.tlsf_memalign_offs .text.tlsf_memalign_offs)
    *libheap.a:tlsf.*(.literal.tlsf_realloc .text.tlsf_realloc)
    *libheap.a:tlsf.*(.literal.tlsf_size .text.tlsf_size)
    *liblog.a:log.*(.literal .literal.* .text .text.*)
    *liblog.a:log_format_text.*(.literal .literal.* .text .text.*)
    *liblog.a:log_lock.*(.literal .literal.* .text .text.*)
    *liblog.a:log_print.*(.literal .literal.* .text .text.*)
    *liblog.a:log_timestamp.*(.literal.esp_log_early_timestamp .text.esp_log_early_timestamp)
    *liblog.a:log_timestamp.*(.literal.esp_log_timestamp .text.esp_log_timestamp)
    *liblog.a:log_timestamp_common.*(.literal .literal.* .text .text.*)
    *liblog.a:log_write.*(.literal.esp_log_write .text.esp_log_write)
    *liblog.a:log_write.*(.literal.esp_log_writev .text.esp_log_writev)
    *liblog.a:tag_log_level.*(.literal.esp_log_level_get_timeout .text.esp_log_level_get_timeout)
    *liblog.a:util.*(.literal .literal.* .text .text.*)
    *liblwip.a:vfs_lwip.*(.literal.lwip_stop_socket_select_isr .text.lwip_stop_socket_select_isr)
    *libnet80211.a:(.wifi0iram .wifi0iram.*)
    *libnet80211.a:(.wifirxiram .wifirxiram.*)
    *libnet80211.a:(.wifislprxiram .wifislprxiram.*)
    *libnewlib.a:abort.*(.literal .literal.* .text .text.*)
    *libnewlib.a:assert.*(.literal .literal.* .text .text.*)
    *libnewlib.a:heap.*(.literal .literal.* .text .text.*)
    *libnewlib.a:stdatomic.*(.literal .literal.* .text .text.*)
    *libpp.a:(.wifi0iram .wifi0iram.*)
    *libpp.a:(.wifiorslpiram .wifiorslpiram.*)
    *libpp.a:(.wifirxiram .wifirxiram.*)
    *libpp.a:(.wifislprxiram .wifislprxiram.*)
    *libriscv.a:interrupt.*(.text .text._global_interrupt_handler .text.intr_get_item .text.intr_handler_get_arg)
    *libriscv.a:rv_utils.*(.literal .literal.* .text .text.*)
    *libriscv.a:vectors.*(.literal .literal.* .text .text.*)
    *librtc.a:(.literal .literal.* .text .text.*)
    *libsoc.a:lldesc.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:esp_flash_api.*(.literal.check_chip_pointer_default .text.check_chip_pointer_default)
    *libspi_flash.a:esp_flash_api.*(.literal.detect_spi_flash_chip .text.detect_spi_flash_chip)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_chip_driver_initialized .text.esp_flash_chip_driver_initialized)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_erase_chip .text.esp_flash_erase_chip)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_erase_region .text.esp_flash_erase_region)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_get_chip_write_protect .text.esp_flash_get_chip_write_protect)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_get_io_mode .text.esp_flash_get_io_mode)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_get_physical_size .text.esp_flash_get_physical_size)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_get_protected_region .text.esp_flash_get_protected_region)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_get_size .text.esp_flash_get_size)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_init .text.esp_flash_init)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_init_main .text.esp_flash_init_main)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_read .text.esp_flash_read)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_read_encrypted .text.esp_flash_read_encrypted)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_set_chip_write_protect .text.esp_flash_set_chip_write_protect)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_set_io_mode .text.esp_flash_set_io_mode)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_set_protected_region .text.esp_flash_set_protected_region)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_write .text.esp_flash_write)
    *libspi_flash.a:esp_flash_api.*(.literal.esp_flash_write_encrypted .text.esp_flash_write_encrypted)
    *libspi_flash.a:esp_flash_api.*(.literal.flash_end_flush_cache .text.flash_end_flush_cache)
    *libspi_flash.a:esp_flash_api.*(.literal.read_unique_id .text.read_unique_id)
    *libspi_flash.a:esp_flash_api.*(.literal.spiflash_end_default .text.spiflash_end_default)
    *libspi_flash.a:esp_flash_api.*(.literal.spiflash_start_core .text.spiflash_start_core)
    *libspi_flash.a:esp_flash_api.*(.literal.spiflash_start_default .text.spiflash_start_default)
    *libspi_flash.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:memspi_host_driver.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_boya.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_gd.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_generic.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_issi.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_mxic.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_th.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_chip_winbond.*(.literal .literal.* .text .text.*)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.delay_us .text.delay_us)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.get_buffer_malloc .text.get_buffer_malloc)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.main_flash_op_status .text.main_flash_op_status)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.main_flash_region_protected .text.main_flash_region_protected)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.release_buffer_malloc .text.release_buffer_malloc)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.spi23_end .text.spi23_end)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.spi23_start .text.spi23_start)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.spi_flash_os_check_yield .text.spi_flash_os_check_yield)
    *libspi_flash.a:spi_flash_os_func_app.*(.literal.spi_flash_os_yield .text.spi_flash_os_yield)
    *libspi_flash.a:spi_flash_os_func_noos.*(.literal.delay_us .text.delay_us)
    *libspi_flash.a:spi_flash_os_func_noos.*(.literal.esp_flash_app_disable_os_functions .text.esp_flash_app_disable_os_functions)
    *libspi_flash.a:spi_flash_os_func_noos.*(.literal.get_temp_buffer_not_supported .text.get_temp_buffer_not_supported)
    *libspi_flash.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libvfs.a:vfs.*(.literal.esp_vfs_select_triggered_isr .text.esp_vfs_select_triggered_isr)
  } > iram0_0_seg
  /**

   * This section is required to skip .iram0.text area because iram0_0_seg and

   * dram0_0_seg reflect the same address space on different buses.

   */
  .dram0.dummy (NOLOAD):
  {
    . = ORIGIN(dram0_0_seg) + _iram_end - _iram_start;
  } > dram0_0_seg
  .dram0.data :
  {
    _data_start = ABSOLUTE(.);
    *(.gnu.linkonce.d.*)
    *(.data1)
    __global_pointer$ = . + 0x800;
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.gnu.linkonce.s2.*)
    *(.jcr)
    *(.data .data.*)
    *(.dram1 .dram1.*)
    _coredump_dram_start = ABSOLUTE(.);
    *(.dram2.coredump .dram2.coredump.*)
    _coredump_dram_end = ABSOLUTE(.);
    *libapp_trace.a:app_trace.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libapp_trace.a:app_trace_util.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libapp_trace.a:port_uart.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libclang_rt.builtins.a:_divsf3.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libclang_rt.builtins.a:restore.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libclang_rt.builtins.a:save.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libdriver.a:twai.*(.rodata.twai_handle_tx_buffer_frame .rodata.twai_handle_tx_buffer_frame.str1.4 .sdata2.twai_handle_tx_buffer_frame .srodata.twai_handle_tx_buffer_frame)
    *libesp_driver_rmt.a:rmt_rx.*(.rodata.rmt_receive .rodata.rmt_receive.str1.4 .sdata2.rmt_receive .srodata.rmt_receive)
    *libesp_driver_rmt.a:rmt_tx.*(.rodata.rmt_isr_handle_tx_done .rodata.rmt_isr_handle_tx_done.str1.4 .sdata2.rmt_isr_handle_tx_done .srodata.rmt_isr_handle_tx_done)
    *libesp_driver_rmt.a:rmt_tx.*(.rodata.rmt_tx_do_transaction .rodata.rmt_tx_do_transaction.str1.4 .sdata2.rmt_tx_do_transaction .srodata.rmt_tx_do_transaction)
    *libesp_driver_twai.a:esp_twai.*(.rodata.twai_node_receive_from_isr .rodata.twai_node_receive_from_isr.str1.4 .sdata2.twai_node_receive_from_isr .srodata.twai_node_receive_from_isr)
    *libesp_driver_twai.a:esp_twai.*(.rodata.twai_node_transmit .rodata.twai_node_transmit.str1.4 .sdata2.twai_node_transmit .srodata.twai_node_transmit)
    *libesp_driver_twai.a:esp_twai_onchip.*(.rodata._node_isr_main .rodata._node_isr_main.str1.4 .sdata2._node_isr_main .srodata._node_isr_main)
    *libesp_driver_twai.a:esp_twai_onchip.*(.rodata._node_parse_rx .rodata._node_parse_rx.str1.4 .sdata2._node_parse_rx .srodata._node_parse_rx)
    *libesp_driver_twai.a:esp_twai_onchip.*(.rodata._node_queue_tx .rodata._node_queue_tx.str1.4 .sdata2._node_queue_tx .srodata._node_queue_tx)
    *libesp_hw_support.a:clk_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:cpu.*(.rodata.esp_cpu_reset .rodata.esp_cpu_reset.str1.4 .sdata2.esp_cpu_reset .srodata.esp_cpu_reset)
    *libesp_hw_support.a:esp_memory_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:gdma.*(.rodata.gdma_start .rodata.gdma_start.str1.4 .sdata2.gdma_start .srodata.gdma_start)
    *libesp_hw_support.a:mspi_timing_tuning.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:rtc_clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:sar_periph_ctrl.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_hw_support.a:systimer.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_mm.a:esp_cache_msync.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_mm.a:esp_cache_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_print.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_spiflash.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_sys.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_rom.a:esp_rom_systimer.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:esp_err.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:image_process.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libesp_system.a:reset_reason.*(.rodata.esp_reset_reason_set_hint .rodata.esp_reset_reason_set_hint.str1.4 .sdata2.esp_reset_reason_set_hint .srodata.esp_reset_reason_set_hint)
    *libesp_system.a:ubsan.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libgcc.a:_divsf3.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libgcc.a:save-restore.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libgcov.a:(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:adc_oneshot_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:cache_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:i2c_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:ledc_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:mmu_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_hal_gpspi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_flash_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:spi_slave_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:systimer_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:twai_hal_v1.*(.rodata.twai_hal_format_frame .rodata.twai_hal_format_frame.str1.4 .sdata2.twai_hal_format_frame .srodata.twai_hal_format_frame)
    *libhal.a:uart_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libhal.a:wdt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libheap.a:multi_heap.*(.rodata.assert_valid_block .rodata.assert_valid_block.str1.4 .sdata2.assert_valid_block .srodata.assert_valid_block)
    *libheap.a:multi_heap.*(.rodata.multi_heap_get_first_block .rodata.multi_heap_get_first_block.str1.4 .sdata2.multi_heap_get_first_block .srodata.multi_heap_get_first_block)
    *libheap.a:multi_heap.*(.rodata.multi_heap_get_next_block .rodata.multi_heap_get_next_block.str1.4 .sdata2.multi_heap_get_next_block .srodata.multi_heap_get_next_block)
    *libheap.a:tlsf.*(.rodata.tlsf_free .rodata.tlsf_free.str1.4 .sdata2.tlsf_free .srodata.tlsf_free)
    *libheap.a:tlsf.*(.rodata.tlsf_malloc .rodata.tlsf_malloc.str1.4 .sdata2.tlsf_malloc .srodata.tlsf_malloc)
    *libheap.a:tlsf.*(.rodata.tlsf_memalign_offs .rodata.tlsf_memalign_offs.str1.4 .sdata2.tlsf_memalign_offs .srodata.tlsf_memalign_offs)
    *libheap.a:tlsf.*(.rodata.tlsf_realloc .rodata.tlsf_realloc.str1.4 .sdata2.tlsf_realloc .srodata.tlsf_realloc)
    *liblog.a:log.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *liblog.a:log_format_text.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *liblog.a:log_lock.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *liblog.a:log_print.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *liblog.a:log_timestamp_common.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *liblog.a:util.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:abort.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:assert.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:heap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libnewlib.a:stdatomic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libphy.a:(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libsoc.a:lldesc.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:flash_brownout_hook.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:memspi_host_driver.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_boya.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_gd.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_generic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_issi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_mxic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_th.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_chip_winbond.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libspi_flash.a:spi_flash_os_func_app.*(.rodata.spi_flash_os_check_yield .rodata.spi_flash_os_check_yield.str1.4 .sdata2.spi_flash_os_check_yield .srodata.spi_flash_os_check_yield)
    *libspi_flash.a:spi_flash_wrap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    _data_end = ABSOLUTE(.);
  } > dram0_0_seg
  /**

   * This section holds data that should not be initialized at power up.

   * The section located in Internal SRAM memory region. The macro _NOINIT

   * can be used as attribute to place data into this section.

   * See the "esp_attr.h" file for more information.

   */
  .noinit (NOLOAD):
  {
    
 . = ALIGN(4); 
 _noinit_start = ABSOLUTE(.);
    *(.noinit .noinit.*)
    
 . = ALIGN(4); 
 _noinit_end = ABSOLUTE(.);
  } > dram0_0_seg
  /* Shared RAM */
  .dram0.bss (NOLOAD) :
  {
    
 . = ALIGN(8); 
 _bss_start = ABSOLUTE(.);
    /**

     * ldgen places all bss-related data to mapping[dram0_bss]

     * (See components/esp_system/app.lf).

     */
    *(.bss .bss.*)
    *(.dynbss .dynsbss .gnu.linkonce.b .gnu.linkonce.b.* .gnu.linkonce.sb .gnu.linkonce.sb.* .gnu.linkonce.sb2 .gnu.linkonce.sb2.* .sbss .sbss.* .sbss2 .sbss2.* .scommon .share.mem)
    *(.ext_ram.bss .ext_ram.bss.*)
    *(COMMON)
    
 . = ALIGN(8); 
 _bss_end = ABSOLUTE(.);
  } > dram0_0_seg
  ASSERT(((_bss_end - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
         "DRAM segment data does not fit.")
  .flash.text :
  {
    _stext = .;
    /**

     * Mark the start of flash.text.

     * This can be used by the MMU driver to maintain the virtual address.

     */
    _instruction_reserved_start = ABSOLUTE(.);
    _text_start = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_adc.a:adc_oneshot.* *libesp_driver_gpio.a:gpio.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_i2c.a:i2c_master.* *libesp_driver_ledc.a:ledc.* *libesp_driver_rmt.a:rmt_encoder.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_sdm.a:sdm.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_driver_uart.a:uart_vfs.* *libesp_driver_usb_serial_jtag.a:usb_serial_jtag_vfs.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_clk_tree.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:regi2c_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:freertos_hooks.* *libesp_system.a:image_process.* *libesp_system.a:panic.* *libesp_system.a:reset_reason.* *libesp_system.a:system_internal.* *libesp_system.a:system_time.* *libesp_system.a:ubsan.* *libesp_wifi.a:esp_adapter.* *libesp_wifi.a:wifi_netif.* *libgcc.a:_divsf3.* *libgcc.a:lib2funcs.* *libgcc.a:save-restore.* *libhal.a:adc_hal_common.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:gdma_hal_ahb_v1.* *libhal.a:gdma_hal_top.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:timer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp.* *liblog.a:log_timestamp_common.* *liblog.a:log_write.* *liblog.a:tag_log_level.* *liblog.a:util.* *liblwip.a:vfs_lwip.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libriscv.a:interrupt.* *libriscv.a:rv_utils.* *libriscv.a:vectors.* *libsoc.a:lldesc.* *libspi_flash.a:esp_flash_api.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_os_func_noos.* *libspi_flash.a:spi_flash_wrap.* *libvfs.a:vfs.*) .literal EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_adc.a:adc_oneshot.* *libesp_driver_gpio.a:gpio.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_i2c.a:i2c_master.* *libesp_driver_ledc.a:ledc.* *libesp_driver_rmt.a:rmt_encoder.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_sdm.a:sdm.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_driver_uart.a:uart_vfs.* *libesp_driver_usb_serial_jtag.a:usb_serial_jtag_vfs.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_clk_tree.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:regi2c_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:freertos_hooks.* *libesp_system.a:image_process.* *libesp_system.a:panic.* *libesp_system.a:reset_reason.* *libesp_system.a:system_internal.* *libesp_system.a:system_time.* *libesp_system.a:ubsan.* *libesp_wifi.a:esp_adapter.* *libesp_wifi.a:wifi_netif.* *libgcc.a:_divsf3.* *libgcc.a:lib2funcs.* *libgcc.a:save-restore.* *libhal.a:adc_hal_common.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:gdma_hal_ahb_v1.* *libhal.a:gdma_hal_top.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:timer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp.* *liblog.a:log_timestamp_common.* *liblog.a:log_write.* *liblog.a:tag_log_level.* *liblog.a:util.* *liblwip.a:vfs_lwip.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libriscv.a:interrupt.* *libriscv.a:rv_utils.* *libriscv.a:vectors.* *libsoc.a:lldesc.* *libspi_flash.a:esp_flash_api.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_os_func_noos.* *libspi_flash.a:spi_flash_wrap.* *libvfs.a:vfs.*) .literal.* EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_adc.a:adc_oneshot.* *libesp_driver_gpio.a:gpio.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_i2c.a:i2c_master.* *libesp_driver_ledc.a:ledc.* *libesp_driver_rmt.a:rmt_encoder.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_sdm.a:sdm.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_driver_uart.a:uart_vfs.* *libesp_driver_usb_serial_jtag.a:usb_serial_jtag_vfs.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_clk_tree.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:regi2c_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:freertos_hooks.* *libesp_system.a:image_process.* *libesp_system.a:panic.* *libesp_system.a:reset_reason.* *libesp_system.a:system_internal.* *libesp_system.a:system_time.* *libesp_system.a:ubsan.* *libesp_wifi.a:esp_adapter.* *libesp_wifi.a:wifi_netif.* *libgcc.a:_divsf3.* *libgcc.a:lib2funcs.* *libgcc.a:save-restore.* *libhal.a:adc_hal_common.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:gdma_hal_ahb_v1.* *libhal.a:gdma_hal_top.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:timer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp.* *liblog.a:log_timestamp_common.* *liblog.a:log_write.* *liblog.a:tag_log_level.* *liblog.a:util.* *liblwip.a:vfs_lwip.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libriscv.a:interrupt.* *libriscv.a:rv_utils.* *libriscv.a:vectors.* *libsoc.a:lldesc.* *libspi_flash.a:esp_flash_api.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_os_func_noos.* *libspi_flash.a:spi_flash_wrap.* *libvfs.a:vfs.*) .text EXCLUDE_FILE(*libesp_ringbuf.a *libfreertos.a *libgcov.a *librtc.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_adc.a:adc_oneshot.* *libesp_driver_gpio.a:gpio.* *libesp_driver_gptimer.a:gptimer.* *libesp_driver_i2c.a:i2c_master.* *libesp_driver_ledc.a:ledc.* *libesp_driver_rmt.a:rmt_encoder.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_sdm.a:sdm.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_driver_uart.a:uart_vfs.* *libesp_driver_usb_serial_jtag.a:usb_serial_jtag_vfs.* *libesp_event.a:default_event_loop.* *libesp_event.a:esp_event.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_clk_tree.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:periph_ctrl.* *libesp_hw_support.a:regi2c_ctrl.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:rtc_sleep.* *libesp_hw_support.a:rtc_time.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:esp_system_chip.* *libesp_system.a:freertos_hooks.* *libesp_system.a:image_process.* *libesp_system.a:panic.* *libesp_system.a:reset_reason.* *libesp_system.a:system_internal.* *libesp_system.a:system_time.* *libesp_system.a:ubsan.* *libesp_wifi.a:esp_adapter.* *libesp_wifi.a:wifi_netif.* *libgcc.a:_divsf3.* *libgcc.a:lib2funcs.* *libgcc.a:save-restore.* *libhal.a:adc_hal_common.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:gdma_hal_ahb_v1.* *libhal.a:gdma_hal_top.* *libhal.a:gpio_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:timer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp.* *liblog.a:log_timestamp_common.* *liblog.a:log_write.* *liblog.a:tag_log_level.* *liblog.a:util.* *liblwip.a:vfs_lwip.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libriscv.a:interrupt.* *libriscv.a:rv_utils.* *libriscv.a:vectors.* *libsoc.a:lldesc.* *libspi_flash.a:esp_flash_api.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_os_func_noos.* *libspi_flash.a:spi_flash_wrap.* *libvfs.a:vfs.*) .text.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifi0iram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifi0iram.*)
    *(.wifiextrairam .wifiextrairam.*)
    *(EXCLUDE_FILE(*libpp.a) .wifiorslpiram EXCLUDE_FILE(*libpp.a) .wifiorslpiram.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifirxiram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifislprxiram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifislprxiram.*)
    *libdriver.a:twai.*(.text .text.twai_alloc_driver_obj .text.twai_clear_receive_queue .text.twai_clear_receive_queue_v2 .text.twai_clear_transmit_queue .text.twai_clear_transmit_queue_v2 .text.twai_configure_gpio .text.twai_driver_install .text.twai_driver_install_v2 .text.twai_driver_uninstall .text.twai_driver_uninstall_v2 .text.twai_free_driver_obj .text.twai_get_status_info .text.twai_get_status_info_v2 .text.twai_initiate_recovery .text.twai_initiate_recovery_v2 .text.twai_read_alerts .text.twai_read_alerts_v2 .text.twai_receive .text.twai_receive_v2 .text.twai_reconfigure_alerts .text.twai_reconfigure_alerts_v2 .text.twai_release_gpio .text.twai_start .text.twai_start_v2 .text.twai_stop .text.twai_stop_v2 .text.twai_transmit .text.twai_transmit_v2)
    *libesp_adc.a:adc_oneshot.*(.text .text.adc_oneshot_channel_to_io .text.adc_oneshot_config_channel .text.adc_oneshot_del_unit .text.adc_oneshot_get_calibrated_result .text.adc_oneshot_io_to_channel .text.adc_oneshot_new_unit .text.adc_oneshot_read .text.s_adc_io_init .text.s_adc_unit_claim .text.s_adc_unit_free)
    *libesp_driver_gpio.a:gpio.*(.text .text.gpio_config .text.gpio_config_as_analog .text.gpio_deep_sleep_hold_dis .text.gpio_deep_sleep_hold_en .text.gpio_deep_sleep_wakeup_disable .text.gpio_deep_sleep_wakeup_enable .text.gpio_dump_io_configuration .text.gpio_func_sel .text.gpio_get_drive_capability .text.gpio_get_io_config .text.gpio_hold_dis .text.gpio_hold_en .text.gpio_input_disable .text.gpio_input_enable .text.gpio_install_isr_service .text.gpio_intr_enable .text.gpio_iomux_in .text.gpio_iomux_input .text.gpio_iomux_out .text.gpio_iomux_output .text.gpio_isr_handler_add .text.gpio_isr_handler_remove .text.gpio_isr_register .text.gpio_isr_register_on_core_static .text.gpio_ll_deepsleep_wakeup_disable .text.gpio_ll_deepsleep_wakeup_enable .text.gpio_ll_force_hold_all .text.gpio_ll_force_unhold_all .text.gpio_ll_get_drive_capability .text.gpio_ll_get_io_config .text.gpio_ll_hold_dis .text.gpio_ll_hold_en .text.gpio_ll_set_drive_capability .text.gpio_od_disable .text.gpio_od_enable .text.gpio_output_disable .text.gpio_output_enable .text.gpio_pulldown_dis .text.gpio_pulldown_en .text.gpio_pullup_dis .text.gpio_pullup_en .text.gpio_reset_pin .text.gpio_set_direction .text.gpio_set_drive_capability .text.gpio_set_intr_type .text.gpio_set_pull_mode .text.gpio_sleep_input_disable .text.gpio_sleep_input_enable .text.gpio_sleep_output_disable .text.gpio_sleep_output_enable .text.gpio_sleep_pulldown_dis .text.gpio_sleep_pulldown_en .text.gpio_sleep_pullup_dis .text.gpio_sleep_pullup_en .text.gpio_sleep_sel_dis .text.gpio_sleep_sel_en .text.gpio_sleep_set_direction .text.gpio_sleep_set_pull_mode .text.gpio_uninstall_isr_service .text.gpio_wakeup_disable .text.gpio_wakeup_enable)
    *libesp_driver_gptimer.a:gptimer.*(.text .text.gptimer_del_timer .text.gptimer_destroy .text.gptimer_disable .text.gptimer_enable .text.gptimer_get_resolution .text.gptimer_new_timer .text.gptimer_register_event_callbacks .text.gptimer_register_to_group .text.gptimer_unregister_from_group)
    *libesp_driver_i2c.a:i2c_master.*(.text .text.i2c_del_master_bus .text.i2c_master_bus_add_device .text.i2c_master_bus_destroy .text.i2c_master_bus_reset .text.i2c_master_bus_rm_device .text.i2c_master_bus_wait_all_done .text.i2c_master_device_change_address .text.i2c_master_execute_defined_operations .text.i2c_master_get_bus_handle .text.i2c_master_multi_buffer_transmit .text.i2c_master_probe .text.i2c_master_receive .text.i2c_master_register_event_callbacks .text.i2c_master_transmit .text.i2c_master_transmit_receive .text.i2c_new_master_bus .text.i2c_param_master_config .text.s_i2c_asynchronous_transaction .text.s_i2c_err_log_print .text.s_i2c_hw_fsm_reset .text.s_i2c_master_clear_bus .text.s_i2c_read_command .text.s_i2c_send_command_async .text.s_i2c_send_commands .text.s_i2c_start_end_command .text.s_i2c_synchronous_transaction .text.s_i2c_transaction_start .text.s_i2c_write_command)
    *libesp_driver_ledc.a:ledc.*(.text .text._ledc_fade_hw_acquire .text._ledc_fade_hw_release .text._ledc_fade_start .text._ledc_op_lock_acquire .text._ledc_op_lock_release .text._ledc_set_fade_with_step .text._ledc_set_fade_with_time .text._ledc_set_pin .text.ilog2 .text.ledc_auto_clk_divisor .text.ledc_auto_global_clk_divisor .text.ledc_bind_channel_timer .text.ledc_cb_register .text.ledc_channel_config .text.ledc_enable_intr_type .text.ledc_fade_channel_deinit .text.ledc_fade_channel_init_check .text.ledc_fade_func_install .text.ledc_fade_func_uninstall .text.ledc_fade_start .text.ledc_fade_stop .text.ledc_find_suitable_duty_resolution .text.ledc_get_duty .text.ledc_get_freq .text.ledc_get_hpoint .text.ledc_get_max_duty .text.ledc_glb_clk_set_sleep_mode .text.ledc_isr_register .text.ledc_ls_timer_update .text.ledc_set_duty .text.ledc_set_duty_and_update .text.ledc_set_duty_with_hpoint .text.ledc_set_fade .text.ledc_set_fade_step_and_start .text.ledc_set_fade_time_and_start .text.ledc_set_fade_with_step .text.ledc_set_fade_with_time .text.ledc_set_freq .text.ledc_set_pin .text.ledc_set_timer_div .text.ledc_set_timer_params .text.ledc_slow_clk_calibrate .text.ledc_speed_mode_ctx_create .text.ledc_timer_clk_src_set_xpd_in_sleep .text.ledc_timer_config .text.ledc_timer_del .text.ledc_timer_pause .text.ledc_timer_resume .text.ledc_timer_rst .text.ledc_timer_set)
    *libesp_driver_rmt.a:rmt_encoder.*(.text .text.rmt_alloc_encoder_mem .text.rmt_del_encoder)
    *libesp_driver_rmt.a:rmt_rx.*(.text .text.rmt_del_rx_channel .text.rmt_ll_rx_set_carrier_high_low_ticks .text.rmt_new_rx_channel .text.rmt_rx_demodulate_carrier .text.rmt_rx_destroy .text.rmt_rx_disable .text.rmt_rx_enable .text.rmt_rx_register_event_callbacks .text.rmt_rx_register_to_group .text.rmt_rx_unregister_from_group)
    *libesp_driver_rmt.a:rmt_tx.*(.text .text.rmt_del_sync_manager .text.rmt_del_tx_channel .text.rmt_ll_tx_set_carrier_high_low_ticks .text.rmt_new_sync_manager .text.rmt_new_tx_channel .text.rmt_sync_reset .text.rmt_transmit .text.rmt_tx_create_trans_queue .text.rmt_tx_destroy .text.rmt_tx_disable .text.rmt_tx_enable .text.rmt_tx_modulate_carrier .text.rmt_tx_register_event_callbacks .text.rmt_tx_register_to_group .text.rmt_tx_switch_gpio .text.rmt_tx_unregister_from_group .text.rmt_tx_wait_all_done)
    *libesp_driver_sdm.a:sdm.*(.text .text.sdm_acquire_group_handle .text.sdm_channel_disable .text.sdm_channel_enable .text.sdm_del_channel .text.sdm_destroy .text.sdm_ll_set_prescale .text.sdm_new_channel .text.sdm_register_to_group .text.sdm_release_group_handle .text.sdm_unregister_from_group)
    *libesp_driver_twai.a:esp_twai.*(.text .text.twai_node_config_mask_filter .text.twai_node_config_range_filter .text.twai_node_delete .text.twai_node_disable .text.twai_node_enable .text.twai_node_get_info .text.twai_node_reconfig_timing .text.twai_node_recover .text.twai_node_register_event_callbacks .text.twai_node_timing_calc_param .text.twai_node_transmit_wait_all_done)
    *libesp_driver_twai.a:esp_twai_onchip.*(.text .text._ctrlr_acquire .text._ctrlr_release .text._node_calc_set_bit_timing .text._node_check_timing_valid .text._node_config_io .text._node_config_mask_filter .text._node_delete .text._node_destroy .text._node_disable .text._node_enable .text._node_get_status .text._node_recover .text._node_register_callbacks .text._node_release_io .text._node_set_bit_timing .text._node_set_clock_source .text._node_wait_tx_all_done .text._twai_rcc_clock_ctrl .text._twai_rcc_clock_sel .text.twai_new_node_onchip)
    *libesp_driver_uart.a:uart_vfs.*(.text .text.__esp_system_init_fn_init_vfs_uart .text.esp_vfs_uart_get_vfs .text.register_select .text.uart_access .text.uart_close .text.uart_end_select .text.uart_fcntl .text.uart_fstat .text.uart_fsync .text.uart_get_avail_data_len .text.uart_get_avail_data_len_via_driver .text.uart_open .text.uart_read .text.uart_read_char .text.uart_return_char .text.uart_rx_char .text.uart_rx_char_via_driver .text.uart_start_select .text.uart_tcdrain .text.uart_tcflush .text.uart_tcgetattr .text.uart_tcsetattr .text.uart_tx_char .text.uart_tx_char_via_driver .text.uart_vfs_dev_port_set_rx_line_endings .text.uart_vfs_dev_port_set_tx_line_endings .text.uart_vfs_dev_register .text.uart_vfs_dev_set_rx_line_endings .text.uart_vfs_dev_set_tx_line_endings .text.uart_vfs_dev_use_driver .text.uart_vfs_dev_use_nonblocking .text.uart_vfs_include_dev_init .text.uart_write .text.unregister_select)
    *libesp_driver_usb_serial_jtag.a:usb_serial_jtag_vfs.*(.text .text.__esp_system_init_fn_init_vfs_usj_sec .text.esp_vfs_usb_serial_jtag_get_vfs .text.register_select .text.unregister_select .text.usb_serial_jtag_close .text.usb_serial_jtag_end_select .text.usb_serial_jtag_fcntl .text.usb_serial_jtag_fstat .text.usb_serial_jtag_fsync .text.usb_serial_jtag_open .text.usb_serial_jtag_read .text.usb_serial_jtag_read_char .text.usb_serial_jtag_return_char .text.usb_serial_jtag_rx_char_no_driver .text.usb_serial_jtag_start_select .text.usb_serial_jtag_tcdrain .text.usb_serial_jtag_tcflush .text.usb_serial_jtag_tcgetattr .text.usb_serial_jtag_tcsetattr .text.usb_serial_jtag_tx_char_no_driver .text.usb_serial_jtag_vfs_include_dev_init .text.usb_serial_jtag_vfs_register .text.usb_serial_jtag_vfs_set_rx_line_endings .text.usb_serial_jtag_vfs_set_tx_line_endings .text.usb_serial_jtag_vfs_use_driver .text.usb_serial_jtag_vfs_use_nonblocking .text.usb_serial_jtag_wait_tx_done_no_driver .text.usb_serial_jtag_write .text.usbjtag_rx_char_via_driver .text.usbjtag_tx_char_via_driver .text.usbjtag_wait_tx_done_via_driver)
    *libesp_event.a:default_event_loop.*(.text .text.esp_event_handler_instance_register .text.esp_event_handler_instance_unregister .text.esp_event_handler_register .text.esp_event_handler_unregister .text.esp_event_loop_create_default .text.esp_event_loop_delete_default .text.esp_event_post)
    *libesp_event.a:esp_event.*(.text .text.base_node_add_handler .text.base_node_remove_all_handler .text.base_node_remove_handler .text.esp_event_dump .text.esp_event_handler_instance_register_with .text.esp_event_handler_instance_unregister_with .text.esp_event_handler_register_with .text.esp_event_handler_register_with_internal .text.esp_event_handler_unregister_with .text.esp_event_handler_unregister_with_internal .text.esp_event_loop_create .text.esp_event_loop_delete .text.esp_event_loop_run .text.esp_event_loop_run_task .text.esp_event_post_to .text.find_and_unregister_handler .text.handler_execute .text.handler_instances_add .text.handler_instances_remove .text.handler_instances_remove_all .text.loop_node_add_handler .text.loop_node_remove_all_handler .text.loop_node_remove_handler .text.loop_remove_handler)
    *libesp_hw_support.a:cpu.*(.text .text.esp_cpu_clear_breakpoint .text.esp_cpu_clear_watchpoint .text.esp_cpu_set_breakpoint .text.esp_cpu_set_watchpoint)
    *libesp_hw_support.a:esp_clk_tree.*(.text .text.esp_clk_tree_src_get_freq_hz)
    *libesp_hw_support.a:gdma.*(.text .text.do_allocate_gdma_channel .text.gdma_acquire_group_handle .text.gdma_acquire_pair_handle .text.gdma_apply_strategy .text.gdma_config_transfer .text.gdma_connect .text.gdma_del_channel .text.gdma_del_rx_channel .text.gdma_del_tx_channel .text.gdma_disconnect .text.gdma_get_alignment_constraints .text.gdma_get_free_m2m_trig_id_mask .text.gdma_get_group_channel_id .text.gdma_install_rx_interrupt .text.gdma_install_tx_interrupt .text.gdma_new_ahb_channel .text.gdma_register_rx_event_callbacks .text.gdma_register_tx_event_callbacks .text.gdma_release_group_handle .text.gdma_release_pair_handle .text.gdma_set_priority)
    *libesp_hw_support.a:periph_ctrl.*(.text .text.periph_ll_disable_clk_set_rst .text.periph_ll_enable_clk_clear_rst .text.periph_ll_get_clk_en_mask .text.periph_ll_get_clk_en_reg .text.periph_ll_get_rst_en_mask .text.periph_ll_get_rst_en_reg .text.periph_ll_reset .text.periph_module_disable .text.periph_module_enable)
    *libesp_hw_support.a:regi2c_ctrl.*(.text .text.regi2c_saradc_disable .text.regi2c_saradc_enable)
    *libesp_hw_support.a:rtc_sleep.*(.text .text.rtc_deep_sleep_start .text.rtc_sleep_get_default_config .text.rtc_sleep_init .text.rtc_sleep_low_init)
    *libesp_system.a:esp_system_chip.*(.text .text.esp_get_free_heap_size .text.esp_get_free_internal_heap_size .text.esp_get_idf_version .text.esp_get_minimum_free_heap_size)
    *libesp_system.a:freertos_hooks.*(.text .text.esp_deregister_freertos_idle_hook .text.esp_deregister_freertos_idle_hook_for_cpu .text.esp_deregister_freertos_tick_hook .text.esp_deregister_freertos_tick_hook_for_cpu .text.esp_register_freertos_idle_hook .text.esp_register_freertos_idle_hook_for_cpu .text.esp_register_freertos_tick_hook .text.esp_register_freertos_tick_hook_for_cpu .text.esp_vApplicationIdleHook)
    *libesp_system.a:panic.*(.text .text.disable_all_wdts .text.esp_panic_handler .text.esp_panic_handler_disable_timg_wdts .text.esp_panic_handler_enable_rtc_wdt .text.esp_panic_handler_feed_wdts .text.esp_panic_handler_increment_entry_count .text.esp_reset_reason_get_hint .text.esp_reset_reason_set_hint .text.panic_print_char .text.panic_print_char_uart .text.panic_print_char_usb_serial_jtag .text.panic_print_dec .text.panic_print_hex .text.panic_print_str .text.print_abort_details .text.semihosting_call_noerrno)
    *libesp_system.a:reset_reason.*(.text .text.esp_reset_reason .text.esp_reset_reason_get_hint .text.esp_reset_reason_init .text.get_reset_reason)
    *libesp_system.a:system_internal.*(.text)
    *libesp_system.a:system_time.*(.text)
    *libesp_wifi.a:esp_adapter.*(.text .text.clear_intr_wrapper .text.coex_deinit_wrapper .text.coex_disable_wrapper .text.coex_enable_wrapper .text.coex_init_wrapper .text.coex_register_start_cb_wrapper .text.coex_schm_curr_period_get_wrapper .text.coex_schm_curr_phase_get_wrapper .text.coex_schm_flexible_period_get_wrapper .text.coex_schm_flexible_period_set_wrapper .text.coex_schm_get_phase_by_idx_wrapper .text.coex_schm_interval_get_wrapper .text.coex_schm_process_restart_wrapper .text.coex_schm_register_cb_wrapper .text.coex_schm_status_bit_clear_wrapper .text.coex_schm_status_bit_set_wrapper .text.coex_wifi_channel_set_wrapper .text.coex_wifi_request_wrapper .text.disable_intr_wrapper .text.enable_intr_wrapper .text.esp_event_post_wrapper .text.esp_log_write_wrapper .text.esp_log_writev_wrapper .text.esp_phy_disable_wrapper .text.esp_phy_enable_wrapper .text.esp_read_mac_wrapper .text.event_group_wait_bits_wrapper .text.get_time_wrapper .text.mutex_create_wrapper .text.mutex_delete_wrapper .text.nvs_open_wrapper .text.queue_create_wrapper .text.queue_recv_wrapper .text.queue_send_to_back_wrapper .text.queue_send_to_front_wrapper .text.queue_send_wrapper .text.recursive_mutex_create_wrapper .text.set_intr_wrapper .text.set_isr_wrapper .text.task_create_pinned_to_core_wrapper .text.task_create_wrapper .text.task_get_max_priority_wrapper .text.wifi_clock_disable_wrapper .text.wifi_clock_enable_wrapper .text.wifi_create_queue .text.wifi_create_queue_wrapper .text.wifi_delete_queue .text.wifi_delete_queue_wrapper .text.wifi_reset_mac_wrapper .text.wifi_thread_semphr_free .text.wifi_thread_semphr_get_wrapper)
    *libesp_wifi.a:wifi_netif.*(.text .text.esp_wifi_create_if_driver .text.esp_wifi_destroy_if_driver .text.esp_wifi_get_if_mac .text.esp_wifi_is_if_ready_when_started .text.esp_wifi_register_if_rxcb .text.wifi_ap_receive .text.wifi_driver_start .text.wifi_free .text.wifi_transmit)
    *libfreertos.a:app_startup.*(.literal .literal.* .text .text.*)
    *libfreertos.a:freertos_compatibility.*(.literal .literal.* .text .text.*)
    *libfreertos.a:idf_additions.*(.literal .literal.* .text .text.*)
    *libfreertos.a:idf_additions_event_groups.*(.literal .literal.* .text .text.*)
    *libfreertos.a:tasks.*(.literal.pxGetTaskListByIndex .text.pxGetTaskListByIndex)
    *libfreertos.a:tasks.*(.literal.uxTaskGetSnapshotAll .text.uxTaskGetSnapshotAll)
    *libfreertos.a:tasks.*(.literal.xTaskGetNext .text.xTaskGetNext)
    *libhal.a:adc_hal_common.*(.text .text.adc_hal_self_calibration .text.adc_ll_calibration_finish .text.adc_ll_calibration_prepare .text.cal_setup .text.read_cal_channel)
    *libhal.a:gdma_hal_ahb_v1.*(.text .text.gdma_ahb_hal_connect_peri .text.gdma_ahb_hal_disconnect_peri .text.gdma_ahb_hal_enable_burst .text.gdma_ahb_hal_enable_intr .text.gdma_ahb_hal_get_intr_status_reg .text.gdma_ahb_hal_init .text.gdma_ahb_hal_set_priority .text.gdma_ahb_hal_set_strategy .text.gdma_ll_rx_enable_interrupt .text.gdma_ll_tx_enable_interrupt)
    *libhal.a:gdma_hal_top.*(.text .text.gdma_hal_connect_peri .text.gdma_hal_deinit .text.gdma_hal_disconnect_peri .text.gdma_hal_enable_burst .text.gdma_hal_enable_intr .text.gdma_hal_get_intr_status_reg .text.gdma_hal_set_burst_size .text.gdma_hal_set_priority .text.gdma_hal_set_strategy)
    *libhal.a:gpio_hal.*(.text .text.gpio_hal_intr_enable_on_core .text.gpio_hal_iomux_in .text.gpio_hal_iomux_out)
    *libhal.a:timer_hal.*(.text .text.timer_hal_deinit .text.timer_hal_init)
    *libhal.a:twai_hal_v1.*(.text .text.twai_hal_check_brp_validation .text.twai_hal_clear_rx_fifo_overrun .text.twai_hal_configure .text.twai_hal_configure_filter .text.twai_hal_configure_timing .text.twai_hal_deinit .text.twai_hal_get_err_state .text.twai_hal_get_mem_requirment .text.twai_hal_get_rec .text.twai_hal_get_tec .text.twai_hal_init .text.twai_hal_start .text.twai_hal_start_bus_recovery .text.twai_hal_stop)
    *libheap.a:multi_heap.*(.text .text.multi_heap_check .text.multi_heap_dump .text.multi_heap_dump_tlsf .text.multi_heap_find_containing_block_impl .text.multi_heap_free_size_impl .text.multi_heap_get_info_impl .text.multi_heap_get_info_tlsf .text.multi_heap_minimum_free_size_impl .text.multi_heap_register_impl .text.multi_heap_reset_minimum_free_bytes .text.multi_heap_restore_minimum_free_bytes .text.multi_heap_walk)
    *libheap.a:tlsf.*(.text .text.control_construct .text.default_walker .text.integrity_walker .text.tlsf_add_pool .text.tlsf_check .text.tlsf_check_pool .text.tlsf_create .text.tlsf_create_with_pool .text.tlsf_destroy .text.tlsf_find_containing_block .text.tlsf_fit_size .text.tlsf_malloc_addr .text.tlsf_pool_overhead .text.tlsf_remove_pool .text.tlsf_walk_pool)
    *liblog.a:log_timestamp.*(.text)
    *liblog.a:log_write.*(.text .text.esp_log_set_vprintf)
    *liblog.a:tag_log_level.*(.text .text.esp_log_level_get .text.esp_log_level_set .text.log_level_get .text.log_level_set)
    *liblwip.a:vfs_lwip.*(.text .text.esp_vfs_lwip_sockets_register .text.lwip_fcntl_r_wrapper .text.lwip_fstat .text.lwip_get_socket_select_semaphore .text.lwip_ioctl_r_wrapper .text.lwip_stop_socket_select)
    *libriscv.a:interrupt.*(.literal.intr_handler_get .text.intr_handler_get)
    *libriscv.a:interrupt.*(.literal.intr_handler_set .text.intr_handler_set)
    *libspi_flash.a:esp_flash_api.*(.text .text.esp_flash_app_disable_protect .text.esp_flash_get_protectable_regions .text.esp_flash_read_chip_id .text.esp_flash_read_id .text.esp_flash_read_unique_chip_id .text.esp_flash_suspend_cmd_init .text.find_region)
    *libspi_flash.a:spi_flash_os_func_app.*(.text .text.esp_flash_app_enable_os_functions .text.esp_flash_deinit_os_functions .text.esp_flash_init_main_bus_lock .text.esp_flash_init_os_functions .text.esp_flash_set_dangerous_write_protection .text.use_bus_lock)
    *libspi_flash.a:spi_flash_os_func_noos.*(.text)
    *libvfs.a:vfs.*(.text .text.call_end_selects .text.esp_get_free_index .text.esp_minify_vfs .text.esp_vfs_access .text.esp_vfs_close .text.esp_vfs_closedir .text.esp_vfs_dump_fds .text.esp_vfs_dump_registered_paths .text.esp_vfs_duplicate_fs_ops .text.esp_vfs_fcntl_r .text.esp_vfs_free_entry .text.esp_vfs_free_fs_ops .text.esp_vfs_fstat .text.esp_vfs_fsync .text.esp_vfs_ftruncate .text.esp_vfs_ioctl .text.esp_vfs_link .text.esp_vfs_lseek .text.esp_vfs_make_fs_ops .text.esp_vfs_mkdir .text.esp_vfs_open .text.esp_vfs_opendir .text.esp_vfs_pread .text.esp_vfs_pwrite .text.esp_vfs_read .text.esp_vfs_readdir .text.esp_vfs_readdir_r .text.esp_vfs_register .text.esp_vfs_register_common .text.esp_vfs_register_fd .text.esp_vfs_register_fd_range .text.esp_vfs_register_fd_with_local_fd .text.esp_vfs_register_fs .text.esp_vfs_register_fs_common .text.esp_vfs_register_fs_with_id .text.esp_vfs_register_with_id .text.esp_vfs_rename .text.esp_vfs_rewinddir .text.esp_vfs_rmdir .text.esp_vfs_safe_fd_isset .text.esp_vfs_seekdir .text.esp_vfs_select .text.esp_vfs_select_triggered .text.esp_vfs_set_readonly_flag .text.esp_vfs_stat .text.esp_vfs_telldir .text.esp_vfs_truncate .text.esp_vfs_unlink .text.esp_vfs_unregister .text.esp_vfs_unregister_fd .text.esp_vfs_unregister_with_id .text.esp_vfs_utime .text.esp_vfs_write .text.free_proxy_members .text.get_vfs_for_fd .text.get_vfs_for_index .text.get_vfs_for_path .text.is_path_prefix_valid .text.set_global_fd_sets .text.tcdrain .text.tcflow .text.tcflush .text.tcgetattr .text.tcgetsid .text.tcsendbreak .text.tcsetattr .text.translate_path .text.vfs_include_syscalls_impl)
    *(.stub)
    *(.gnu.linkonce.t.*)
    *(.gnu.warning)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */
    /**

     * CPU will try to prefetch up to 16 bytes of of instructions.

     * This means that any configuration (e.g. MMU, PMS) must allow

     * safe access to up to 16 bytes after the last real instruction, add

     * dummy bytes to ensure this

     */
    . += 16;
    _text_end = ABSOLUTE(.);
    /**

     * Mark the flash.text end.

     * This can be used for MMU driver to maintain virtual address.

     */
    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;
    /**

     * Similar to _iram_start, this symbol goes here so it is

     * resolved by addr2line in preference to the first symbol in

     * the flash.text segment.

     */
    _flash_cache_start = ABSOLUTE(0);
  } > default_code_seg
  /**

   * Dummy section represents the .flash.text section but in default_rodata_seg.

   * Thus, it must have its alignment and (at least) its size.

   */
  .flash_rodata_dummy (NOLOAD):
  {
    _flash_rodata_dummy_start = .;
    . = ALIGN(ALIGNOF(.flash.text)) + SIZEOF(.flash.text);
    /* Add alignment of MMU page size + 0x20 bytes for the mapping header. */
    . = ALIGN(0x10000) + 0x20;
  } > default_rodata_seg
  .flash.appdesc : ALIGN(0x10)
  {
    /**

     * Mark flash.rodata start.

     * This can be used for mmu driver to maintain virtual address

     */
    _rodata_reserved_start = ABSOLUTE(.);
    _rodata_start = ABSOLUTE(.);
    /* !DO NOT PUT ANYTHING BEFORE THIS! */
    /* Should be the first.  App version info. */
    *(.rodata_desc .rodata_desc.*)
    /* Should be the second. Custom app version info. */
    *(.rodata_custom_desc .rodata_custom_desc.*)
    /**

     * Create an empty gap within this section. Thanks to this, the end of this

     * section will match .flash.rodata's begin address. Thus, both sections

     * will be merged when creating the final bin image.

     */
    . = ALIGN(ALIGNOF(.flash.rodata));
  } > default_rodata_seg
  ASSERT((ADDR(.flash.rodata) == ADDR(.flash.appdesc) + SIZEOF(.flash.appdesc)), "The gap between .flash.appdesc and .flash.rodata must not exist to produce the final bin image.")
  .flash.rodata : ALIGN(0x10)
  {
    _flash_rodata_start = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:reset_reason.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_wrap.*) .rodata EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:reset_reason.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_wrap.*) .rodata.* EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:reset_reason.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_wrap.*) .sdata2 EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:reset_reason.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_wrap.*) .sdata2.* EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:reset_reason.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_wrap.*) .srodata EXCLUDE_FILE(*libgcov.a *libphy.a *libapp_trace.a:app_trace.* *libapp_trace.a:app_trace_util.* *libapp_trace.a:port_uart.* *libclang_rt.builtins.a:_divsf3.* *libclang_rt.builtins.a:restore.* *libclang_rt.builtins.a:save.* *libdriver.a:twai.* *libesp_driver_rmt.a:rmt_rx.* *libesp_driver_rmt.a:rmt_tx.* *libesp_driver_twai.a:esp_twai.* *libesp_driver_twai.a:esp_twai_onchip.* *libesp_hw_support.a:clk_utils.* *libesp_hw_support.a:cpu.* *libesp_hw_support.a:esp_memory_utils.* *libesp_hw_support.a:gdma.* *libesp_hw_support.a:mspi_timing_tuning.* *libesp_hw_support.a:rtc_clk.* *libesp_hw_support.a:sar_periph_ctrl.* *libesp_hw_support.a:systimer.* *libesp_mm.a:esp_cache_msync.* *libesp_mm.a:esp_cache_utils.* *libesp_rom.a:esp_rom_print.* *libesp_rom.a:esp_rom_spiflash.* *libesp_rom.a:esp_rom_sys.* *libesp_rom.a:esp_rom_systimer.* *libesp_system.a:esp_err.* *libesp_system.a:image_process.* *libesp_system.a:reset_reason.* *libesp_system.a:ubsan.* *libgcc.a:_divsf3.* *libgcc.a:save-restore.* *libhal.a:adc_oneshot_hal.* *libhal.a:cache_hal.* *libhal.a:i2c_hal_iram.* *libhal.a:ledc_hal_iram.* *libhal.a:mmu_hal.* *libhal.a:spi_flash_encrypt_hal_iram.* *libhal.a:spi_flash_hal_gpspi.* *libhal.a:spi_flash_hal_iram.* *libhal.a:spi_hal_iram.* *libhal.a:spi_slave_hal_iram.* *libhal.a:systimer_hal.* *libhal.a:twai_hal_v1.* *libhal.a:uart_hal_iram.* *libhal.a:wdt_hal_iram.* *libheap.a:multi_heap.* *libheap.a:tlsf.* *liblog.a:log.* *liblog.a:log_format_text.* *liblog.a:log_lock.* *liblog.a:log_print.* *liblog.a:log_timestamp_common.* *liblog.a:util.* *libnewlib.a:abort.* *libnewlib.a:assert.* *libnewlib.a:heap.* *libnewlib.a:stdatomic.* *libsoc.a:lldesc.* *libspi_flash.a:flash_brownout_hook.* *libspi_flash.a:memspi_host_driver.* *libspi_flash.a:spi_flash_chip_boya.* *libspi_flash.a:spi_flash_chip_gd.* *libspi_flash.a:spi_flash_chip_generic.* *libspi_flash.a:spi_flash_chip_issi.* *libspi_flash.a:spi_flash_chip_mxic.* *libspi_flash.a:spi_flash_chip_th.* *libspi_flash.a:spi_flash_chip_winbond.* *libspi_flash.a:spi_flash_os_func_app.* *libspi_flash.a:spi_flash_wrap.*) .srodata.*)
    *(.rodata_wlog_error .rodata_wlog_error.*)
    *(.rodata_wlog_info .rodata_wlog_info.*)
    *(.rodata_wlog_warning .rodata_wlog_warning.*)
    *libdriver.a:twai.*(.rodata.__FUNCTION__.9 .rodata.__func__.0 .rodata.__func__.1 .rodata.__func__.2 .rodata.__func__.3 .rodata.__func__.4 .rodata.__func__.5 .rodata.__func__.6 .rodata.__func__.7 .rodata.__func__.8 .rodata.twai_configure_gpio.str1.4 .rodata.twai_driver_install_v2.str1.4 .rodata.twai_driver_uninstall_v2.str1.4 .rodata.twai_free_driver_obj.str1.4 .rodata.twai_release_gpio.str1.4 .rodata.twai_transmit_v2.str1.4)
    *libesp_driver_rmt.a:rmt_rx.*(.rodata.__FUNCTION__.0 .rodata.__FUNCTION__.1 .rodata.__FUNCTION__.2 .rodata.__FUNCTION__.4 .rodata.__FUNCTION__.5 .rodata.__FUNCTION__.7 .rodata.__FUNCTION__.8 .rodata.__FUNCTION__.9 .rodata.__func__.6 .rodata.rmt_del_rx_channel.str1.4 .rodata.rmt_ll_rx_set_carrier_high_low_ticks.str1.4 .rodata.rmt_new_rx_channel.str1.4 .rodata.rmt_rx_destroy.str1.4 .rodata.rmt_rx_disable.str1.4 .rodata.rmt_rx_enable.str1.4 .rodata.rmt_rx_register_event_callbacks.str1.4 .rodata.rmt_rx_register_to_group.str1.4 .srodata.cst8)
    *libesp_driver_rmt.a:rmt_tx.*(.rodata.__FUNCTION__.0 .rodata.__FUNCTION__.10 .rodata.__FUNCTION__.12 .rodata.__FUNCTION__.13 .rodata.__FUNCTION__.14 .rodata.__FUNCTION__.18 .rodata.__FUNCTION__.19 .rodata.__FUNCTION__.20 .rodata.__FUNCTION__.3 .rodata.__FUNCTION__.4 .rodata.__FUNCTION__.5 .rodata.__FUNCTION__.6 .rodata.__FUNCTION__.7 .rodata.__FUNCTION__.9 .rodata.__func__.1 .rodata.__func__.15 .rodata.__func__.16 .rodata.__func__.17 .rodata.__func__.2 .rodata.__func__.8 .rodata.rmt_del_tx_channel.str1.4 .rodata.rmt_ll_tx_set_carrier_high_low_ticks.str1.4 .rodata.rmt_new_sync_manager.str1.4 .rodata.rmt_new_tx_channel.str1.4 .rodata.rmt_transmit.str1.4 .rodata.rmt_tx_create_trans_queue.str1.4 .rodata.rmt_tx_destroy.str1.4 .rodata.rmt_tx_disable.str1.4 .rodata.rmt_tx_enable.str1.4 .rodata.rmt_tx_register_event_callbacks.str1.4 .rodata.rmt_tx_register_to_group.str1.4 .rodata.rmt_tx_switch_gpio.str1.4 .rodata.rmt_tx_wait_all_done.str1.4)
    *libesp_driver_twai.a:esp_twai.*(.rodata.__FUNCTION__.0 .rodata.__FUNCTION__.1 .rodata.__FUNCTION__.10 .rodata.__FUNCTION__.11 .rodata.__FUNCTION__.2 .rodata.__FUNCTION__.3 .rodata.__FUNCTION__.4 .rodata.__FUNCTION__.5 .rodata.__FUNCTION__.6 .rodata.__FUNCTION__.7 .rodata.__FUNCTION__.8 .rodata.__FUNCTION__.9 .rodata.twai_node_config_mask_filter.str1.4 .rodata.twai_node_config_range_filter.str1.4 .rodata.twai_node_delete.str1.4 .rodata.twai_node_disable.str1.4 .rodata.twai_node_enable.str1.4 .rodata.twai_node_get_info.str1.4 .rodata.twai_node_reconfig_timing.str1.4 .rodata.twai_node_recover.str1.4 .rodata.twai_node_register_event_callbacks.str1.4 .rodata.twai_node_transmit_wait_all_done.str1.4)
    *libesp_driver_twai.a:esp_twai_onchip.*(.rodata .rodata.__FUNCTION__.1 .rodata.__FUNCTION__.10 .rodata.__FUNCTION__.11 .rodata.__FUNCTION__.12 .rodata.__FUNCTION__.13 .rodata.__FUNCTION__.16 .rodata.__FUNCTION__.17 .rodata.__FUNCTION__.19 .rodata.__FUNCTION__.2 .rodata.__FUNCTION__.20 .rodata.__FUNCTION__.4 .rodata.__FUNCTION__.6 .rodata.__FUNCTION__.7 .rodata.__FUNCTION__.8 .rodata.__FUNCTION__.9 .rodata.__func__.0 .rodata.__func__.14 .rodata.__func__.15 .rodata.__func__.18 .rodata.__func__.3 .rodata.__func__.5 .rodata._ctrlr_release.str1.4 .rodata._node_calc_set_bit_timing.str1.4 .rodata._node_check_timing_valid.str1.4 .rodata._node_config_io.str1.4 .rodata._node_config_mask_filter.str1.4 .rodata._node_delete.str1.4 .rodata._node_disable.str1.4 .rodata._node_enable.str1.4 .rodata._node_recover.str1.4 .rodata._node_register_callbacks.str1.4 .rodata._node_set_bit_timing.str1.4 .rodata._node_set_clock_source.str1.4 .rodata._node_wait_tx_all_done.str1.4 .rodata._twai_rcc_clock_sel.str1.4 .rodata.twai_new_node_onchip.str1.4)
    *libesp_hw_support.a:cpu.*(.rodata.__func__.0)
    *libesp_hw_support.a:gdma.*(.rodata .rodata.__FUNCTION__.0 .rodata.__FUNCTION__.10 .rodata.__FUNCTION__.11 .rodata.__FUNCTION__.12 .rodata.__FUNCTION__.13 .rodata.__FUNCTION__.14 .rodata.__FUNCTION__.15 .rodata.__FUNCTION__.16 .rodata.__FUNCTION__.17 .rodata.__FUNCTION__.18 .rodata.__FUNCTION__.19 .rodata.__FUNCTION__.20 .rodata.__FUNCTION__.4 .rodata.__FUNCTION__.5 .rodata.__FUNCTION__.6 .rodata.__FUNCTION__.7 .rodata.__FUNCTION__.8 .rodata.__FUNCTION__.9 .rodata.__func__.1 .rodata.__func__.2 .rodata.__func__.3 .rodata.do_allocate_gdma_channel.str1.4 .rodata.gdma_config_transfer.str1.4 .rodata.gdma_connect.str1.4 .rodata.gdma_disconnect.str1.4 .rodata.gdma_install_tx_interrupt.str1.4 .rodata.gdma_register_rx_event_callbacks.str1.4 .rodata.gdma_register_tx_event_callbacks.str1.4 .rodata.gdma_release_group_handle.str1.4 .rodata.gdma_release_pair_handle.str1.4)
    *libesp_system.a:reset_reason.*(.rodata.__func__.0)
    *libheap.a:multi_heap.*(.rodata.__func__.0 .rodata.__func__.1 .rodata.__func__.2 .rodata.__func__.3 .rodata.__func__.4 .rodata.__func__.5 .rodata.__func__.6 .rodata.__func__.7 .rodata.__func__.8 .rodata.multi_heap_dump.str1.4 .rodata.multi_heap_dump_tlsf.str1.4 .rodata.multi_heap_find_containing_block_impl.str1.4 .rodata.multi_heap_register_impl.str1.4)
    *libheap.a:tlsf.*(.rodata.__func__.0 .rodata.__func__.1 .rodata.__func__.10 .rodata.__func__.11 .rodata.__func__.12 .rodata.__func__.13 .rodata.__func__.14 .rodata.__func__.15 .rodata.__func__.16 .rodata.__func__.17 .rodata.__func__.19 .rodata.__func__.2 .rodata.__func__.20 .rodata.__func__.3 .rodata.__func__.5 .rodata.__func__.6 .rodata.__func__.7 .rodata.__func__.8 .rodata.__func__.9 .rodata.control_construct.str1.4 .rodata.default_walker.str1.4 .rodata.tlsf_add_pool.str1.4 .rodata.tlsf_check.str1.4 .rodata.tlsf_create.str1.4 .rodata.tlsf_remove_pool.str1.4)
    *libspi_flash.a:spi_flash_os_func_app.*(.rodata.__func__.0 .rodata.esp_flash_spi23_default_os_functions)
    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    /**

     * C++ constructor tables.

     *

     * Excluding crtbegin.o/crtend.o since IDF doesn't use the toolchain crt.

     *

     * RISC-V gcc is configured with --enable-initfini-array so it emits

     * .init_array section instead. But the init_priority sections will be

     * sorted for iteration in ascending order during startup.

     * The rest of the init_array sections is sorted for iteration in descending

     * order during startup, however. Hence a different section is generated for

     * the init_priority functions which is iterated in ascending order during

     * startup. The corresponding code can be found in startup.c.

     */
    
 . = ALIGN(4); 
 __init_priority_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .init_array.*))
    __init_priority_array_end = ABSOLUTE(.);
    
 . = ALIGN(4); 
 __init_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .init_array))
    __init_array_end = ABSOLUTE(.);
    /* Addresses of memory regions reserved via SOC_RESERVE_MEMORY_REGION() */
    
 . = ALIGN(4); 
 soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);
    /* System init functions registered via ESP_SYSTEM_INIT_FN */
    
 . = ALIGN(4); 
 _esp_system_init_fn_array_start = ABSOLUTE(.);
    KEEP (*(SORT_BY_INIT_PRIORITY(.esp_system_init_fn.*)))
    _esp_system_init_fn_array_end = ABSOLUTE(.);
    _rodata_end = ABSOLUTE(.);
    . = ALIGN(ALIGNOF(.flash.tdata));
  } > default_rodata_seg
  ASSERT((ADDR(.flash.tdata) == ADDR(.flash.rodata) + SIZEOF(.flash.rodata)), "The gap between .flash.rodata and .flash.tdata must not exist to produce the final bin image.")
  .flash.tdata :
  {
    _thread_local_data_start = ABSOLUTE(.);
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(ALIGNOF(.flash.tbss));
    _thread_local_data_end = ABSOLUTE(.);
  } > default_rodata_seg
  ASSERT((ADDR(.flash.tbss) == ADDR(.flash.tdata) + SIZEOF(.flash.tdata)), "The gap between .flash.tdata and .flash.tbss must not exist to produce the final bin image.")
  .flash.tbss (NOLOAD) :
  {
    _thread_local_bss_start = ABSOLUTE(.);
    *(.tbss .tbss.* .gnu.linkonce.tb.*)
    *(.tcommon .tcommon.*)
    _thread_local_bss_end = ABSOLUTE(.);
  } > default_rodata_seg
  /**

   * This section contains all the rodata that is not used

   * at runtime, helping to avoid an increase in binary size.

   */
  .flash.rodata_noload (NOLOAD) :
  {
    /**

     * This symbol marks the end of flash.rodata. It can be utilized by the MMU

     * driver to maintain the virtual address.

     * NOLOAD rodata may not be included in this section.

     */
    _rodata_reserved_end = ADDR(.flash.tbss);
    *(.rodata_wlog_debug .rodata_wlog_debug.*)
    *(.rodata_wlog_verbose .rodata_wlog_verbose.*)
  } > default_rodata_seg
  /* Marks the end of IRAM code segment */
  .iram0.text_end (NOLOAD) :
  {
    /* Padding for possible CPU prefetch + alignment for PMS split lines */
    . += 16;
    . = ALIGN(512);
    /* iram_end_test section exists for use by memprot unit tests only */
    *(.iram_end_test)
    _iram_text_end = ABSOLUTE(.);
  } > iram0_0_seg
  .iram0.data :
  {
    
 . = ALIGN(16); 
 _iram_data_start = ABSOLUTE(.);
    *(.iram.data .iram.data.*)
    _coredump_iram_start = ABSOLUTE(.);
    *(.iram2.coredump .iram2.coredump.*)
    _coredump_iram_end = ABSOLUTE(.);
    _iram_data_end = ABSOLUTE(.);
  } > iram0_0_seg
  .iram0.bss (NOLOAD) :
  {
    
 . = ALIGN(16); 
 _iram_bss_start = ABSOLUTE(.);
    *(.iram.bss .iram.bss.*)
    _iram_bss_end = ABSOLUTE(.);
    
 . = ALIGN(16); 
 _iram_end = ABSOLUTE(.);
  } > iram0_0_seg
  /* Marks the end of data, bss and possibly rodata */
  .dram0.heap_start (NOLOAD) :
  {
    
 . = ALIGN(16); 
 _heap_start = ABSOLUTE(.);
  } > dram0_0_seg
  /**

   * This section is not included in the binary image; it is only present in the ELF file.

   * It is used to keep certain symbols in the ELF file.

   */
  .noload 0 (INFO) :
  {
    _noload_keep_in_elf_start = ABSOLUTE(.);
    KEEP(*(.noload_keep_in_elf .noload_keep_in_elf.*))
    _noload_keep_in_elf_end = ABSOLUTE(.);
  }
  /* DWARF 1 */
  .debug 0 : { *(.debug) }
  .line 0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo 0 : { *(.debug_srcinfo) }
  .debug_sfnames 0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges 0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_line 0 : { *(.debug_line) }
  .debug_frame 0 : { *(.debug_frame) }
  .debug_str 0 : { *(.debug_str) }
  .debug_loc 0 : { *(.debug_loc) }
  .debug_macinfo 0 : { *(.debug_macinfo) }
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  /* DWARF 3 */
  .debug_ranges 0 : { *(.debug_ranges) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames 0 : { *(.debug_varnames) }
  /* GNU DWARF 2 extensions */
  .debug_gnu_pubnames 0 : { *(.debug_gnu_pubnames) }
  .debug_gnu_pubtypes 0 : { *(.debug_gnu_pubtypes) }
  /* DWARF 4 */
  .debug_types 0 : { *(.debug_types) }
  /* DWARF 5 */
  .debug_addr 0 : { *(.debug_addr) }
  .debug_line_str 0 : { *(.debug_line_str) }
  .debug_loclists 0 : { *(.debug_loclists) }
  .debug_macro 0 : { *(.debug_macro) }
  .debug_names 0 : { *(.debug_names) }
  .debug_rnglists 0 : { *(.debug_rnglists) }
  .debug_str_offsets 0 : { *(.debug_str_offsets) }
  .comment 0 : { *(.comment) }
  .note.GNU-stack 0: { *(.note.GNU-stack) }
  .riscv.attributes 0: { *(.riscv.attributes) }
  /DISCARD/ :
  {
   /**

    * Discarding .rela.* sections results in the following mapping:

    * .rela.text.* -> .text.*

    * .rela.data.* -> .data.*

    * And so forth...

    */
   *(.rela.*)
   *(.got .got.plt) /* TODO: GCC-382 */
   *(.eh_frame_hdr)
   *(.eh_frame)
  }
}
ASSERT(((_iram_end - ORIGIN(iram0_0_seg)) <= LENGTH(iram0_0_seg)),
          "IRAM0 segment data does not fit.")
ASSERT(((_heap_start - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
          "DRAM segment data does not fit.")
