# app_ui.py
#
# OLED UI runner that works with OLED-only hardware.
# Everything else is mocked automatically by board.py.

import time
import uasyncio as asyncio

import config
from board import Board
from drivers.oled_ssd1306 import OLED_SSD1306
from ui.model import UIModel
from ui.navigator import UINavigator
from ui.renderer import UIRenderer


def _read_mcu_temp_c_best_effort(prev=0.0):
    """
    Best-effort MCU temperature for ESP32 MicroPython builds.
    Many ports expose esp32.raw_temperature() (often returns Fahrenheit).
    If unavailable, keep previous value.
    """
    try:
        import esp32
        if hasattr(esp32, "raw_temperature"):
            tf = esp32.raw_temperature()
            return (tf - 32.0) * (5.0 / 9.0)
    except Exception:
        pass
    return prev


class AppUI:
    def __init__(self):
        self.board = Board()
        self.oled = None
        self.model = UIModel()
        self.nav = UINavigator(rotate_ms=config.STATUS_ROTATE_MS)
        self.ren = None

        self._task = None
        self._running = False
        self._last_sec_ms = time.ticks_ms()

    def init(self):
        self.board.init_all(max_current_a=5.0)
        self.oled = OLED_SSD1306(self.board.i2c, addr=config.OLED_ADDR, width=128, height=64)
        self.ren = UIRenderer(self.oled)

    async def run(self, poll_ms=50):
        self._running = True
        while self._running:
            # consume edge events
            ev = self.board.poll_button_edges()

            self.nav.on_buttons(
                menu_pressed=ev["menu"],
                left_pressed=ev["left"],
                right_pressed=ev["right"],
                ok_pressed=ev["ok"],
            )
            self.nav.snapshot_to_model(self.model)

            # update timer
            now = time.ticks_ms()
            if time.ticks_diff(now, self._last_sec_ms) >= 1000:
                self._last_sec_ms = now
                self.model.status_seconds += 1

            # update model from (real or mock) devices
            self._update_input_metrics()
            self._update_battery_metrics()
            self._update_batt_temp()

            # actual MCU temp (best effort)
            self.model.mcu_temp_c = _read_mcu_temp_c_best_effort(self.model.mcu_temp_c)

            # render
            try:
                self.ren.render(self.model)
            except Exception as e:
                # Keep running even if renderer glitches
                print("UI render error:", e)

            await asyncio.sleep_ms(poll_ms)

    def start(self):
        if self._task is not None:
            return
        self.init()
        self._task = asyncio.create_task(self.run())

    def stop(self):
        self._running = False
        self._task = None

    # -------------------------
    # Telemetry updates
    # -------------------------
    def _update_input_metrics(self):
        try:
            self.model.input_voltage_v = self.board.ina_sys.read_bus_voltage_v()
            self.model.input_current_a = self.board.ina_sys.read_current_a()
        except Exception:
            pass

    def _update_battery_metrics(self):
        try:
            self.model.batt_voltage_v = self.board.ina_batt.read_bus_voltage_v()
            self.model.batt_current_a = self.board.ina_batt.read_current_a()
        except Exception:
            pass

    def _update_batt_temp(self):
        try:
            if not self.board.has_bq:
                self.model.batt_temp_c = None
                return
            from control.ntc import ts_percent_to_temp_c
            ts_pct = self.board.bq.read_ts_percent_regn()
            temp_c = ts_percent_to_temp_c(
                ts_percent=ts_pct,
                r_pullup_ohm=config.TS_PULLUP_OHMS,
                r0_ohm=config.TS_NTC_R0_OHMS,
                beta_k=config.TS_NTC_BETA_K,
                t0_c=config.TS_NTC_T0_C,
            )
            if temp_c != temp_c:  # NaN check
                temp_c = None
            self.model.batt_temp_c = temp_c
        except Exception:
            self.model.batt_temp_c = None

    # -------------------------
    # REPL helpers for button injection
    # -------------------------
    def press_menu(self, ms=80):  self.board.inputs.press("MENU", ms)
    def press_left(self, ms=80):  self.board.inputs.press("LEFT", ms)
    def press_right(self, ms=80): self.board.inputs.press("RIGHT", ms)
    def press_ok(self, ms=80):    self.board.inputs.press("OK", ms)

    # -------------------------
    # REPL helpers to change mock telemetry
    # -------------------------
    def set_mock_values(self, *,
                        vin_v=None, iin_a=None,
                        vbat_v=None, ibat_a=None,
                        ts_percent=None,
                        state_label=None,
                        mcu_temp_c=None):
        # INAs might be real or mock; only mocks have attributes
        if vin_v is not None:
            if hasattr(self.board.ina_sys, "bus_v"):
                self.board.ina_sys.bus_v = float(vin_v)
            self.model.input_voltage_v = float(vin_v)

        if iin_a is not None:
            if hasattr(self.board.ina_sys, "current_a"):
                self.board.ina_sys.current_a = float(iin_a)
            self.model.input_current_a = float(iin_a)

        if vbat_v is not None:
            if hasattr(self.board.ina_batt, "bus_v"):
                self.board.ina_batt.bus_v = float(vbat_v)
            self.model.batt_voltage_v = float(vbat_v)

        if ibat_a is not None:
            if hasattr(self.board.ina_batt, "current_a"):
                self.board.ina_batt.current_a = float(ibat_a)
            self.model.batt_current_a = float(ibat_a)

        if ts_percent is not None:
            if hasattr(self.board.bq, "ts_percent"):
                self.board.bq.ts_percent = float(ts_percent)
            if self.board.has_bq:
                try:
                    from control.ntc import ts_percent_to_temp_c
                    temp_c = ts_percent_to_temp_c(
                        ts_percent=float(ts_percent),
                        r_pullup_ohm=config.TS_PULLUP_OHMS,
                        r0_ohm=config.TS_NTC_R0_OHMS,
                        beta_k=config.TS_NTC_BETA_K,
                        t0_c=config.TS_NTC_T0_C,
                    )
                    if temp_c != temp_c:
                        temp_c = None
                    self.model.batt_temp_c = temp_c
                except Exception:
                    self.model.batt_temp_c = None
            else:
                self.model.batt_temp_c = None

        if state_label is not None:
            self.model.device_state_label = str(state_label)

        if mcu_temp_c is not None:
            # allow forcing it for demos even if real temp is available
            self.model.mcu_temp_c = float(mcu_temp_c)


# Global singleton for convenience
app = AppUI()
