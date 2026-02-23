# menu.py
import esp32
import time
import ssd1306
from neopixel import NeoPixel
from machine import Pin
from board import Board

# ----------------------------
# Utility: repeat button
# ----------------------------
class RepeatButton:
    """
    Generate "press events" from a held button with:
      - immediate event on press
      - after delay_ms, repeat every repeat_ms while held
    Active-low input: pressed when level == 0
    """
    def __init__(self, read_level_fn, *, delay_ms=400, repeat_ms=100):
        self.read_level = read_level_fn
        self.delay_ms = int(delay_ms)
        self.repeat_ms = int(repeat_ms)

        self._was_down = False
        self._t_down = 0
        self._t_last_rep = 0

    def poll(self, now_ms) -> bool:
        down = (self.read_level() == 0)  # active-low
        fired = False

        if down and not self._was_down:
            self._t_down = now_ms
            self._t_last_rep = now_ms
            fired = True

        elif down and self._was_down:
            held_ms = time.ticks_diff(now_ms, self._t_down)
            if held_ms >= self.delay_ms:
                since_rep = time.ticks_diff(now_ms, self._t_last_rep)
                if since_rep >= self.repeat_ms:
                    self._t_last_rep = now_ms
                    fired = True

        self._was_down = down
        return fired


def _edge_press(prev_down: bool, level: int):
    """Active-low edge detect: return (pressed_edge, new_prev_down)"""
    down = (level == 0)
    pressed = down and not prev_down
    return pressed, down


# ----------------------------
# Value stepping helpers
# ----------------------------
def _wrap_step_float(val, step, vmin, vmax, direction):
    v = float(val) + float(step) * (1 if direction > 0 else -1)
    step = float(step)
    vmin = float(vmin)
    vmax = float(vmax)

    if v > vmax + 1e-9:
        v = vmin
    elif v < vmin - 1e-9:
        v = vmax

    n = round((v - vmin) / step)
    v = vmin + n * step

    if v > vmax:
        v = vmax
    if v < vmin:
        v = vmin
    return v


def _wrap_step_int(val, step, vmin, vmax, direction):
    v = int(val) + int(step) * (1 if direction > 0 else -1)
    if v > int(vmax):
        v = int(vmin)
    elif v < int(vmin):
        v = int(vmax)
    return v


def _wrap_list(idx, n, direction):
    if n <= 0:
        return 0
    if direction > 0:
        return (idx + 1) % n
    return (idx - 1) % n


# ----------------------------
# Renderer helpers
# ----------------------------
def draw_inverted_text(oled, x, y, text):
    w = len(text) * 8
    oled.fill_rect(x, y, w, 8, 1)
    oled.text(text, x, y, 0)


# ----------------------------
# TS% -> °C conversion
# Pull-up: 10k to REGN, NTC: 10k@25C, Beta 3435 to GND
# TS% is Vts/Vregn * 100
# ----------------------------
def _ts_percent_to_temp_c(ts_percent,
                          r_pullup_ohm=10000.0,
                          r0_ohm=10000.0,
                          beta_k=3435.0,
                          t0_c=25.0):
    try:
        import math
        x = float(ts_percent) / 100.0
        if x <= 0.000001:
            x = 0.000001
        if x >= 0.999999:
            x = 0.999999

        r_ntc = float(r_pullup_ohm) * (x / (1.0 - x))
        t0_k = float(t0_c) + 273.15
        inv_t = (1.0 / t0_k) + (1.0 / float(beta_k)) * math.log(r_ntc / float(r0_ohm))
        t_k = 1.0 / inv_t
        return t_k - 273.15
    except Exception:
        return None


class MenuApp:
    def __init__(self):
        self.board = Board()
        self.oled = None

        # OK (BOOT) is on MCU GPIO9
        self.pin_ok = Pin(9, Pin.IN, Pin.PULL_UP)

        # --- CHARGE page state ---
        self._charge_next_draw_ms = 0
        self._charge_applied = False
        self._charge_last_msg = ""

        # --- SK6812 setup (GPIO8) ---
        self._led_pin = Pin(8, Pin.OUT)
        self._leds = NeoPixel(self._led_pin, 10)  # total strip length = 10

        # LED animation state
        self._led_pos = 0
        self._led_dir = 1
        self._led_next_ms = 0
        self._led_interval_ms = 120
        self._led_color = (30, 0, 40)
        
        # expander cached inputs
        self.in0 = 0xFF
        self.in1 = 0xFF

        cfg = __import__("config")

        # buttons on expander
        self.P_BTN1 = getattr(cfg, "P_BTN1", 0)  # MENU
        self.P_BTN2 = getattr(cfg, "P_BTN2", 1)  # LEFT
        self.P_BTN3 = getattr(cfg, "P_BTN3", 2)  # RIGHT

        # expander outputs
        self.P_FAN = getattr(cfg, "P_FAN", None)     # output, active high
        self.P_QON = getattr(cfg, "P_QON", None)     # output, pulse high to wake BQ (per your request)

        # TACH is on MCU now (GPIO4 in your current file)
        self.pin_tach = Pin(4, Pin.IN, Pin.PULL_UP)

        # Fan pulses-per-revolution (most PC fans are 2 PPR)
        self.TACH_PPR = 2

        # Edge state
        self._menu_prev = False
        self._ok_prev = False

        # Repeat handlers for left/right
        self.rep_left = RepeatButton(self._read_left_level)
        self.rep_right = RepeatButton(self._read_right_level)

        # Menu state
        self.mode = "MAIN"     # MAIN or EDIT or PAGE
        self.sel = 0
        self.page = None       # Charge/Discharge/Measure/Status

        self.items = [
            {"key": "ChgFrom", "type": "float", "step": 0.1,  "min": 1.5,  "max": 3.0,  "val": 2.5,  "fmt": "{:.1f}"},
            {"key": "ChgTo", "type": "float", "step": 0.01, "min": 4.00, "max": 4.30, "val": 4.20, "fmt": "{:.2f}"},
            {"key": "DscTo", "type": "float", "step": 0.01, "min": 3.00, "max": 4.00, "val": 3.70, "fmt": "{:.2f}"},
            {"key": "Capacity",  "type": "int",   "step": 100,  "min": 300,  "max": 65500, "val": 3000, "fmt": "{}"},
            {"key": "MaxTemp", "type": "int",   "step": 1,    "min": 30,   "max": 80,   "val": 50,   "fmt": "{}"},
            {"key": "Exec",  "type": "list",  "opts": ["Charge", "Discharge", "Measure", "Status"], "idx": 3},
        ]

        self._edit_backup = None

        # STATUS page timing
        self._status_next_draw_ms = 0
        self._last_page = None

        # BQ wake attempt (only once)
        self._bq_wake_attempted = False

        # Tachometer via IRQ (period measurement)
        self._tach_last_us = None
        self._tach_period_us = None
        self._tach_last_edge_us = None
        self._tach_rpm = 0.0
        self._tach_ema_rpm = None
        self._tach_timeout_us = 2_000_000  # 2s -> consider stopped

        # Install IRQ (falling edge is typical for open-collector tach)
        try:
            self.pin_tach.irq(trigger=Pin.IRQ_FALLING, handler=self._tach_irq)
        except Exception:
            # If IRQ not available, we can still show 0 rpm.
            pass

    def _led_update(self, now_ms):
        if time.ticks_diff(now_ms, self._led_next_ms) < 0:
            return

        self._led_next_ms = time.ticks_add(now_ms, self._led_interval_ms)

        # clear strip
        self._leds.fill((0, 0, 0))

        # animate LEDs 0–4
        self._leds[self._led_pos] = self._led_color
        self._leds.write()

        self._led_pos += self._led_dir
        if self._led_pos >= 4:
            self._led_pos = 4
            self._led_dir = -1
        elif self._led_pos <= 0:
            self._led_pos = 0
            self._led_dir = 1

    # ---- Expander I/O ----
    def _refresh_exp(self):
        if getattr(self.board, "has_exp", False) and getattr(self.board, "exp", None) is not None:
            try:
                self.in0, self.in1 = self.board.exp.read_all()
                return True
            except Exception:
                return False
        return False

    def _exp_pin_level(self, pin):
        p = int(pin)
        if p < 8:
            return (self.in0 >> p) & 1
        p -= 8
        return (self.in1 >> p) & 1

    def _exp_write_pin(self, pin, value):
        if pin is None:
            return
        if getattr(self.board, "exp", None) is None:
            return
        try:
            self.board.exp.write_pin(int(pin), 1 if value else 0)
        except Exception:
            pass

    # ---- Inputs ----
    def _read_left_level(self):
        if self._refresh_exp():
            return self._exp_pin_level(self.P_BTN2)
        return 0 if self.board.inputs.is_down("LEFT") else 1

    def _read_right_level(self):
        if self._refresh_exp():
            return self._exp_pin_level(self.P_BTN3)
        return 0 if self.board.inputs.is_down("RIGHT") else 1

    def _read_menu_level(self):
        if self._refresh_exp():
            return self._exp_pin_level(self.P_BTN1)
        return 0 if self.board.inputs.is_down("MENU") else 1

    def _poll_inputs(self):
        now = time.ticks_ms()

        left_evt = self.rep_left.poll(now)
        right_evt = self.rep_right.poll(now)

        menu_level = self._read_menu_level()
        ok_level = self.pin_ok.value()

        menu_pressed, self._menu_prev = _edge_press(self._menu_prev, menu_level)
        ok_pressed, self._ok_prev = _edge_press(self._ok_prev, ok_level)

        return {"left": left_evt, "right": right_evt, "menu": menu_pressed, "ok": ok_pressed}

    # ---- Tach IRQ ----
    def _tach_irq(self, pin):
        # Keep ISR tiny: capture delta
        try:
            now = time.ticks_us()
        except Exception:
            return

        if self._tach_last_us is not None:
            dt = time.ticks_diff(now, self._tach_last_us)
            if dt > 0:
                self._tach_period_us = dt
        self._tach_last_us = now
        self._tach_last_edge_us = now

    def _tach_compute_rpm(self):
        # If no edges for timeout -> stopped
        try:
            now = time.ticks_us()
        except Exception:
            return 0.0

        if self._tach_last_edge_us is None:
            return 0.0

        if time.ticks_diff(now, self._tach_last_edge_us) > self._tach_timeout_us:
            self._tach_period_us = None
            self._tach_ema_rpm = None
            return 0.0

        p = self._tach_period_us
        if p is None or p <= 0:
            return 0.0

        # rpm = 60e6 / (period_us * pulses_per_rev)
        ppr = self.TACH_PPR if self.TACH_PPR > 0 else 2
        rpm = 60_000_000.0 / (float(p) * float(ppr))

        # EMA smoothing
        a = 0.2
        if self._tach_ema_rpm is None:
            self._tach_ema_rpm = rpm
        else:
            self._tach_ema_rpm = (1.0 - a) * self._tach_ema_rpm + a * rpm

        return self._tach_ema_rpm

    # ---- Menu logic ----
    def _enter_edit(self):
        self.mode = "EDIT"
        it = self.items[self.sel]
        if it["type"] == "list":
            self._edit_backup = ("list", it["idx"])
        else:
            self._edit_backup = ("val", it["val"])

    def _cancel_edit(self):
        it = self.items[self.sel]
        if self._edit_backup:
            kind, v = self._edit_backup
            if kind == "list":
                it["idx"] = v
            else:
                it["val"] = v
        self._edit_backup = None
        self.mode = "MAIN"

    def _confirm_edit(self):
        it = self.items[self.sel]
        self._edit_backup = None

        if it["key"] == "Exec":
            action = it["opts"][it["idx"]]
            self.page = action
            self.mode = "PAGE"
        else:
            self.mode = "MAIN"

    def _step_current(self, direction):
        it = self.items[self.sel]
        if it["type"] == "float":
            it["val"] = _wrap_step_float(it["val"], it["step"], it["min"], it["max"], direction)
        elif it["type"] == "int":
            it["val"] = _wrap_step_int(it["val"], it["step"], it["min"], it["max"], direction)
        elif it["type"] == "list":
            it["idx"] = _wrap_list(it["idx"], len(it["opts"]), direction)

    def _move_sel(self, direction):
        n = len(self.items)
        if n <= 0:
            return
        if direction > 0:
            self.sel = (self.sel + 1) % n
        else:
            self.sel = (self.sel - 1) % n

    # ---- Rendering: MAIN ----
    def _render_main(self):
        self.oled.fill(0)
        y0 = 0
        for i, it in enumerate(self.items):
            y = y0 + i * 10
            key = it["key"]
            if it["type"] == "list":
                val_s = it["opts"][it["idx"]]
            else:
                val_s = it["fmt"].format(it["val"])

            left = "{}:".format(key)
            right = " {} ".format(val_s)

            if i == self.sel and self.mode == "MAIN":
                draw_inverted_text(self.oled, 0, y, left)
                self.oled.text(right, len(left) * 8, y, 1)
            elif i == self.sel and self.mode == "EDIT":
                self.oled.text(left, 0, y, 1)
                x_val = len(left) * 8
                draw_inverted_text(self.oled, x_val, y, right)
            else:
                self.oled.text(left, 0, y, 1)
                self.oled.text(right, len(left) * 8, y, 1)

        self.oled.show()

    def _get_item_value(self, key, default=None):
        for it in self.items:
            if it.get("key") == key:
                if it.get("type") == "list":
                    return it["opts"][it["idx"]]
                return it.get("val", default)
        return default

    def _bq_is_powered(self):
        """
        True if BQ is reachable and indicates power-good / VBUS good.
        """
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return False
        try:
            st = bq.read_status()
            # Your driver exposes 'power_good' in read_status() dict. :contentReference[oaicite:3]{index=3}
            if isinstance(st, dict) and st.get("power_good", False):
                return True
        except Exception:
            pass
        try:
            # Also available in driver. :contentReference[oaicite:4]{index=4}
            return bool(bq.read_vbus_good())
        except Exception:
            return False

    def _set_charge_enabled(self, enabled: bool):
        """
        /CE is active-low externally.
        """
        
        cfg = __import__("config")
        p_ce = getattr(cfg, "P_CE", None)
        if p_ce is not None:
            # active-low: enabled => 0, disabled => 1
            self._exp_write_pin(p_ce, 0 if enabled else 1)
    

    def _apply_charge_settings(self):
        """
        Apply menu settings to BQ25895. Returns (ok:bool, msg:str).
        If not powered/reachable -> ok False.
        """
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return (False, "CONNECT POWER")

        # If offline/unpowered, try your wake-once routine, then re-check
        if not self._bq_is_powered():
            self._bq_wake_once()
            if not self._bq_is_powered():
                return (False, "CONNECT POWER")

        # Pull values from your menu
        chg_to_v = float(self._get_item_value("ChgTo", 4.20))  # volts
        cap_mah = int(self._get_item_value("Capacity", 3000))

        # Policy: 0.5C default charge current (your earlier spec)
        ichg_ma = int(cap_mah * 0.5)

        # Reasonable input current limit (you can tune later)
        # If you want it derived from current too: iinlim ~= ichg_ma (rough)
        iinlim_ma = min(max(ichg_ma, 500), 3000)

        # Apply to BQ (driver clamps in each setter) :contentReference[oaicite:5]{index=5}
        try:
            # Enable ADC continuous so STATUS reads update ~1s
            bq.adc_set_continuous(True)
        except Exception:
            pass

        try:
            bq.set_hiz(False)
        except Exception:
            pass

        # Enable charger via I2C bit + /CE pin (external) :contentReference[oaicite:6]{index=6}
        try:
            bq.set_charge_enable_i2c(True)
        except Exception:
            pass
        self._set_charge_enabled(True)

        try:
            bq.set_input_current_limit_ma(iinlim_ma)
        except Exception:
            pass

        try:
            bq.set_charge_voltage_mv(int(chg_to_v * 1000))
        except Exception:
            pass

        try:
            bq.set_fast_charge_current_ma(ichg_ma)
        except Exception:
            pass

        # Precharge / termination defaults (optional but nice)
        try:
            bq.set_precharge_current_ma(max(64, int(ichg_ma * 0.1)))
        except Exception:
            pass
        try:
            bq.set_termination_current_ma(max(64, int(ichg_ma * 0.05)))
        except Exception:
            pass

        return (True, "APPLIED {:0.2f}V".format(chg_to_v))

    # ---- BQ status helpers ----
    def _bq_try_read_status(self):
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return None
        try:
            return bq.read_status()
        except Exception:
            return None

    def _bq_wake_once(self):
        if self._bq_wake_attempted:
            return
        self._bq_wake_attempted = True

        # Pulse QON high 1s then low
        if self.P_QON is not None:
            self._exp_write_pin(self.P_QON, 1)
            time.sleep(1.0)
            self._exp_write_pin(self.P_QON, 0)

        # Best-effort re-init if board exposes helper
        init_bq = getattr(self.board, "init_bq", None)
        if callable(init_bq):
            try:
                init_bq()
            except Exception:
                pass

    def _format_chg_11(self):
        """
        Returns exactly <=11 chars (not counting 'CHG: ').
        If BQ not available => 'UNPOWERED'
        Otherwise derive from read_status() dict.
        """
        st = self._bq_try_read_status()
        if st is None:
            # attempt wake once, then re-check
            self._bq_wake_once()
            st = self._bq_try_read_status()
            if st is None:
                return "UNPOWERED"

        # Try to interpret fields (best-effort, depends on your driver)
        chrg_stat = st.get("chrg_stat", None) if isinstance(st, dict) else None
        pg = st.get("pg_stat", None) if isinstance(st, dict) else None
        vbus = st.get("vbus_stat", None) if isinstance(st, dict) else None

        # Charge state mapping
        ch_map = {
            0: "IDLE",     # not charging
            1: "PCHG",     # pre-charge
            2: "FCHG",     # fast charge (CC/CV)
            3: "DONE",   # charge done
        }
        ch = ch_map.get(chrg_stat, "UNK")

        # Power-good flag
        pg_s = "PG" if pg in (1, True) else "--"

        # VBUS mapping (best-effort)
        vb_map = {
            0: "UNK",
            1: "USB",
            2: "ADP",
            3: "OTG",
        }
        vb = vb_map.get(vbus, "")
        if vb:
            out = "{} {} {}".format(ch, pg_s, vb)
        else:
            out = "{} {}".format(ch, pg_s)

        # Ensure <=11 chars
        if len(out) > 11:
            out = out[:11]
        return out

    def _read_bq_temp_c(self):
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return None
        try:
            ts_pct = bq.read_ts_percent_regn()
        except Exception:
            return None
        return _ts_percent_to_temp_c(ts_pct)

    def _charge_on_enter(self):
        self._charge_applied = False
        self._charge_last_msg = ""
        now = time.ticks_ms()
        self._charge_next_draw_ms = now  # draw immediately

        ok, msg = self._apply_charge_settings()
        self._charge_applied = ok
        self._charge_last_msg = msg

    def _charge_on_exit(self):
        self._set_charge_enabled(False)

    # ---- STATUS page support ----
    def _status_on_enter(self):
        # fan on
        self._exp_write_pin(self.P_FAN, 1)

        now = time.ticks_ms()
        self._status_next_draw_ms = now  # draw immediately

        # Try BQ status once immediately (may trigger wake-once)
        _ = self._format_chg_11()

    def _status_on_exit(self):
        # fan off
        self._exp_write_pin(self.P_FAN, 0)

    def _render_charge(self, now_ms):
        if time.ticks_diff(now_ms, self._charge_next_draw_ms) < 0:
            return
        self._charge_next_draw_ms = time.ticks_add(now_ms, 1000)

        # If not applied, just show connect power message
        if not self._charge_applied:
            self.oled.fill(0)
            #self.oled.text("CHARGE", 0, 0, 1)
            self.oled.text(" CONNECT POWER", 0, 16, 1)
            self.oled.text(" XT30 / USB-PD", 0, 28, 1)
            self.oled.text(" MENU to exit", 4, 48, 1)
            self.oled.show()
            return

        # If applied, show a couple of BQ values + your message
        bq = getattr(self.board, "bq", None)
        chg11 = self._format_chg_11()  # already exists, includes wake-once; returns UNPOWERED if missing :contentReference[oaicite:7]{index=7}

        vsys = None
        vbus = None
        ichg_adc = None
        try:
            if bq is not None:
                vsys = bq.read_vsys_v()               # :contentReference[oaicite:8]{index=8}
                vbus = bq.read_vbus_v()               # :contentReference[oaicite:9]{index=9}
                ichg_adc = bq.read_charge_current_ma_adc()  # :contentReference[oaicite:10]{index=10}
        except Exception:
            pass

        def f2(x): return "N/A" if x is None else "{:.2f}".format(float(x))
        def fi(x): return "N/A" if x is None else "{}".format(int(x))

        self.oled.fill(0)
        self.oled.text("CHG: {}".format(chg11), 0, 0, 1)
        self.oled.text("VBUS: {}".format(f2(vbus)), 0, 12, 1)
        self.oled.text("VSYS: {}".format(f2(vsys)), 0, 24, 1)
        self.oled.text("ICHG: {}mA".format(fi(ichg_adc)), 0, 36, 1)
        self.oled.text(self._charge_last_msg[:16], 0, 48, 1)
        self.oled.show()

    def _render_status(self, now_ms):
        # 1 FPS redraw
        if time.ticks_diff(now_ms, self._status_next_draw_ms) < 0:
            return
        self._status_next_draw_ms = time.ticks_add(now_ms, 1000)

        # Read INP/BAT
        vin = iin = vbat = ibat = None
        try:
            vin = self.board.ina_sys.read_bus_voltage_v()
            iin = self.board.ina_sys.read_current_from_shunt_a()
        except Exception:
            pass
        try:
            vbat = self.board.ina_batt.read_bus_voltage_v()
            ibat = self.board.ina_batt.read_current_from_shunt_a()
        except Exception:
            pass

        t_mcu = None
        try:
            t_mcu = esp32.mcu_temperature()
        except Exception:
            t_mcu = None

        t_bq = self._read_bq_temp_c()

        rpm = self._tach_compute_rpm()

        # Format helpers
        def f2(x):
            return "N/A" if x is None else "{:.2f}".format(float(x))

        def f1(x):
            return "N/A" if x is None else "{:.1f}".format(float(x))

        rpm_s = "{:.0f}".format(rpm) if rpm is not None else "0"

        chg11 = self._format_chg_11()

        # Render 5 lines (tight spacing)
        self.oled.fill(0)
        self.oled.text("INP: {}/{}".format(f1(vin), f1(iin)), 0, 0, 1)
        self.oled.text("BAT: {}/{}".format(f2(vbat), f2(ibat)), 0, 12, 1)
        self.oled.text("TMP: {}/{}".format(f1(t_mcu), f1(t_bq)), 0, 24, 1)
        self.oled.text("CHG: {}".format(chg11), 0, 36, 1)
        self.oled.text("FAN: {}rpm".format(rpm_s), 0, 48, 1)
        self.oled.show()

    # ---- Generic page render (non-status) ----
    def _render_page_generic(self):
        self.oled.fill(0)
        title = self.page if self.page else "PAGE"
        x = max(0, (128 - len(title) * 8) // 2)
        self.oled.text(title, x, 24, 1)
        self.oled.text("MENU to return", 16, 48, 1)
        self.oled.show()

    # ---- Main loop ----
    def run(self, poll_ms=20):
        self.board.init_all()
        self.oled = ssd1306.SSD1306_I2C(128, 64, self.board.i2c, addr=0x3C)

        try:
            self.oled.rotate(0)
        except Exception:
            pass
        
        last = time.ticks_ms()
        while True:
            now = time.ticks_ms()
            _ = time.ticks_diff(now, last)
            last = now
            
            self._led_update(now)

            ev = self._poll_inputs()

            # PAGE mode: only MENU exits
            if self.mode == "PAGE":
                if self._last_page != self.page:
                    # leaving old page
                    if self._last_page == "Status":
                        self._status_on_exit()
                    if self._last_page == "Charge":
                        self._charge_on_exit()

                    # entering new page
                    if self.page == "Status":
                        self._status_on_enter()
                    if self.page == "Charge":
                        self._charge_on_enter()

                    self._last_page = self.page

                if ev["menu"]:
                    if self.page == "Status":
                        self._status_on_exit()
                    if self.page == "Charge":
                        self._charge_on_exit()
                    self.mode = "MAIN"
                    self.page = None
                    self._last_page = None
                    self._render_main()
                    time.sleep_ms(poll_ms)
                    continue

                if self.page == "Status":
                    self._render_status(now)
                elif self.page == "Charge":
                    self._render_charge(now)
                else:
                    self._render_page_generic()

                time.sleep_ms(poll_ms)
                continue

            # MAIN / EDIT mode
            if self.mode == "MAIN":
                if ev["left"]:
                    self._move_sel(-1)
                if ev["right"]:
                    self._move_sel(+1)
                if ev["ok"]:
                    self._enter_edit()
            else:  # EDIT
                if ev["left"]:
                    self._step_current(-1)
                if ev["right"]:
                    self._step_current(+1)
                if ev["ok"]:
                    self._confirm_edit()
                if ev["menu"]:
                    self._cancel_edit()

            self._render_main()
            time.sleep_ms(poll_ms)


app = MenuApp()
app.run()
