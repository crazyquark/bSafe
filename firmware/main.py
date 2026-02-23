# main.py
import esp32
import time
import math
import drivers.ssd1306 as ssd1306
from neopixel import NeoPixel
from machine import Pin, PWM, ADC
from board import Board

# External NTC divider constants
_EXT_R_PULL_OHM = 10_000.0     # 10k pull-up to 3V3
_EXT_R0_OHM     = 89_000.0     # ~100k NTC @ 25C
_EXT_BETA_K     = 3950.0       # adjust to your NTC (e.g. 3435, 3950, 4100...)
_EXT_T0_C       = 25.0
_EXT_VREF       = 3.3          # rough; ADC isn't precision-referenced

# Create ADC object once (recommended)
_ext_adc = ADC(Pin(1))
try:
    _ext_adc.atten(ADC.ATTN_11DB)  # allows reading up to ~3.3V range on ESP32-class ports
except Exception:
    pass
try:
    _ext_adc.width(ADC.WIDTH_12BIT)    # make sure read() is 0..4095
except Exception:
    pass

def read_ext_temp() -> float | None:
    """
    External NTC on GPIO1 via divider:
      3V3 -- 10k -- ADC(GPIO1) -- NTC -- GND
    Uses Steinhart-Hart fit to your measured points:
      35C@3.2V, 45C@3.0V, 77C@2.1V (assuming Vref=3.3 and Rpull=10k).
    """
    import time, math

    # 16-sample smoothing @ 1ms
    acc = 0
    for _ in range(20):
        acc += _ext_adc.read()
        time.sleep_ms(1)
    raw = acc / 20.0
    raw_max = 4095.0
    if raw <= 5 or raw >= (raw_max - 5):
        return None

    r_ntc = _EXT_R_PULL_OHM * (raw / (raw_max - raw))
    if r_ntc <= 1.0:
        return None

    # Steinhart-Hart coefficients (your calibration)
    A = -0.0006247654143591243
    B =  0.0004308473967926403
    C = -0.0000007813696611701303

    lnr = math.log(r_ntc)
    inv_t = A + B * lnr + C * (lnr ** 3)
    t_k = 1.0 / inv_t
    return t_k - 273.15

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
    def __init__(self, read_level_fn, *, delay_ms=360, repeat_ms=120):
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
    
def draw_inverted_title(oled, text):
    x = 64 - len(text) * 4
    oled.fill_rect(0, 0, 127, 18, 1)
    oled.text(text, x, 5, 0)

def draw_progress_bar(oled, x, y, w, h, frac):
    # frac: 0.0 .. 1.0
    if frac is None:
        frac = 0.0
    if frac < 0.0:
        frac = 0.0
    if frac > 1.0:
        frac = 1.0

    oled.rect(x, y, w, h, 1)
    fill_w = int((w - 2) * frac)
    if fill_w > 0:
        oled.fill_rect(x + 1, y + 1, fill_w, h - 2, 1)

# ----------------------------
# TS% -> °C conversion
# Pull-up: 5~10k to REGN, NTC: 10k@25C, Beta 3435 to GND
# TS% is Vts/Vregn * 100
# ----------------------------
def _ts_percent_to_temp_c(ts_percent,
                          r_pullup_ohm=7500.0,  #  10k || 30k
                          r0_ohm=10000.0,       # "103" thermistor
                          beta_k=3435.0,
                          t0_c=25.0):
    try:
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


class App:
    # LED colors
    LED_OFF = (0, 0, 0)
    LED_CHG = (10, 50, 0)   # green
    LED_ERR = (60, 0, 0)    # red
    LED_DSC = (50, 25, 0)   # orange
    
    LED_BLINK_MS = 400

    # Battery presence threshold (simple, no debounce yet)
    VBAT_PRESENT_V = 1.5

    # Tach glitch rejection: ignore dt < MIN_PERIOD_US
    # 6000 rpm @ 2 PPR => 5000us period. Anything much smaller is noise.
    TACH_MIN_PERIOD_US = 1000

    def __init__(self):
        self.board = Board()
        self.oled = None

        # OK (BOOT) is on MCU GPIO9
        self.pin_ok = Pin(9, Pin.IN, Pin.PULL_UP)

        # --- SK6812 setup (GPIO8) ---
        self._led_pin = Pin(8, Pin.OUT)
        self._leds = NeoPixel(self._led_pin, 10)  # total strip length = 10

        # Idle LED animation (ping-pong on LEDs 1-5)
        self._idle_led_pos = 0
        self._idle_led_dir = 1
        self._idle_led_next_ms = 0
        self._idle_led_interval_ms = 120
        self._idle_led_color = (40, 20, 50)

        # Charge LED indicator state
        self._led_blink_on = False
        self._led_blink_next_ms = 0
        self._chg_error_latched = False
        self._chg_full_latched = False

        # expander cached inputs
        self.in0 = 0xFF
        self.in1 = 0xFF

        cfg = __import__("config")

        # buttons on expander
        self.P_BTN1 = getattr(cfg, "P_BTN1", 0)  # MENU
        self.P_BTN2 = getattr(cfg, "P_BTN2", 1)  # LEFT
        self.P_BTN3 = getattr(cfg, "P_BTN3", 2)  # RIGHT

        # expander outputs
        self.P_FAN = getattr(cfg, "P_FAN", None)   # output, active high
        self.P_QON = getattr(cfg, "P_QON", None)   # output, pulse high to wake BQ
        self.P_CE  = getattr(cfg, "P_CE", None)    # output, active-low enable

        # TACH is on MCU (GPIO4)
        self.pin_tach = Pin(4, Pin.IN, Pin.PULL_UP)
        self.TACH_PPR = 2

        # --- DISCHARGE/MEASURE load PWM (GPIO0), active high ---
        self._dsc_pwm = None
        self._dsc_pct = 10  # percent, min 10, max 100, step 5
        self._dsc_duty = int(1023 * self._dsc_pct / 100.0)
        self._dsc_freq_hz = 10000  # 10 kHz
        self._dsc_next_draw_ms = 0

        # --- MEASURE I.R. results/state ---
        self._meas_r_30 = None   # float ohms (dv/i)
        self._meas_r_60 = None   # float ohms (dv/i)
        self._meas_v1_30 = None  # loaded voltage
        self._meas_v1_60 = None  # loaded voltage
        self._meas_vdrop_30 = None
        self._meas_i_30 = None   # A
        self._meas_vdrop_60 = None
        self._meas_i_60 = None   # A
        self._meas_next_draw_ms = 0
        
        # Edge state
        self._menu_prev = False
        self._ok_prev = False

        # Repeat handlers for left/right (200ms repeat after 400ms delay)
        self.rep_left = RepeatButton(self._read_left_level, delay_ms=400, repeat_ms=200)
        self.rep_right = RepeatButton(self._read_right_level, delay_ms=400, repeat_ms=200)

        # Menu state
        self.mode = "MAIN"     # MAIN or EDIT or PAGE
        self.sel = 0
        self.page = None       # Charge/Discharge/Measure/Status
        self._last_page = None

        self.items = [
            # {"key": "ChgFrom",  "type": "float", "step": 0.1,  "min": 1.5,  "max": 3.0,   "val": 2.5,  "fmt": "{:.1f}"},
            {"key": "Address",  "type": "int",   "step": 1,    "min": 0,    "max": 63,    "val": 0,    "fmt": "{}"},
            {"key": "ChgTo",    "type": "float", "step": 0.01, "min": 4.00, "max": 4.30,  "val": 4.20, "fmt": "{:.2f}"},
            {"key": "DscTo",    "type": "float", "step": 0.01, "min": 3.00, "max": 4.00,  "val": 3.70, "fmt": "{:.2f}"},
            {"key": "Capacity", "type": "int",   "step": 100,  "min": 300,  "max": 65500, "val": 10000, "fmt": "{}"},
            {"key": "MaxTemp",  "type": "int",   "step": 1,    "min": 30,   "max": 80,    "val": 50,   "fmt": "{}"},
            {"key": "Exec",     "type": "list",  "opts": ["Charge", "Discharge", "Measure", "Status"], "idx": 3},
        ]
        self._edit_backup = None

        # STATUS page timing
        self._status_next_draw_ms = 0

        # CHARGE page timing
        self._charge_next_draw_ms = 0
        self._charge_configured = False

        # BQ wake attempt (only once)
        self._bq_wake_attempted = False

        # Tachometer via IRQ (period measurement)
        self._tach_last_us = None
        self._tach_period_us = None
        self._tach_last_edge_us = None
        self._tach_ema_rpm = None
        self._tach_timeout_us = 2_000_000  # 2s -> consider stopped

        try:
            self.pin_tach.irq(trigger=Pin.IRQ_FALLING, handler=self._tach_irq)
        except Exception:
            pass

    # -------------------------
    # Expander I/O
    # -------------------------
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

    # -------------------------
    # Inputs
    # -------------------------
    def _read_left_level(self):
        if self._refresh_exp():
            return self._exp_pin_level(self.P_BTN2)
        return 1

    def _read_right_level(self):
        if self._refresh_exp():
            return self._exp_pin_level(self.P_BTN3)
        return 1

    def _read_menu_level(self):
        if self._refresh_exp():
            return self._exp_pin_level(self.P_BTN1)
        return 1

    def _poll_inputs(self):
        now = time.ticks_ms()

        left_evt = self.rep_left.poll(now)
        right_evt = self.rep_right.poll(now)

        menu_level = self._read_menu_level()
        ok_level = self.pin_ok.value()

        menu_pressed, self._menu_prev = _edge_press(self._menu_prev, menu_level)
        ok_pressed, self._ok_prev = _edge_press(self._ok_prev, ok_level)

        return {"left": left_evt, "right": right_evt, "menu": menu_pressed, "ok": ok_pressed}

    # -------------------------
    # Tach IRQ
    # -------------------------
    def _tach_irq(self, pin):
        try:
            now = time.ticks_us()
        except Exception:
            return

        if self._tach_last_us is not None:
            dt = time.ticks_diff(now, self._tach_last_us)
            if dt >= self.TACH_MIN_PERIOD_US:
                self._tach_period_us = dt
                self._tach_last_us = now
                self._tach_last_edge_us = now
            else:
                # glitch, ignore
                return
        else:
            self._tach_last_us = now
            self._tach_last_edge_us = now

    def _tach_compute_rpm(self):
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

        ppr = self.TACH_PPR if self.TACH_PPR > 0 else 2
        rpm = 60_000_000.0 / (float(p) * float(ppr))

        # EMA smoothing
        a = 0.3
        if self._tach_ema_rpm is None:
            self._tach_ema_rpm = rpm
        else:
            self._tach_ema_rpm = (1.0 - a) * self._tach_ema_rpm + a * rpm

        return self._tach_ema_rpm

    # -------------------------
    # Menu logic
    # -------------------------
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

    # -------------------------
    # Rendering: MAIN
    # -------------------------
    def _render_main(self):
        self.oled.fill(0)
        y0 = 0
        for i, it in enumerate(self.items):
            y = y0 + i * 9
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

    # -------------------------
    # Helpers: settings values
    # -------------------------
    def _get_item_value(self, key, default=None):
        for it in self.items:
            if it.get("key") == key:
                if it.get("type") == "list":
                    return it["opts"][it["idx"]]
                return it.get("val", default)
        return default

    # -------------------------
    # Charger helpers
    # -------------------------
    def _set_ce_enabled(self, enabled: bool):
        """
        /CE is active-low externally:
          enabled=True  -> P_CE LOW
          enabled=False -> P_CE HIGH
        """
        if self.P_CE is None:
            return
        self._exp_write_pin(self.P_CE, 0 if enabled else 1)

    def _bq_try_read_status(self):
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return None
        try:
            return bq.read_status()
        except Exception:
            return None

    def _bq_is_powered(self):
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return False
        try:
            st = bq.read_status()
            if isinstance(st, dict) and st.get("power_good", False):
                return True
        except Exception:
            pass
        try:
            return bool(bq.read_vbus_good())
        except Exception:
            return False

    def _bq_wake_once(self, progress_cb=None):
        if self._bq_wake_attempted:
            return
        self._bq_wake_attempted = True

        # Pulse QON high for 2.5s, update progress bar while we wait
        if self.P_QON is not None:
            self._exp_write_pin(self.P_QON, 1)

            t0 = time.ticks_ms()
            dur = 2500
            while True:
                now = time.ticks_ms()
                dt = time.ticks_diff(now, t0)
                if dt >= dur:
                    break
                if progress_cb is not None:
                    try:
                        progress_cb(dt / dur)
                    except Exception:
                        pass
                time.sleep_ms(50)

            self._exp_write_pin(self.P_QON, 0)

        # Best-effort re-init if board exposes helper
        init_bq = getattr(self.board, "init_bq", None)
        if callable(init_bq):
            try:
                init_bq()
            except Exception:
                pass


    def _read_bq_temp_c(self):
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return None
        try:
            ts_pct = bq.read_ts_percent_regn()
        except Exception:
            return None
        return _ts_percent_to_temp_c(ts_pct)

    def _read_vbat_v(self):
        try:
            return float(self.board.ina_batt.read_bus_voltage_v())
        except Exception:
            return None
        
    def _input_power_available(self):
        # "Power available" = BQ reachable + INA226 VIN > ~7V
        if self._bq_try_read_status() is None:
            return False
        try:
            vin = float(self.board.ina_sys.read_bus_voltage_v())
        except Exception:
            return False
        return vin > 7.5


    def _battery_present(self, vbat_v):
        return (vbat_v is not None) and (vbat_v >= self.VBAT_PRESENT_V)

    def _estimate_pct(self, vbat, vmin, vmax):
        if vbat is None or vmax <= vmin:
            return None
        x = (vbat - vmin) / (vmax - vmin)
        if x < 0.0: x = 0.0
        if x > 1.0: x = 1.0
        return x * 100.0

    def _bq_charge_mode(self, st, vbat, v_target):
        """
        Returns mode string for UI/LED logic.
        """
        if st is None or not isinstance(st, dict):
            return "UNPOWERED"

        cs = st.get("chrg_stat", None)
        if cs == 0:
            return "OFF"
        if cs == 1:
            return "PRE"
        if cs == 2:
            if vbat is not None and vbat >= (v_target - 0.03):
                return "CV"
            return "CC"
        if cs == 3:
            return "DONE"
        return "UNK"

    def _apply_bq_registers_for_charge(self):
        """
        Configure BQ with menu settings for charging.
        Returns True if BQ is powered/reachable and settings written.
        """
        bq = getattr(self.board, "bq", None)
        if bq is None:
            return False

        # Ensure powered; try wake once
        # self._bq_wake_once()
        if not self._bq_is_powered():
            return False

        # Pull settings
        v_target = float(self._get_item_value("ChgTo", 4.20))
        v_min = float(self._get_item_value("ChgFrom", 2.5))
        cap_mah = int(self._get_item_value("Capacity", 3000))

        # 0.5C of min(rated, 5000mAh)
        cap_for_c = cap_mah if cap_mah < 10200 else 10200
        ichg_ma = int(cap_for_c * 0.5)  # up to ~2500mA for 5000mAh cap_for_c
        # Let user set bigger capacities without exceeding BQ caps:
        if ichg_ma > 5056:
            ichg_ma = 5056

        # Input current limit: set to max (3250mA) for now (power-path will limit if needed)
        iinlim_ma = 3250

        # Log what we write
        print("[CHG] Config:")
        print("  Vmin   :", v_min, "V")
        print("  Vtarget:", v_target, "V")
        print("  Cap    :", cap_mah, "mAh (cap_for_C=%d)" % cap_for_c)
        print("  ICHG   :", ichg_ma, "mA (0.5C)")
        print("  IINLIM :", iinlim_ma, "mA")

        # Write regs via driver API
        try:
            bq.adc_set_continuous(True)
        except Exception:
            pass

        try:
            bq.set_hiz(False)
        except Exception:
            pass

        # Enable charging (I2C bit) and external /CE pin LOW
        try:
            bq.set_charge_enable_i2c(True)
        except Exception:
            pass
        self._set_ce_enabled(True)

        try:
            bq.set_input_current_limit_ma(iinlim_ma)
        except Exception:
            pass

        try:
            bq.set_charge_voltage_mv(int(v_target * 1000))
        except Exception:
            pass

        try:
            bq.set_fast_charge_current_ma(ichg_ma)
        except Exception:
            pass

        # Precharge / termination
        ipre_ma = max(64, int(ichg_ma * 0.1))
        iterm_ma = max(64, int(ichg_ma * 0.05))

        try:
            bq.set_precharge_current_ma(ipre_ma)
        except Exception:
            pass
        try:
            bq.set_termination_current_ma(iterm_ma)
        except Exception:
            pass

        try:
            reg07 = self.board.bq._rd(0x07)
            reg07 = (reg07 & ~0x30) | 0x00      # WATCHDOG bits -> 00 (disabled)
            reg07 = (reg07 & ~0x08) | 0x00      # EN_TIMER -> 0 (disable safety timer)
            reg07 = (reg07 & ~0x80) | 0x80      # EN_TERM -> 1
            self.board.bq._wr(0x07, reg07)
        except Exception as e:
            print("[CHG] REG07 write failed:", e)

        # Kick watchdog once anyway (harmless even if disabled)
        try:
            self.board.bq.watchdog_kick()
        except Exception:
            pass
        
        '''
        # Best-effort: set REG07 bits (enable termination, disable watchdog & safety timer)
        # This uses driver internals; safe and common in MicroPython.
        
        try:
            import drivers.bq25895 as bqmod
            # EN_TERM=1, STAT_DIS=0, WATCHDOG=00 (disabled), EN_TIMER=0
            bq._update_bits(bqmod.REG07, bqmod.B_EN_TERM, bqmod.B_EN_TERM)
            bq._update_bits(bqmod.REG07, bqmod.M_WATCHDOG, 0)
            bq._update_bits(bqmod.REG07, bqmod.B_EN_TIMER, 0)
            bq._update_bits(bqmod.REG07, bqmod.B_STAT_DIS, 0)
        except Exception:
            pass
        '''
        try:
            print("[CHG] REG03:", hex(self.board.bq._rd(0x03)))  # CHG_CONFIG, OTG, etc
            print("[CHG] REG07:", hex(self.board.bq._rd(0x07)))  # watchdog/timer/term bits
            print("[CHG] REG0B:", hex(self.board.bq._rd(0x0B)))  # status
            print("[CHG] REG0C:", hex(self.board.bq._rd(0x0C)))  # faults
            print("[CHG] TS%   :", self.board.bq.read_ts_percent_regn())
        except Exception as e:
            print("[CHG] reg readback failed:", e)

        try:
            f = self.board.bq.read_faults()
            print("[CHG] Faults:", f)
        except Exception as e:
            print("[CHG] faults read failed:", e)

        # Read back status once
        try:
            st = bq.read_status()
            print("[CHG] Status:", st)
        except Exception as e:
            print("[CHG] Status read failed:", e)

        return True

    # -------------------------
    # Page enter/exit
    # -------------------------
    def _status_on_enter(self):
        self._exp_write_pin(self.P_FAN, 1)
        self._status_next_draw_ms = time.ticks_ms()

    def _status_on_exit(self):
        self._exp_write_pin(self.P_FAN, 0)

    def _charge_on_enter(self):
        self._exp_write_pin(self.P_FAN, 1)
        self._charge_next_draw_ms = time.ticks_ms()
        self._charge_configured = False

        # Reset latches
        self._chg_error_latched = False
        self._chg_full_latched = False
        self._led_blink_on = False
        self._led_blink_next_ms = time.ticks_ms()

        # Try configure once on entry
        self._charge_configured = self._apply_bq_registers_for_charge()

    def _charge_on_exit(self):
        self._exp_write_pin(self.P_FAN, 0)
        # Disable charging when leaving charge page
        self._set_ce_enabled(False)
        # Also clear latches & LEDs 1-5
        self._chg_error_latched = False
        self._chg_full_latched = False
        for i in range(5):
            self._leds[i] = self.LED_OFF
        self._leds.write()

    def _discharge_on_enter(self):
        # Fan ON while discharging
        self._exp_write_pin(self.P_FAN, 1)

        # Ensure charger is disabled while discharging
        self._set_ce_enabled(False)

        # Start PWM on GPIO0 at current RATE (%)
        self._dsc_duty = int(1023 * self._dsc_pct / 100.0)
        try:
            if self._dsc_pwm is None:
                self._dsc_pwm = PWM(Pin(0))
            self._dsc_pwm.freq(self._dsc_freq_hz)
            self._dsc_pwm.duty(self._dsc_duty)
        except Exception as e:
            print("[DSC] PWM start failed:", e)

        self._dsc_next_draw_ms = time.ticks_ms()

    def _discharge_on_exit(self):
        # Fan OFF when leaving discharge
        self._exp_write_pin(self.P_FAN, 0)

        # Stop PWM (set duty=0, then deinit)
        try:
            if self._dsc_pwm is not None:
                self._dsc_pwm.duty(0)
                self._dsc_pwm.deinit()
        except Exception as e:
            print("[DSC] PWM stop failed:", e)
        finally:
            self._dsc_pwm = None

        # Clear LEDs 1-5 (optional)
        for i in range(5):
            self._leds[i] = self.LED_OFF
        self._leds.write()

    def _measure_on_enter(self):
        # Fan ON while measuring
        self._exp_write_pin(self.P_FAN, 1)

        # Ensure charger is disabled while applying load
        self._set_ce_enabled(False)

        # Ensure PWM object exists, duty=0
        try:
            if self._dsc_pwm is None:
                self._dsc_pwm = PWM(Pin(0))
            self._dsc_pwm.freq(self._dsc_freq_hz)
            self._dsc_pwm.duty(0)
        except Exception as e:
            print("[MEAS] PWM init failed:", e)

        self._meas_next_draw_ms = time.ticks_ms()

    def _measure_on_exit(self):
        # Fan OFF when leaving measure
        self._exp_write_pin(self.P_FAN, 0)

        # Disable load (duty 0) but keep PWM object alive for other pages if needed
        try:
            if self._dsc_pwm is not None:
                self._dsc_pwm.duty(0)
        except Exception:
            pass

    # -------------------------
    # LED logic
    # -------------------------
    def _blink_tick(self, now_ms):
        if time.ticks_diff(now_ms, self._led_blink_next_ms) >= 0:
            self._led_blink_on = not self._led_blink_on
            self._led_blink_next_ms = time.ticks_add(now_ms, self.LED_BLINK_MS)

    def _idle_led_update(self, now_ms):
        if time.ticks_diff(now_ms, self._idle_led_next_ms) < 0:
            return
        self._idle_led_next_ms = time.ticks_add(now_ms, self._idle_led_interval_ms)

        self._leds.fill((0, 0, 0))
        self._leds[self._idle_led_pos] = self._idle_led_color
        self._leds.write()

        self._idle_led_pos += self._idle_led_dir
        if self._idle_led_pos >= 4:
            self._idle_led_pos = 4
            self._idle_led_dir = -1
        elif self._idle_led_pos <= 0:
            self._idle_led_pos = 0
            self._idle_led_dir = 1

    def _led_apply_5(self, colors5):
        for i in range(5):
            self._leds[i] = colors5[i]
        self._leds.write()

    def _charge_led_update(self, now_ms, vbat, vmin, vmax, mode):
        """
        Implements your LED spec on LEDs 1-5 (indices 0..4).
        """
        self._blink_tick(now_ms)

        # If vbat below min at ANY time: disable charging, latch error, blink red/black
        if vbat is not None and vbat < (vmin - 1e-6):
            if not self._chg_error_latched:
                self._set_ce_enabled(False)  # CE HIGH
                self._chg_error_latched = True
            c = self.LED_ERR if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c, c, c, c, c])
            return

        # If charger says DONE: disable, latch full, blink all green
        if mode == "DONE":
            if not self._chg_full_latched:
                self._set_ce_enabled(False)  # CE HIGH
                self._chg_full_latched = True
            c = self.LED_CHG if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c, c, c, c, c])
            return

        # If latched, keep pattern until leaving charge page
        if self._chg_error_latched:
            c = self.LED_ERR if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c, c, c, c, c])
            return
        if self._chg_full_latched:
            c = self.LED_CHG if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c, c, c, c, c])
            return

        # If no battery: turn LEDs off (charger page will say NO BATT)
        if not self._battery_present(vbat):
            self._led_apply_5([self.LED_OFF]*5)
            return

        # PRE mode: blink LED1 (500ms on/off)
        if mode == "PRE":
            c1 = self.LED_CHG if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c1, self.LED_OFF, self.LED_OFF, self.LED_OFF, self.LED_OFF])
            return

        # CC/CV (fast charge): progress buckets by estimated voltage %
        pct = self._estimate_pct(vbat, vmin, vmax)
        if pct is None:
            c1 = self.LED_CHG if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c1, self.LED_OFF, self.LED_OFF, self.LED_OFF, self.LED_OFF])
            return

        solid = [self.LED_OFF]*5
        solid[0] = self.LED_CHG  # LED1 always on while charging

        if pct < 25.0:
            solid_on = 1
            blink_idx = 1
        elif pct < 50.0:
            solid_on = 2
            blink_idx = 2
        elif pct < 75.0:
            solid_on = 3
            blink_idx = 3
        else:
            solid_on = 4
            blink_idx = 4

        for i in range(solid_on):
            solid[i] = self.LED_CHG

        solid[blink_idx] = self.LED_CHG if self._led_blink_on else self.LED_OFF
        self._led_apply_5(solid)

    def _discharge_led_update(self, now_ms, vbat, v_full, v_empty):
        """
        Discharge LED pattern on LEDs 1-5 (indices 0..4), reverse of charging.
        0% discharge at v_full (ChgTo), 100% at v_empty (DscTo).
        Orange color.
        """
        self._blink_tick(now_ms)

        # If no battery voltage reading, turn off LEDs 1-5
        if vbat is None:
            self._led_apply_5([self.LED_OFF]*5)
            return

        # Clamp voltage
        if v_full <= v_empty:
            # invalid configuration; just show blinking error
            c = self.LED_ERR if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c, c, c, c, c])
            return

        # discharge_pct: 0 at full, 100 at empty
        if vbat >= v_full:
            discharge_pct = 0.0
        elif vbat <= v_empty:
            discharge_pct = 100.0
        else:
            discharge_pct = (v_full - vbat) / (v_full - v_empty) * 100.0

        # Buckets (reverse of charge):
        # 0..25%: LED1 solid, blink LED2
        # 25..50%: LEDs 1-2 solid, blink LED3
        # 50..75%: LEDs 1-3 solid, blink LED4
        # 75..100%: LEDs 1-4 solid, blink LED5
        solid = [self.LED_OFF]*5
        solid[0] = self.LED_DSC

        if discharge_pct < 25.0:
            solid_on = 1
            blink_idx = 1
        elif discharge_pct < 50.0:
            solid_on = 2
            blink_idx = 2
        elif discharge_pct < 75.0:
            solid_on = 3
            blink_idx = 3
        else:
            solid_on = 4
            blink_idx = 4

        for i in range(solid_on):
            solid[i] = self.LED_DSC

        solid[blink_idx] = self.LED_DSC if self._led_blink_on else self.LED_OFF
        self._led_apply_5(solid)

    # -------------------------
    # Render: STATUS page
    # -------------------------
    def _format_chg_11(self):
        st = self._bq_try_read_status()
        if st is None:
            # self._bq_wake_once()
            st = self._bq_try_read_status()
            if st is None:
                return "UNPOWERED"

        chrg_stat = st.get("chrg_stat", None) if isinstance(st, dict) else None
        pg = st.get("power_good", None) if isinstance(st, dict) else None
        vbus = st.get("vbus_stat", None) if isinstance(st, dict) else None

        ch_map = {0: "IDLE", 1: "PCHG", 2: "FCHG", 3: "DONE"}
        ch = ch_map.get(chrg_stat, "UNK")

        pg_s = "PG" if pg in (1, True) else "--"

        vb_map = {0: "UNK", 1: "USB", 2: "ADP", 3: "OTG"}
        vb = vb_map.get(vbus, "")

        out = "{} {} {}".format(ch, pg_s, vb) if vb else "{} {}".format(ch, pg_s)
        if len(out) > 11:
            out = out[:11]
        return out

    def _render_status(self, now_ms):
        if time.ticks_diff(now_ms, self._status_next_draw_ms) < 0:
            return
        self._status_next_draw_ms = time.ticks_add(now_ms, 1000)

        vin = iin = vbat = ibat = None
        try:
            vin = self.board.ina_sys.read_bus_voltage_v()
            iin = self.board.ina_sys.read_current_a()
        except Exception:
            pass
        try:
            vbat = self.board.ina_batt.read_bus_voltage_v()
            ibat = self.board.ina_batt.read_current_a()
        except Exception:
            pass

        t_mcu = None
        try:
            t_mcu = esp32.mcu_temperature()
        except Exception:
            pass

        t_bq = self._read_bq_temp_c()
        rpm = self._tach_compute_rpm()
        chg11 = self._format_chg_11()

        def f2(x): return "N/A" if x is None else "{:.2f}".format(float(x))
        def f1(x): return "N/A" if x is None else "{:.1f}".format(float(x))
        rpm_s = "{:.0f}".format(rpm) if rpm is not None else "0"

        self.oled.fill(0)
        self.oled.text("INP: {}/{}".format(f2(vin), f2(iin)), 0, 0, 1)
        self.oled.text("BAT: {}/{}".format(f2(vbat), f2(ibat)), 0, 12, 1)
        self.oled.text("TMP: {}/{}".format(f1(t_mcu), f1(t_bq)), 0, 24, 1)
        self.oled.text("CHG: {}".format(chg11), 0, 36, 1)
        self.oled.text("FAN: {}rpm".format(rpm_s), 0, 48, 1)
        self.oled.show()

    # -------------------------
    # Render: CHARGE page
    # -------------------------
    def _render_charge(self, now_ms):
        if time.ticks_diff(now_ms, self._charge_next_draw_ms) < 0:
            return
        self._charge_next_draw_ms = time.ticks_add(now_ms, 1000)

        pwr_ok = self._input_power_available()

        # keep charger enabled unless latched error/full, but only if input power is truly present
        if pwr_ok and (not self._chg_error_latched) and (not self._chg_full_latched):
            try: self.board.bq.set_hiz(False)
            except: pass
            try: self.board.bq.set_charge_enable_i2c(True)
            except: pass
            self._set_ce_enabled(True)  # CE low
        else:
            # if no power, ensure we don't accidentally enable charging
            self._set_ce_enabled(False)

        bq = getattr(self.board, "bq", None)
        st = None
        if bq is not None:
            try:
                st = bq.read_status()
            except Exception:
                st = None
                
        if not pwr_ok:
            self.oled.fill(0)
            self.oled.text("Plug in XT-30"[:16], 0, 16, 1)
            self.oled.text("or USB-C PD 12V"[:16], 0, 32, 1)
            self.oled.show()
            return
        
        vbat = self._read_vbat_v()
        vmin = float(self._get_item_value("ChgFrom", 2.5))
        vtar = float(self._get_item_value("ChgTo", 4.2))
        
        # Charger temperature
        t_bq = self._read_bq_temp_c()

        # Determine status/mode
        if st is None or not self._bq_is_powered():
            status = "UNPOWERED"
            mode = "UNP"
        else:
            mode = self._bq_charge_mode(st, vbat, vtar)
            if not self._battery_present(vbat):
                status = "NO BATT"
            elif vbat is not None and vbat < (vmin - 1e-6):
                status = "LOWV"
                mode = "LOWV"
            elif mode == "DONE":
                status = "CHARGED"
            elif mode in ("PRE", "CC", "CV"):
                status = "CHARGING"
            elif mode == "OFF":
                status = "IDLE"
            else:
                status = mode

        # Current read: only meaningful if charging
        ichg_a = None
        if mode in ("PRE", "CC", "CV") and bq is not None:
            try:
                ichg_ma = bq.read_charge_current_ma_adc()
                ichg_a = float(ichg_ma) / 1000.0
            except Exception:
                ichg_a = None

        def f2(x): return "N/A" if x is None else "{:.2f}".format(float(x))
        def f1c(x): return "N/A" if x is None else "{:.1f}C".format(float(x))

        # OLED lines:
        # 1) STATUS
        # 2) V: vbat -> target
        # 3) A: ichg / mode (or N/A / mode)
        # 4) T: temp
        self.oled.fill(0)
        self.oled.text("S: {}".format(status), 0, 0, 1)
        self.oled.text("V: {}->{}".format(f2(vbat), f2(vtar)), 0, 16, 1)

        if mode in ("PRE", "CC", "CV"):
            self.oled.text("A: {}/{}".format(f2(ichg_a), mode), 0, 32, 1)
        else:
            self.oled.text("A: N/A/{}".format(mode), 0, 32, 1)

        self.oled.text("T: {}".format(f1c(t_bq)), 0, 48, 1)
        self.oled.show()

        # LED logic (only when charge page active)
        if st is None or not self._bq_is_powered():
            # show error-ish (off is fine too); user asked error is red.
            # We'll blink red/black to indicate missing power if battery is present.
            self._blink_tick(now_ms)
            c = self.LED_ERR if self._led_blink_on else self.LED_OFF
            self._led_apply_5([c, c, c, c, c])
        else:
            # Apply specified patterns
            self._charge_led_update(now_ms, vbat, vmin, vtar, mode)

    def _measure_ir_run(self, pct):
        """
        Apply load at pct% PWM for 5 seconds, then compute R = dV / I.
        Stores result into 30% or 60% slot. Always disables load after.
        """
        duty = int(1023 * float(pct) / 100.0)

        # baseline (no load)
        try:
            if self._dsc_pwm is not None:
                self._dsc_pwm.duty(0)
        except Exception:
            pass
        time.sleep_ms(100)

        v0 = self._read_vbat_v()

        # apply load
        try:
            if self._dsc_pwm is None:
                self._dsc_pwm = PWM(Pin(0))
                self._dsc_pwm.freq(self._dsc_freq_hz)
            self._dsc_pwm.duty(duty)
        except Exception:
            return  # can't measure if PWM fails

        # wait for settling (5s)
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 5000:
            # optional: show a tiny "MEAS..." blink by forcing redraw
            self._meas_next_draw_ms = 0
            self._render_measure(time.ticks_ms())
            time.sleep_ms(50)

        # sample under load
        v1 = self._read_vbat_v()
        i1 = None
        try:
            i1 = float(self.board.ina_batt.read_current_a())
        except Exception:
            i1 = None

        # disable load
        try:
            if self._dsc_pwm is not None:
                self._dsc_pwm.duty(0)
        except Exception:
            pass

        # compute
        r = None
        dv = None
        if (v0 is not None) and (v1 is not None):
            dv = float(v0) - float(v1)

        i_abs = None
        if i1 is not None:
            i_abs = abs(float(i1))

        if (dv is not None) and (i_abs is not None) and (i_abs > 0.001):
            r = dv / i_abs

        print("[MEAS] pct=", pct, " duty=", duty)
        print("[MEAS] v0=", v0, " v1=", v1, " dv=", dv)
        print("[MEAS] i1=", i1, " i_abs=", i_abs)
        print("[MEAS] r=", r)

        # store into slot
        if pct == 30:
            self._meas_vdrop_30 = dv
            self._meas_i_30 = i1
            self._meas_v1_30 = v1
            self._meas_r_30 = r
        elif pct == 60:
            self._meas_vdrop_60 = dv
            self._meas_i_60 = i1
            self._meas_v1_60 = v1
            self._meas_r_60 = r

        self._meas_next_draw_ms = 0  # force redraw


    def _render_measure(self, now_ms):
        if time.ticks_diff(now_ms, self._meas_next_draw_ms) < 0:
            return
        self._meas_next_draw_ms = time.ticks_add(now_ms, 250)

        def f2(x): return None if x is None else "{:.2f}".format(float(x))
        def f3(x): return None if x is None else "{:.3f}".format(float(x))
        def milli(x): return None if x is None else int(round(float(x) * 1000))

        self.oled.fill(0)

        # 5 rows @ y = 0,12,24,36,48
        self.oled.text("MEASURE I.R.", 0, 0, 1)

        # Line 2: "<" inverted + label
        draw_inverted_text(self.oled, 0, 12, "<")
        self.oled.text(" Discharge@30%", 8, 12, 1)

        # Line 3: ">" inverted + label
        draw_inverted_text(self.oled, 0, 24, ">")
        self.oled.text(" Discharge@60%", 8, 24, 1)

        # Line 4: 30% result
        if self._meas_vdrop_30 is None or self._meas_i_30 is None:
            self.oled.text("30% not yet read", 0, 36, 1)
        else:
            r30 = self._meas_r_30
            if r30 is None:
                self.oled.text("{}/{}=---".format(
                    f2(self._meas_vdrop_30), f2(-self._meas_i_30)
                )[:16], 0, 36, 1)
            else:
                self.oled.text("{}mV/{}A=>{}mO".format(
                    milli(self._meas_vdrop_30), f2(-self._meas_i_30), milli(r30 - CIRCUIT_R_DISCHG_MOHM)
                )[:16], 0, 36, 1)
                
        if (self._meas_v1_30 is not None and self._meas_v1_60 is not None and
            self._meas_i_30 is not None and self._meas_i_60 is not None):
            i30 = abs(float(self._meas_i_30))
            i60 = abs(float(self._meas_i_60))
            di = i60 - i30

        # Line 5: 60% result
        if self._meas_vdrop_60 is None or self._meas_i_60 is None:
            self.oled.text("60% not yet read", 0, 48, 1)
        else:
            r60 = self._meas_r_60
            if r60 is None:
                self.oled.text("{}/{}=---".format(
                    f2(self._meas_vdrop_60), f2(-self._meas_i_60)
                )[:16], 0, 48, 1)
            else:
                self.oled.text("{}mV/{}A=>{}mO".format(
                    milli(self._meas_vdrop_60), f2(-self._meas_i_60), milli(r60 - CIRCUIT_R_DISCHG_MOHM)
                )[:16], 0, 48, 1)


        self.oled.show()


    def _render_discharge(self, now_ms):
        if time.ticks_diff(now_ms, self._dsc_next_draw_ms) < 0:
            return
        self._dsc_next_draw_ms = time.ticks_add(now_ms, 1000)

        vbat = self._read_vbat_v()
        v_target = float(self._get_item_value("DscTo", 3.70))

        # stop discharge PWM when empty
        if vbat is not None and vbat <= v_target:
            try:
                if self._dsc_pwm is not None:
                    self._dsc_pwm.duty(0)
            except Exception:
                pass

        # battery current (A -> mA integer)
        ibat_a = None
        try:
            ibat_a = float(self.board.ina_batt.read_current_a())
        except Exception:
            pass
        ibat_ma = None if ibat_a is None else int(round(ibat_a * 1000.0))

        # fan rpm
        rpm = self._tach_compute_rpm()
        rpm_i = int(round(rpm)) if rpm is not None else 0

        # external temp
        t_ext = None
        try:
            t_ext = read_ext_temp()
        except Exception:
            t_ext = None

        def f2(x): return "N/A" if x is None else "{:.2f}".format(float(x))
        def f0t(x): return "N/A" if x is None else "{:.0f}".format(float(x))
        def temp(x): return "LOW" if x is None else "{}C".format(int(x))
        self.oled.fill(0)

        # Row 1 (y=0): RATE label normal, VALUE inverted
        y = 0
        label = "RATE:"
        self.oled.text(label, 0, y, 1)
        val = " {}% ".format(int(self._dsc_pct))
        x_val = (len(label) + 1) * 8
        draw_inverted_text(self.oled, x_val, y, val)

        # Row 2 (y=12): VB -> target
        self.oled.text("VB: {}->{}".format(f2(vbat), f2(v_target)), 0, 12, 1)

        # Row 3 (y=24): current
        if ibat_ma is None:
            self.oled.text("I: N/A", 0, 24, 1)
        else:
            self.oled.text("I: {}mA".format(ibat_ma), 0, 24, 1)

        self.oled.text("FAN: {}rpm".format(rpm_i), 0, 36, 1)

        #self.oled.text("T:{}C".format(rpm_i, f0t(t_ext))[:16], 0, 48, 1)
        self.oled.text("TEMP:{}".format(temp(t_ext)), 0, 48, 1)
        
        self.oled.show()


    def _render_startup(self, line1, line2="", line3="", frac=None):
        self.oled.fill(0)
        #self.oled.text("bSafe V1 (C)2026", 0, 0, 1)
        draw_inverted_title(self.oled, "bSafe V1")
        self.oled.text(line1[:16], 0, 12, 1)
        if line2:
            self.oled.text(line2[:16], 0, 24, 1)
        if line3:
            self.oled.text(line3[:16], 0, 36, 1)
        if frac is not None:
            draw_progress_bar(self.oled, 0, 56, 128, 8, frac)
        self.oled.show()


    def _startup_sequence(self):
        """Boot splash + best-effort BQ wake, then gate on INA226 VIN."""
        dur = 2500

        def read_vin_v():
            try:
                return float(self.board.ina_sys.read_bus_voltage_v())
            except Exception:
                return None

        # One combined message for both states (<=16 chars recommended on OLED line)
        def render_progress(frac):
            self._render_startup("", "  INITIALIZING", frac=frac)

        st = self._bq_try_read_status()

        if st is None:
            # _bq_wake_once already waits ~2.5s and calls progress_cb during the wait
            try:
                self._bq_wake_once(progress_cb=render_progress)
            except Exception:
                pass
        else:
            # If BQ already responds, still show the same 2.5s progress bar
            t0 = time.ticks_ms()
            while True:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                if dt >= dur:
                    break
                render_progress(dt / dur)
                time.sleep_ms(50)

        VIN_DETECT_V = 0.5
        VIN_MIN_OK_V = 7.5

        # allow user override to enter UI without input power
        self._no_input_override = False

        while True:
            vin = read_vin_v()

            # OK + MENU held together => bypass power requirement
            try:
                ok_down = (self.pin_ok.value() == 0)
            except Exception:
                ok_down = False
            try:
                menu_down = (self._read_menu_level() == 0)
            except Exception:
                menu_down = False

            if ok_down and menu_down:
                self._no_input_override = True
                return True

            if vin is None or vin < VIN_DETECT_V:
                self._render_startup("", "Plug in XT-30", "or USB-C PD 12V")
                time.sleep_ms(250)
                continue

            if vin < VIN_MIN_OK_V:
                self._render_startup("", "FAN NOT AVAILABLE", "LIMITED POWER")
                time.sleep_ms(250)
                continue

            return True


    def _render_page_generic(self):
        self.oled.fill(0)
        title = self.page if self.page else "PAGE"
        x = max(0, (128 - len(title) * 8) // 2)
        self.oled.text(title, x, 24, 1)
        self.oled.text("MENU to return", 16, 48, 1)
        self.oled.show()

    def run(self, poll_ms=20):
        self.board.init_all()
        self.oled = ssd1306.SSD1306_I2C(128, 64, self.board.i2c, addr=0x3C)

        # If your driver supports rotate, keep this (your OLED is flipped 180°)
        try:
            self.oled.rotate(0)
        except Exception:
            pass

         # Clear LEDs at boot
        self._leds.fill((0, 0, 0))
        self._leds.write()

        # Startup screen: wake BQ w/ progress, then gate on input power
        self._startup_sequence()

        # Enter main menu
        self.mode = "MAIN"
        self.page = None
        self._last_page = None
        self._render_main()

        while True:
            now = time.ticks_ms()
            ev = self._poll_inputs()

            # PAGE mode
            if self.mode == "PAGE":
                # transition hooks
                if self._last_page != self.page:
                    # leaving old
                    if self._last_page == "Status":
                        self._status_on_exit()
                    if self._last_page == "Charge":
                        self._charge_on_exit()
                    if self._last_page == "Discharge":
                        self._discharge_on_exit()
                    if self._last_page == "Measure":
                        self._measure_on_exit()


                    # entering new
                    if self.page == "Status":
                        self._status_on_enter()
                    if self.page == "Charge":
                        self._charge_on_enter()
                    if self.page == "Discharge":
                        self._discharge_on_enter()
                    if self.page == "Measure":
                        self._measure_on_enter()

                    self._last_page = self.page

                # MENU exits page
                if ev["menu"]:
                    if self.page == "Status":
                        self._status_on_exit()
                    if self.page == "Charge":
                        self._charge_on_exit()
                    if self.page == "Discharge":
                        self._discharge_on_exit()
                    if self.page == "Measure":
                        self._measure_on_exit()

                    self.mode = "MAIN"
                    self.page = None
                    self._last_page = None
                    self._render_main()
                    time.sleep_ms(poll_ms)
                    continue

                # Render page
                if self.page == "Status":
                    # Idle LED animation while on Status page
                    self._idle_led_update(now)
                    self._render_status(now)
                elif self.page == "Charge":
                    # Charge page drives its own LEDs
                    self._render_charge(now)
                elif self.page == "Discharge":
                    # LEFT/RIGHT adjust RATE immediately (10..100 step 5)
                    if ev["left"] or ev["right"]:
                        step = 1 # percentage, formerly 5%
                        if ev["left"]:
                            self._dsc_pct -= step
                        if ev["right"]:
                            self._dsc_pct += step
                        if self._dsc_pct < 10:
                            self._dsc_pct = 10
                        if self._dsc_pct > 100:
                            self._dsc_pct = 100

                        self._dsc_duty = int(1023 * self._dsc_pct / 100.0)
                        vbat_now = self._read_vbat_v()
                        v_target = float(self._get_item_value("DscTo", 3.70))
                        
                        try:
                            if self._dsc_pwm is not None:
                                if vbat_now is not None and vbat_now <= v_target:
                                    self._dsc_pwm.duty(0)
                                else:
                                    self._dsc_pwm.duty(self._dsc_duty)
                        except Exception:
                            pass

                        # force an immediate redraw (don’t wait 1s)
                        self._dsc_next_draw_ms = now

                    self._render_discharge(now)

                    # Discharge LED pattern (reverse of charge)
                    vbat = self._read_vbat_v()
                    v_full = float(self._get_item_value("ChgTo", 4.2))
                    v_empty = float(self._get_item_value("DscTo", 3.7))
                    self._discharge_led_update(now, vbat, v_full, v_empty)

                elif self.page == "Measure":
                    # LEFT: 30% load test, RIGHT: 60% load test
                    if ev["left"]:
                        print("[MEAS] LEFT pressed -> 30% run")
                        self._measure_ir_run(30)
                    if ev["right"]:
                        print("[MEAS] RIGHT pressed -> 60% run")
                        self._measure_ir_run(60)


                    # Render MEASURE page
                    self._render_measure(now)

                else:
                    # Other pages: idle LED animation
                    self._idle_led_update(now)
                    self._render_page_generic()

                time.sleep_ms(poll_ms)
                continue

            # MAIN / EDIT mode: idle LED animation
            self._idle_led_update(now)

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

app = App()
app.run()
