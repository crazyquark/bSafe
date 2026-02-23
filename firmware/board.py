from machine import Pin, I2C
import time

import config

# Real drivers (if present on I2C)
from drivers.tca9535 import TCA9535
from drivers.ina226 import INA226
from drivers.bq25895 import BQ25895

class Board:
    def __init__(self):
        self.i2c = None
        
        self.btn_ok = Pin(9, Pin.IN)  # GPIO9, active-low

        # Devices
        self.exp = None
        self.ina_batt = None
        self.ina_sys = None
        self.bq = None
        self.has_exp = False
        self.has_ina_batt = False
        self.has_ina_sys = False
        self.has_bq = False

        # Inputs
        self.pin_io_int = None

        # cached
        self.in0 = 0xFF
        self.in1 = 0xFF
        self._btn_prev = {"menu": False, "left": False, "right": False, "ok": False}

    def init_i2c(self):
        self.i2c = I2C(
            0,
            scl=Pin(config.I2C_SCL),
            sda=Pin(config.I2C_SDA),
            freq=config.I2C_FREQ,
        )
        return self.i2c

    def _scan(self):
        try:
            return set(self.i2c.scan())
        except Exception:
            return set()

    # -------------------------
    # Expander (optional)
    # -------------------------
    def init_expander(self):
        addrs = self._scan()
        if config.TCA9535_ADDR not in addrs:
            print("WARN: TCA9535 not found at 0x{:02X}".format(config.TCA9535_ADDR))
            self.exp = None
            self.has_exp = False
            # self.pin_io_int = Pin(1, Pin.IN)
            return False

        try:
            self.exp = TCA9535(self.i2c, config.TCA9535_ADDR)
            self.exp.configure(config.TCA_DIR0, config.TCA_DIR1)

            # Default safe output states:
            # /CE HIGH = disabled by default (active-low)
            self._set_pin(config.P_CE, 1)
            self._set_pin(config.P_QON, 0)
            self._set_pin(config.P_FAN, 0)

            self.in0, self.in1 = self.exp.read_all()
            self.has_exp = True
        except Exception as e:
            print("WARN: TCA9535 init failed.", e)
            self.exp = None
            self.has_exp = False
        # self.pin_io_int = Pin(1, Pin.IN)
        return self.has_exp

    def _set_pin(self, pin, value):
        if self.exp is None:
            return
        self.exp.write_pin(pin, 1 if value else 0)

    def int_asserted(self) -> bool:
        return self.pin_io_int is not None and self.pin_io_int.value() == 0
    
    def _refresh_exp_inputs(self):
        if self.exp is None:
            return
        try:
            self.in0, self.in1 = self.exp.read_all()
        except:
            # keep old values on read failure
            pass

    def _pin_is_low(self, pin) -> bool:
        """
        Reads cached expander inputs and returns True if that expander pin is LOW.
        This matches your hardware: 10K pull-ups, button pulls to GND when pressed.
        """
        # If your TCA9535 driver uses 0..15 pin numbering, this works:
        if pin < 8:
            return ((self.in0 >> pin) & 1) == 0
        else:
            b = pin - 8
            return ((self.in1 >> b) & 1) == 0

    def poll_button_edges(self):
        # If expander present, refresh its cached inputs (polling is simplest/reliable)
        if getattr(self, "has_exp", False) and self.exp is not None:
            self._refresh_exp_inputs()

            menu_pressed  = self._exp_pin_low(config.P_BTN1)  # BTN1 = MENU
            left_pressed  = self._exp_pin_low(config.P_BTN2)  # BTN2 = LEFT
            right_pressed = self._exp_pin_low(config.P_BTN3)  # BTN3 = RIGHT

        # OK is ALWAYS GPIO9 (BOOT), active-low
        ok_pressed = (self.btn_ok.value() == 0)

        pressed = {
            "menu": menu_pressed,
            "left": left_pressed,
            "right": right_pressed,
            "ok": ok_pressed,
        }

        # Edge detect: only emit press events
        edges = {}
        for k, v in pressed.items():
            prev = self._btn_prev.get(k, False)
            edges[k] = (v and not prev)
            self._btn_prev[k] = v

        return edges


    # -------------------------
    # INA226 (optional)
    # -------------------------
    def init_ina226(self, max_current_a=5.0):
        addrs = self._scan()

        if config.INA226_ADDR_BATT in addrs:
            try:
                self.ina_batt = INA226(self.i2c, config.INA226_ADDR_BATT, config.SHUNT_RESISTANCE_OHMS)
                self.ina_batt.reset()
                self.ina_batt.configure_precision(config.INA_AVG_SAMPLES, config.INA_VBUS_CT_US, config.INA_VSH_CT_US)
                self.ina_batt.calibrate_for_max_current(max_current_a)
                self.has_ina_batt = True
            except Exception as e:
                print("WARN: INA226 batt init failed.", e)
                self.has_ina_batt = False
        else:
            print("WARN: INA226 batt not found at 0x{:02X}.".format(config.INA226_ADDR_BATT))
            self.has_ina_batt = False

        if config.INA226_ADDR_SYS in addrs:
            try:
                self.ina_sys = INA226(self.i2c, config.INA226_ADDR_SYS, config.SHUNT_RESISTANCE_OHMS)
                self.ina_sys.reset()
                self.ina_sys.configure_precision(config.INA_AVG_SAMPLES, config.INA_VBUS_CT_US, config.INA_VSH_CT_US)
                self.ina_sys.calibrate_for_max_current(max_current_a)
                self.has_ina_sys = True
            except Exception as e:
                print("WARN: INA226 sys init failed.", e)
                self.has_ina_sys = False
        else:
            print("WARN: INA226 sys not found at 0x{:02X}.".format(config.INA226_ADDR_SYS))
            self.has_ina_sys = False

    # -------------------------
    # BQ25895 (optional)
    # -------------------------
    def init_bq25895(self):
        addrs = self._scan()
        if config.BQ25895_ADDR in addrs:
            try:
                self.bq = BQ25895(self.i2c, config.BQ25895_ADDR)
                self.bq.set_charge_enable_i2c(False)
                self.bq.set_hiz(False)
                self.has_bq = True
            except Exception as e:
                print("WARN: BQ25895 init failed.", e)
                self.has_bq = False
        else:
            print("WARN: BQ25895 not found at 0x{:02X}.".format(config.BQ25895_ADDR))
            self.has_bq = False

    def init_all(self, max_current_a=5.0):
        self.init_i2c()
        self.init_expander()
        self.init_ina226(max_current_a=max_current_a)
        self.init_bq25895()

    # -------------------------
    # Inputs unified API
    # -------------------------
    def poll_button_edges(self):
        ev = {"menu": False, "left": False, "right": False, "ok": False}
        edges = self.inputs.poll_edges()
        for name, pressed in edges:
            if not pressed:
                continue
            if name == "MENU":
                ev["menu"] = True
            elif name == "LEFT":
                ev["left"] = True
            elif name == "RIGHT":
                ev["right"] = True
            elif name == "OK":
                ev["ok"] = True

        return ev

    # -------------------------
    # Charger enable helper
    # -------------------------
    def set_ce_enabled(self, enabled: bool):
        """
        Control /CE (active-low) if expander exists.
        No-op when expander is absent.
        """
        if self.exp is None:
            return
        self._set_pin(config.P_CE, 0 if enabled else 1)

    # -------------------------
    # Backwards-compatibility helpers (older UI harness)
    # -------------------------
    def read_inputs(self):
        if self.exp is None:
            self.in0, self.in1 = 0xFF, 0xFF
            return self.in0, self.in1
        self.in0, self.in1 = self.exp.read_all()
        return self.in0, self.in1

    def read_changes(self):
        if self.exp is None:
            return 0, 0, 0xFF, 0xFF
        chg0, chg1, new0, new1 = self.exp.read_changes()
        self.in0, self.in1 = new0, new1
        return chg0, chg1, new0, new1
