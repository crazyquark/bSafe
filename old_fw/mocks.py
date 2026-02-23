# mocks.py
#
# Mock devices so you can run UI without fab hardware.
# You can edit values at runtime from REPL.

import time

class MockINA226:
    def __init__(self, name="ina"):
        self.name = name
        self.bus_v = 0.0
        self.shunt_v = 0.0
        self.current_a = 0.0
        self.power_w = 0.0

    def reset(self): pass
    def configure_precision(self, *args, **kwargs): pass
    def calibrate_for_max_current(self, *args, **kwargs): pass

    def read_bus_voltage_v(self): return float(self.bus_v)
    def read_shunt_voltage_v(self): return float(self.shunt_v)
    def read_current_a(self): return float(self.current_a)
    def read_power_w(self): return float(self.power_w)

    def set_bus_overvoltage_alert(self, *args, **kwargs): pass
    def set_shunt_overcurrent_alert(self, *args, **kwargs): pass
    def clear_alert_latch(self): pass


class MockBQ25895:
    def __init__(self):
        self.charge_enable_i2c = False
        self.hiz = False
        self.vsys_v = 0.0
        self.vbus_v = 0.0
        self.vbus_good = False
        self.ts_percent = 21.0
        self.ichg_adc_ma = 0

        # status simulation
        # chrg_stat: 0=not_charging,1=precharge,2=fast_charge,3=done
        self.chrg_stat = 0
        self.power_good = False

    def set_charge_enable_i2c(self, enabled: bool):
        self.charge_enable_i2c = bool(enabled)

    def set_hiz(self, enabled: bool):
        self.hiz = bool(enabled)

    def watchdog_kick(self): pass

    def adc_start_oneshot(self): pass
    def adc_set_continuous(self, enabled: bool): pass

    def read_vsys_v(self): return float(self.vsys_v)
    def read_vbus_v(self): return float(self.vbus_v)
    def read_vbus_good(self): return bool(self.vbus_good)
    def read_ts_percent_regn(self): return float(self.ts_percent)
    def read_charge_current_ma_adc(self): return int(self.ichg_adc_ma)

    def read_status(self):
        # Return compatible dict with real driver
        return {
            "raw": 0,
            "vbus_stat": 0,
            "chrg_stat": int(self.chrg_stat),
            "power_good": bool(self.power_good),
            "sdp_stat": False,
            "vsys_stat": False,
        }

    def read_faults(self):
        return {
            "raw": 0,
            "watchdog_fault": False,
            "boost_fault": False,
            "chrg_fault": 0,
            "bat_fault": False,
            "ntc_fault": 0,
        }

    @staticmethod
    def chrg_stat_label(chrg_stat: int) -> str:
        if chrg_stat == 0: return "not_charging"
        if chrg_stat == 1: return "precharge"
        if chrg_stat == 2: return "fast_charge"
        if chrg_stat == 3: return "done"
        return "unknown"


class MockInputs:
    """
    Emulates active-low inputs with edge injection.
    Buttons/lines default to pulled-up (not asserted).
    """
    def __init__(self):
        self._state = {
            "MENU": False,
            "LEFT": False,
            "RIGHT": False,
            "OK": False,      # optional synthetic OK
            "STAT": False,
            "CHG_INT": False,
            "ALERT1": False,
            "ALERT2": False,
        }
        self._edge_queue = []  # list of events: ("MENU", True)

    def press(self, name: str, ms: int = 80):
        """
        Generates a press edge then auto-release after ms.
        """
        name = name.upper()
        self._enqueue_edge(name, True)
        # schedule release in a crude but simple way
        t_release = time.ticks_add(time.ticks_ms(), ms)
        self._edge_queue.append(("__release__", name, t_release))

    def set_line(self, name: str, asserted: bool):
        """
        For lines like STAT/ALERT: set level (asserted True means pulled low).
        """
        name = name.upper()
        cur = self._state.get(name, False)
        if cur != bool(asserted):
            self._enqueue_edge(name, bool(asserted))

    def _enqueue_edge(self, name, new_val):
        self._state[name] = bool(new_val)
        self._edge_queue.append(("edge", name, bool(new_val)))

    def poll_edges(self):
        """
        Returns list of edge events since last poll:
          [("MENU", True), ("MENU", False), ...]
        Also processes scheduled releases.
        """
        now = time.ticks_ms()
        out = []

        # Process scheduled releases
        keep = []
        for item in self._edge_queue:
            if item[0] == "__release__":
                _, name, t_release = item
                if time.ticks_diff(now, t_release) >= 0:
                    # release
                    if self._state.get(name, False):
                        out.append((name, False))
                        self._state[name] = False
                else:
                    keep.append(item)
            elif item[0] == "edge":
                _, name, val = item
                out.append((name, val))
            else:
                keep.append(item)

        self._edge_queue = keep
        return out

    def is_asserted(self, name: str) -> bool:
        return bool(self._state.get(name.upper(), False))
