# drivers/ina226.py
#
# INA226 current/voltage/power monitor
# Focus: precision measurement + alerts only for overvoltage/overcurrent.
#
# Notes:
# - Bus voltage register LSB = 1.25 mV
# - Shunt voltage register LSB = 2.5 uV
# - Calibration register sets current LSB via:
#     CAL = trunc(0.00512 / (current_lsb * Rshunt))
#
# We'll choose current_lsb so max current has headroom while keeping good resolution.

from micropython import const
import time

# Registers
REG_CONFIG        = const(0x00)
REG_SHUNT_V       = const(0x01)
REG_BUS_V         = const(0x02)
REG_POWER         = const(0x03)
REG_CURRENT       = const(0x04)
REG_CALIBRATION   = const(0x05)
REG_MASK_ENABLE   = const(0x06)
REG_ALERT_LIMIT   = const(0x07)
REG_MANUF_ID      = const(0xFE)
REG_DIE_ID        = const(0xFF)

# Config bitfields
# AVG (11..9), VBUSCT (8..6), VSHCT (5..3), MODE (2..0)
_AVG_MAP = {
    1:    0b000,
    4:    0b001,
    16:   0b010,
    64:   0b011,
    128:  0b100,
    256:  0b101,
    512:  0b110,
    1024: 0b111,
}

_CT_MAP_US = {
    140:  0b000,
    204:  0b001,
    332:  0b010,
    588:  0b011,
    1100: 0b100,
    2116: 0b101,
    4156: 0b110,
    8244: 0b111,
}

# Modes
MODE_POWER_DOWN           = const(0b000)
MODE_SHUNT_TRIGGERED      = const(0b001)
MODE_BUS_TRIGGERED        = const(0b010)
MODE_SHUNT_BUS_TRIGGERED  = const(0b011)
MODE_SHUNT_CONT           = const(0b101)
MODE_BUS_CONT             = const(0b110)
MODE_SHUNT_BUS_CONT       = const(0b111)

# Mask/Enable bits
ME_SOL   = const(1 << 15)  # Shunt over-voltage
ME_SUL   = const(1 << 14)  # Shunt under-voltage
ME_BOL   = const(1 << 13)  # Bus over-voltage
ME_BUL   = const(1 << 12)  # Bus under-voltage
ME_POL   = const(1 << 11)  # Power over-limit
ME_CNVR  = const(1 << 10)  # Conversion ready
ME_AFF   = const(1 << 4)   # Alert function flag (RO)
ME_CVRF  = const(1 << 3)   # Conversion ready flag (RO)
ME_OVF   = const(1 << 2)   # Math overflow flag (RO)
ME_APOL  = const(1 << 1)   # Alert polarity (1=active high)
ME_LEN   = const(1 << 0)   # Latch enable

# LSBs
BUS_LSB_V   = 1.25e-3   # V
SHUNT_LSB_V = 2.5e-6    # V

class INA226:
    def __init__(self, i2c, addr, r_shunt_ohms):
        self.i2c = i2c
        self.addr = addr
        self.r_shunt = float(r_shunt_ohms)

        self.current_lsb = None  # A/bit
        self.power_lsb = None    # W/bit

    # -------------------------
    # Low-level I2C
    # -------------------------
    def _wr16(self, reg, value):
        value &= 0xFFFF
        self.i2c.writeto_mem(self.addr, reg, bytes([(value >> 8) & 0xFF, value & 0xFF]))

    def _rd16(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        return (b[0] << 8) | b[1]

    @staticmethod
    def _to_signed16(v):
        return v - 65536 if v & 0x8000 else v

    # -------------------------
    # Initialization / config
    # -------------------------
    def reset(self):
        # Set reset bit (bit 15)
        self._wr16(REG_CONFIG, 1 << 15)
        time.sleep_ms(2)

    def configure_precision(self, avg_samples=16, vbus_ct_us=1100, vsh_ct_us=1100, mode=MODE_SHUNT_BUS_CONT):
        if avg_samples not in _AVG_MAP:
            raise ValueError("avg_samples must be one of {}".format(sorted(_AVG_MAP.keys())))
        if vbus_ct_us not in _CT_MAP_US:
            raise ValueError("vbus_ct_us must be one of {}".format(sorted(_CT_MAP_US.keys())))
        if vsh_ct_us not in _CT_MAP_US:
            raise ValueError("vsh_ct_us must be one of {}".format(sorted(_CT_MAP_US.keys())))

        cfg = (_AVG_MAP[avg_samples] << 9) | (_CT_MAP_US[vbus_ct_us] << 6) | (_CT_MAP_US[vsh_ct_us] << 3) | (mode & 0x7)
        self._wr16(REG_CONFIG, cfg)

    def calibrate_for_max_current(self, max_expected_current_a):
        """
        Picks a current LSB based on max_expected_current_a for good resolution + headroom,
        then writes CAL register accordingly.
        """
        max_i = float(max_expected_current_a)
        if max_i <= 0:
            raise ValueError("max_expected_current_a must be > 0")

        # INA226 current register is signed 16-bit. To avoid clipping:
        # use ~80% of full-scale as max expected.
        # full-scale counts ~ 32767.
        # current_lsb ≈ max_i / 26214 (0.8*32767)
        self.current_lsb = max_i / 26214.0

        # Quantize current_lsb to a "nice" value to stabilize calculations/logging.
        # We'll round to 1e-6 A steps (1 uA) or bigger depending on range.
        step = 1e-6
        if self.current_lsb > 1e-3:
            step = 1e-5
        if self.current_lsb > 1e-2:
            step = 1e-4
        self.current_lsb = round(self.current_lsb / step) * step

        # Calibration register
        cal = int(0.00512 / (self.current_lsb * self.r_shunt))
        if cal <= 0 or cal > 0xFFFF:
            raise ValueError("calibration out of range (cal={})".format(cal))

        self._wr16(REG_CALIBRATION, cal)

        # Power LSB is 25 * current_lsb (datasheet)
        self.power_lsb = 25.0 * self.current_lsb

    # -------------------------
    # Reading measurements
    # -------------------------
    def read_bus_voltage_v(self):
        raw = self._rd16(REG_BUS_V)
        return raw * BUS_LSB_V

    def read_shunt_voltage_v(self):
        raw = self._to_signed16(self._rd16(REG_SHUNT_V))
        return raw * SHUNT_LSB_V

    def read_current_a(self):
        if self.current_lsb is None:
            raise RuntimeError("INA226 not calibrated; call calibrate_for_max_current()")
        raw = self._to_signed16(self._rd16(REG_CURRENT))
        return raw * self.current_lsb

    def read_power_w(self):
        if self.power_lsb is None:
            raise RuntimeError("INA226 not calibrated; call calibrate_for_max_current()")
        raw = self._rd16(REG_POWER)
        return raw * self.power_lsb

    def read_all(self):
        """
        Convenience read. Minimizes I2C transactions by reading each register once.
        """
        bus = self.read_bus_voltage_v()
        shunt = self.read_shunt_voltage_v()
        cur = self.read_current_a()
        pwr = self.read_power_w()
        return bus, shunt, cur, pwr

    # -------------------------
    # Alerts
    # -------------------------
    def clear_alert_latch(self):
        """
        Reading Mask/Enable clears AFFF in some modes; safest is to read it.
        """
        _ = self._rd16(REG_MASK_ENABLE)

    def _write_mask_enable(self, *, latch=True, active_low=True, enable_bits=0):
        mask = enable_bits & 0xFC00  # only alert function enables in top bits
        if latch:
            mask |= ME_LEN
        # APOL=1 is active-high. Your inputs are pulled up and devices pull down -> active-low desired.
        if not active_low:
            mask |= ME_APOL
        self._wr16(REG_MASK_ENABLE, mask)

    def set_bus_overvoltage_alert(self, vbus_limit_v, *, latch=True, active_low=True):
        """
        Bus over-voltage uses BUS voltage register scale (1.25mV LSB) in ALERT_LIMIT.
        """
        limit = int(vbus_limit_v / BUS_LSB_V)
        limit = max(0, min(0xFFFF, limit))
        self._wr16(REG_ALERT_LIMIT, limit)
        self._write_mask_enable(latch=latch, active_low=active_low, enable_bits=ME_BOL)

    def set_bus_undervoltage_alert(self, vbus_limit_v, *, latch=True, active_low=True):
        limit = int(vbus_limit_v / BUS_LSB_V)
        limit = max(0, min(0xFFFF, limit))
        self._wr16(REG_ALERT_LIMIT, limit)
        self._write_mask_enable(latch=latch, active_low=active_low, enable_bits=ME_BUL)

    def set_shunt_overcurrent_alert(self, current_limit_a, *, latch=True, active_low=True):
        """
        Shunt over-voltage threshold corresponds to shunt voltage threshold.
        For over-current: Vshunt = I * Rshunt
        ALERT_LIMIT is in units of 2.5uV, signed comparisons handled internally.
        SOL triggers when shunt voltage > limit.
        """
        v_limit = float(current_limit_a) * self.r_shunt
        raw = int(v_limit / SHUNT_LSB_V)
        raw = max(0, min(0x7FFF, raw))  # positive limit for SOL
        self._wr16(REG_ALERT_LIMIT, raw)
        self._write_mask_enable(latch=latch, active_low=active_low, enable_bits=ME_SOL)

    def set_shunt_undercurrent_alert(self, current_limit_a, *, latch=True, active_low=True):
        """
        Under-current here means shunt voltage < limit. Useful for "not drawing current" detection.
        Uses SUL.
        """
        v_limit = float(current_limit_a) * self.r_shunt
        raw = int(v_limit / SHUNT_LSB_V)
        raw = max(0, min(0x7FFF, raw))
        self._wr16(REG_ALERT_LIMIT, raw)
        self._write_mask_enable(latch=latch, active_low=active_low, enable_bits=ME_SUL)

    # -------------------------
    # Identity / debug
    # -------------------------
    def read_ids(self):
        return self._rd16(REG_MANUF_ID), self._rd16(REG_DIE_ID)
