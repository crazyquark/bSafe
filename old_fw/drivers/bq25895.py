# drivers/bq25895.py
#
# TI BQ25895 1-cell switch-mode charger / power-path
# Address: 0x6A
#
# This driver is register-level and intentionally avoids board-specific pin control.
# /CE and QON are on your TCA9535 expander and should be handled by board logic.
#
# Datasheet register map used for key fields:
# - REG00 IINLIM (100mA + n*50mA)
# - REG02 ADC control (CONV_START / CONV_RATE)
# - REG03 CHG_CONFIG, SYS_MIN, watchdog reset
# - REG04 ICHG (n*64mA)
# - REG05 IPRECHG/ITERM (64mA + n*64mA)
# - REG06 VREG (3840mV + n*16mV)
# - REG07 watchdog/timers/stat disable
# - REG0B status
# - REG0C faults
# - REG0F/10/11/12 ADC readouts (VSYS, TS%, VBUS, ICHG)
#
# Notes:
# - Status REG0B CHRG_STAT encodes: 00 not charging, 01 precharge, 10 fast, 11 done
# - REG10 TS% is "percentage of REGN" with offset 21% and range up to 80%
#
# References:
# - TI BQ25895 datasheet rev C (Oct 2022), register tables around sections 8.4.x

from micropython import const
import time

# I2C address
DEFAULT_ADDR = const(0x6A)

# Registers
REG00 = const(0x00)
REG01 = const(0x01)
REG02 = const(0x02)
REG03 = const(0x03)
REG04 = const(0x04)
REG05 = const(0x05)
REG06 = const(0x06)
REG07 = const(0x07)
REG08 = const(0x08)
REG09 = const(0x09)
REG0A = const(0x0A)
REG0B = const(0x0B)  # status
REG0C = const(0x0C)  # faults
REG0D = const(0x0D)
REG0E = const(0x0E)
REG0F = const(0x0F)  # VSYS ADC
REG10 = const(0x10)  # TS% ADC
REG11 = const(0x11)  # VBUS ADC + VBUS_GD
REG12 = const(0x12)  # ICHG ADC
REG13 = const(0x13)
REG14 = const(0x14)

# REG00
B_EN_HIZ  = const(1 << 7)
B_EN_ILIM = const(1 << 6)
M_IINLIM  = const(0x3F)       # bits [5:0]

# REG02
B_CONV_START = const(1 << 7)
B_CONV_RATE  = const(1 << 6)  # 0 one-shot, 1 continuous 1s
# other bits exist but are not needed yet for core control

# REG03
B_BAT_LOADEN = const(1 << 7)
B_WD_RST     = const(1 << 6)
B_OTG_CONFIG = const(1 << 5)
B_CHG_CONFIG = const(1 << 4)
M_SYS_MIN    = const(0b111 << 1)

# REG04
B_EN_PUMPX = const(1 << 7)
M_ICHG     = const(0x7F)       # bits [6:0], 64mA step

# REG05
M_IPRECHG = const(0x0F << 4)   # [7:4]
M_ITERM   = const(0x0F)        # [3:0]

# REG06
M_VREG = const(0x3F << 2)      # [7:2], 16mV step, offset 3840mV

# REG07 (subset we care about now)
B_EN_TERM   = const(1 << 7)
B_STAT_DIS  = const(1 << 6)
M_WATCHDOG  = const(0b11 << 4)  # [5:4]
B_EN_TIMER  = const(1 << 3)
M_CHG_TIMER = const(0b11 << 1)  # [2:1]

# REG0B status fields
M_VBUS_STAT = const(0b111 << 5)  # [7:5]
M_CHRG_STAT = const(0b11  << 3)  # [4:3]
B_PG_STAT   = const(1 << 2)
B_SDP_STAT  = const(1 << 1)
B_VSYS_STAT = const(1 << 0)

# REG0C fault fields
B_WATCHDOG_FAULT = const(1 << 7)
B_BOOST_FAULT    = const(1 << 6)
M_CHRG_FAULT     = const(0b11 << 4)  # [5:4]
B_BAT_FAULT      = const(1 << 3)
M_NTC_FAULT      = const(0b111)      # [2:0]

class BQ25895:
    def __init__(self, i2c, addr=DEFAULT_ADDR):
        self.i2c = i2c
        self.addr = addr

    # -------------------------
    # Low-level I2C
    # -------------------------
    def _wr(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _rd(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _update_bits(self, reg, mask, value):
        cur = self._rd(reg)
        new = (cur & ~mask) | (value & mask)
        if new != cur:
            self._wr(reg, new)
        return new

    # -------------------------
    # Basic control / keepalive
    # -------------------------
    def watchdog_kick(self):
        # Set WD_RST=1 then it self-clears after timer reset
        self._update_bits(REG03, B_WD_RST, B_WD_RST)

    def set_charge_enable_i2c(self, enabled: bool):
        """
        Controls CHG_CONFIG bit only.
        NOTE: Real charging also requires /CE pin LOW externally.
        """
        self._update_bits(REG03, B_CHG_CONFIG, B_CHG_CONFIG if enabled else 0)

    def set_hiz(self, enabled: bool):
        self._update_bits(REG00, B_EN_HIZ, B_EN_HIZ if enabled else 0)

    # -------------------------
    # Programmable limits
    # -------------------------
    @staticmethod
    def _clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def set_input_current_limit_ma(self, ma: int):
        """
        REG00 IINLIM: Offset 100mA, step 50mA, range 100..3250mA
        """
        ma = int(ma)
        ma = self._clamp(ma, 100, 3250)
        code = (ma - 100) // 50
        code = self._clamp(code, 0, 0x3F)
        self._update_bits(REG00, M_IINLIM, code)

    def set_fast_charge_current_ma(self, ma: int):
        """
        REG04 ICHG: step 64mA, range 0..5056mA (clamped by IC)
        Note: ICHG=0 disables charge.
        """
        ma = int(ma)
        ma = self._clamp(ma, 0, 5056)
        code = ma // 64
        code = self._clamp(code, 0, 0x7F)
        self._update_bits(REG04, M_ICHG, code)

    def set_precharge_current_ma(self, ma: int):
        """
        REG05 IPRECHG: offset 64mA, step 64mA, range 64..1024mA
        """
        ma = int(ma)
        ma = self._clamp(ma, 64, 1024)
        code = (ma - 64) // 64
        code = self._clamp(code, 0, 0x0F)
        self._update_bits(REG05, M_IPRECHG, code << 4)

    def set_termination_current_ma(self, ma: int):
        """
        REG05 ITERM: offset 64mA, step 64mA, range 64..1024mA
        """
        ma = int(ma)
        ma = self._clamp(ma, 64, 1024)
        code = (ma - 64) // 64
        code = self._clamp(code, 0, 0x0F)
        self._update_bits(REG05, M_ITERM, code)

    def set_charge_voltage_mv(self, mv: int):
        """
        REG06 VREG: offset 3840mV, step 16mV, range 3840..4608mV
        """
        mv = int(mv)
        mv = self._clamp(mv, 3840, 4608)
        code = (mv - 3840) // 16
        code = self._clamp(code, 0, 0x3F)
        self._update_bits(REG06, M_VREG, code << 2)

    # -------------------------
    # ADC control + reads
    # -------------------------
    def adc_start_oneshot(self):
        """
        One-shot ADC: CONV_RATE=0, then pulse CONV_START=1.
        Bit stays high during conversion.
        """
        self._update_bits(REG02, B_CONV_RATE, 0)
        self._update_bits(REG02, B_CONV_START, B_CONV_START)

    def adc_set_continuous(self, enabled: bool):
        """
        Continuous ADC: CONV_RATE=1. CONV_START becomes read-only in this mode.
        """
        self._update_bits(REG02, B_CONV_RATE, B_CONV_RATE if enabled else 0)

    def read_vsys_v(self):
        """
        REG0F: VSYS ADC
        Offset: 2.304V, LSB: 20mV, 7-bit value in bits [6:0].
        """
        raw = self._rd(REG0F) & 0x7F
        return 2.304 + (raw * 0.020)

    def read_vbus_v(self):
        """
        REG11: VBUS ADC
        Offset: 2.6V, LSB: 100mV, 7-bit in bits [6:0]
        """
        raw = self._rd(REG11) & 0x7F
        return 2.6 + (raw * 0.100)

    def read_vbus_good(self) -> bool:
        return bool(self._rd(REG11) & 0x80)

    def read_ts_percent_regn(self):
        """
        REG10: TS voltage as % of REGN.
        Offset: 21% (raw=0), range up to ~80% (raw=127).
        LSB ~0.465%
        Returns percent (0..100), best-effort.
        """
        raw = self._rd(REG10) & 0x7F
        return 21.0 + (raw * 0.465)

    def read_charge_current_ma_adc(self):
        """
        REG12: ICHGR ADC (charge current only), LSB 50mA, 7-bit in [6:0]
        Returns 0 for VBAT < VBATSHORT per datasheet.
        """
        raw = self._rd(REG12) & 0x7F
        return raw * 50

    # -------------------------
    # Status + faults
    # -------------------------
    def read_status(self):
        """
        Returns a dict with decoded REG0B fields.
        """
        v = self._rd(REG0B)

        vbus_stat = (v & M_VBUS_STAT) >> 5
        chrg_stat = (v & M_CHRG_STAT) >> 3

        return {
            "raw": v,
            "vbus_stat": vbus_stat,
            "chrg_stat": chrg_stat,
            "power_good": bool(v & B_PG_STAT),
            "sdp_stat": bool(v & B_SDP_STAT),
            "vsys_stat": bool(v & B_VSYS_STAT),
        }

    def read_faults(self):
        """
        Returns a dict with decoded REG0C fields.
        """
        v = self._rd(REG0C)
        chrg_fault = (v & M_CHRG_FAULT) >> 4
        ntc_fault = (v & M_NTC_FAULT)

        return {
            "raw": v,
            "watchdog_fault": bool(v & B_WATCHDOG_FAULT),
            "boost_fault": bool(v & B_BOOST_FAULT),
            "chrg_fault": chrg_fault,
            "bat_fault": bool(v & B_BAT_FAULT),
            "ntc_fault": ntc_fault,
        }

    # -------------------------
    # Convenience: charger "phase" label (for your STATUS page / RGB)
    # -------------------------
    @staticmethod
    def chrg_stat_label(chrg_stat: int) -> str:
        # REG0B CHRG_STAT
        if chrg_stat == 0b00:
            return "not_charging"
        if chrg_stat == 0b01:
            return "precharge"
        if chrg_stat == 0b10:
            return "fast_charge"
        if chrg_stat == 0b11:
            return "done"
        return "unknown"

    # -------------------------
    # Debug helpers
    # -------------------------
    def dump_regs(self, start=0x00, end=0x14):
        """
        Returns list of (reg, value)
        """
        out = []
        for r in range(start, end + 1):
            out.append((r, self._rd(r)))
        return out
