"""
bsafe_frames.py — shared frame constants, enums, and parsed dataclasses.

No CAN dependency. Imported by both bsafe_host.py (CAN transport) and
bsafe_wifi_host.py (TCP transport) so the two transports stay decoupled.
"""

import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum

# ---------------------------------------------------------------------------
# CAN arbitration IDs (informational — used by bsafe_host.py only)
# ---------------------------------------------------------------------------
CAN_ID_STATUS_BASE = 0x100
CAN_ID_CMD_BASE    = 0x200

# ---------------------------------------------------------------------------
# STATUS frame types  (byte [0] of every status payload)
# ---------------------------------------------------------------------------
FRAME_TYPE_OP       = 0x00   # Operational — autonomous 2 Hz
FRAME_TYPE_TELEM    = 0x01   # Extended telemetry — on request
FRAME_TYPE_IR       = 0x02   # IR measurement — on request
FRAME_TYPE_IDENTITY = 0x03   # Firmware identity — on request / page entry

# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class Mode(IntEnum):
    IDLE      = 0
    CHARGE    = 1
    DISCHARGE = 2
    MEASURE   = 3
    WAIT      = 4
    SETTLE    = 5


class ChrgStat(IntEnum):
    NONE = 0
    PRE  = 1
    FAST = 2
    DONE = 3


# ---------------------------------------------------------------------------
# Operational frame flag bits
# ---------------------------------------------------------------------------
FLAG_ERROR               = 0x01
FLAG_FULL                = 0x02
FLAG_PWR_OK              = 0x04
FLAG_BATT_PRESENT        = 0x08
FLAG_MODE_CHANGE_REQUEST = 0x10

# ---------------------------------------------------------------------------
# Host-side IR correction (MOSFET + resistor Rdson table)
# Power resistors fixed at 825 mΩ; MOSFET Rdson varies with Vbat.
# ---------------------------------------------------------------------------
_RDSON_TABLE = [
    (4.2, 30),
    (4.0, 35),
    (3.8, 45),
    (3.6, 65),
    (3.4, 105),
]


def circuit_resistance_mohm(vbat_v: float) -> float:
    """Interpolate total discharge circuit resistance at given battery voltage."""
    if vbat_v >= _RDSON_TABLE[0][0]:
        return 825 + _RDSON_TABLE[0][1]
    if vbat_v <= _RDSON_TABLE[-1][0]:
        return 825 + _RDSON_TABLE[-1][1]
    for i in range(len(_RDSON_TABLE) - 1):
        v_hi, r_hi = _RDSON_TABLE[i]
        v_lo, r_lo = _RDSON_TABLE[i + 1]
        if v_lo <= vbat_v <= v_hi:
            t = (vbat_v - v_lo) / (v_hi - v_lo)
            rdson = r_lo + t * (r_hi - r_lo)
            return 825 + rdson
    return 825 + 70


def corrected_ir_uohm(raw_ir_uohm: float, vbat_v: float) -> float:
    """Subtract interpolated circuit resistance from raw IR measurement."""
    circuit_uohm = circuit_resistance_mohm(vbat_v) * 1000.0
    return max(0.0, raw_ir_uohm - circuit_uohm)


# ---------------------------------------------------------------------------
# Parsed STATUS frames
# ---------------------------------------------------------------------------

@dataclass
class OperationalStatus:
    """TYPE 0x00 — main telemetry, arrives at 2 Hz autonomously."""
    address:             int      = 0
    vbat_v:              float    = 0.0
    ibat_a:              float    = 0.0
    error:               bool     = False
    full:                bool     = False
    pwr_ok:              bool     = False
    batt_present:        bool     = False
    mode_change_request: bool     = False
    chrg_stat:           ChrgStat = ChrgStat.NONE
    dsc_pct:             int      = 0
    mode:                Mode     = Mode.WAIT   # not on wire; set by engine
    rx_time:             float    = field(default_factory=time.time)

    @classmethod
    def parse(cls, address: int, data: bytes) -> 'OperationalStatus':
        if len(data) < 8:
            raise ValueError(f"OP frame too short ({len(data)}B)")
        vbat_mv = struct.unpack_from('>H', data, 1)[0]
        ibat_ma = struct.unpack_from('>h', data, 3)[0]
        flags   = data[5]
        cs_raw  = data[6]
        return cls(
            address             = address,
            vbat_v              = vbat_mv / 1000.0,
            ibat_a              = ibat_ma / 1000.0,
            error               = bool(flags & FLAG_ERROR),
            full                = bool(flags & FLAG_FULL),
            pwr_ok              = bool(flags & FLAG_PWR_OK),
            batt_present        = bool(flags & FLAG_BATT_PRESENT),
            mode_change_request = bool(flags & FLAG_MODE_CHANGE_REQUEST),
            chrg_stat           = ChrgStat(cs_raw) if cs_raw in ChrgStat._value2member_map_ else ChrgStat.NONE,
            dsc_pct             = data[7],
            rx_time             = time.time(),
        )

    def __str__(self) -> str:
        clab = {ChrgStat.NONE:'–', ChrgStat.PRE:'PRE', ChrgStat.FAST:'CC/CV', ChrgStat.DONE:'DONE'}
        fl = []
        if self.error:               fl.append('ERR')
        if self.full:                fl.append('FULL')
        if self.pwr_ok:              fl.append('PWR')
        if self.batt_present:        fl.append('BATT')
        if self.mode_change_request: fl.append('MCR')
        return (f"[{self.address}] OP  V={self.vbat_v:.3f}V  I={self.ibat_a:+.3f}A  "
                f"CHG={clab[self.chrg_stat]}  DSC={self.dsc_pct}%  "
                f"{' '.join(fl) or 'ok'}")


@dataclass
class TelemetryStatus:
    """TYPE 0x01 — extended telemetry, on request."""
    address:   int   = 0
    vbat_v:    float = 0.0
    ibat_a:    float = 0.0
    rpm:       int   = 0      # actual RPM (already ×50)
    bq_temp_c: int   = 0
    rx_time:   float = field(default_factory=time.time)

    @classmethod
    def parse(cls, address: int, data: bytes) -> 'TelemetryStatus':
        if len(data) < 7:
            raise ValueError(f"TELEM frame too short ({len(data)}B)")
        vbat_mv = struct.unpack_from('>H', data, 1)[0]
        ibat_ma = struct.unpack_from('>h', data, 3)[0]
        rpm8    = data[5]
        bq_temp = struct.unpack_from('b', data, 6)[0]   # signed int8
        return cls(
            address   = address,
            vbat_v    = vbat_mv / 1000.0,
            ibat_a    = ibat_ma / 1000.0,
            rpm       = rpm8 * 50,
            bq_temp_c = bq_temp,
            rx_time   = time.time(),
        )

    def __str__(self) -> str:
        return (f"[{self.address}] TEL V={self.vbat_v:.3f}V  I={self.ibat_a:+.3f}A  "
                f"RPM={self.rpm}  BQT={self.bq_temp_c}°C")


@dataclass
class IrStatus:
    """TYPE 0x02 — IR measurement + MCU temp, on request."""
    address:     int   = 0
    ir_uohm_raw: int   = 0     # raw (includes circuit resistance)
    mcu_temp_c:  int   = 0
    rx_time:     float = field(default_factory=time.time)

    def corrected_ir(self, vbat_v: float) -> float:
        """Return IR in µΩ with circuit Rdson subtracted."""
        return corrected_ir_uohm(float(self.ir_uohm_raw), vbat_v)

    @classmethod
    def parse(cls, address: int, data: bytes) -> 'IrStatus':
        if len(data) < 6:
            raise ValueError(f"IR frame too short ({len(data)}B)")
        ir_u  = struct.unpack_from('>I', data, 1)[0]   # uint32 BE
        mcu_t = struct.unpack_from('b',  data, 5)[0]   # signed int8
        return cls(
            address     = address,
            ir_uohm_raw = ir_u,
            mcu_temp_c  = mcu_t,
            rx_time     = time.time(),
        )

    def __str__(self) -> str:
        return (f"[{self.address}] IR  raw={self.ir_uohm_raw}µΩ  "
                f"MCU={self.mcu_temp_c}°C")


@dataclass
class IdentityStatus:
    """TYPE 0x03 — firmware identity, on request + CAN/WiFi page entry."""
    address:        int   = 0
    schema_version: int   = 0
    min_compat:     int   = 0
    hw_ver_hash:    int   = 0
    fw_major:       int   = 0
    fw_minor:       int   = 0
    fw_build:       int   = 0
    rx_time:        float = field(default_factory=time.time)

    @property
    def fw_version_str(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_build}"

    @classmethod
    def parse(cls, address: int, data: bytes) -> 'IdentityStatus':
        if len(data) < 8:
            raise ValueError(f"ID frame too short ({len(data)}B)")
        fw_build = struct.unpack_from('>H', data, 6)[0]
        return cls(
            address        = address,
            schema_version = data[1],
            min_compat     = data[2],
            hw_ver_hash    = data[3],
            fw_major       = data[4],
            fw_minor       = data[5],
            fw_build       = fw_build,
            rx_time        = time.time(),
        )

    def __str__(self) -> str:
        return (f"[{self.address}] ID  fw={self.fw_version_str}  "
                f"schema={self.schema_version}  min_compat={self.min_compat}  "
                f"hw_hash=0x{self.hw_ver_hash:02X}")


# Legacy alias so engine.py ChargerStatus references still work
ChargerStatus = OperationalStatus
