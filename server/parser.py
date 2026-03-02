"""
parser.py — bSafe program language parser
Validates, clamps, and suggests improvements to charge programs.
"""
import re
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# MOSFET + resistor Rdson table (voltage → total circuit resistance mΩ)
# From measured data: power_resistors=825mΩ + Rdson(Vbat)
# ---------------------------------------------------------------------------
RDSON_TABLE = [
    (4.2, 30),
    (4.0, 35),
    (3.8, 45),
    (3.6, 65),
    (3.4, 105),
]

def circuit_resistance_mohm(vbat_v: float) -> float:
    """Interpolate total discharge circuit resistance at given battery voltage."""
    if vbat_v >= RDSON_TABLE[0][0]:
        return 825 + RDSON_TABLE[0][1]
    if vbat_v <= RDSON_TABLE[-1][0]:
        return 825 + RDSON_TABLE[-1][1]
    for i in range(len(RDSON_TABLE) - 1):
        v_hi, r_hi = RDSON_TABLE[i]
        v_lo, r_lo = RDSON_TABLE[i + 1]
        if v_lo <= vbat_v <= v_hi:
            t = (vbat_v - v_lo) / (v_hi - v_lo)
            rdson = r_lo + t * (r_hi - r_lo)
            return 825 + rdson
    return 825 + 70  # fallback midpoint

def corrected_ir_uohm(raw_ir_mohm: float, vbat_v: float) -> float:
    """Remove circuit resistance from raw IR measurement (host-side correction)."""
    circuit_mohm = circuit_resistance_mohm(vbat_v)
    corrected_mohm = raw_ir_mohm - circuit_mohm
    return max(0.0, corrected_mohm) * 1000.0  # → µΩ

# ---------------------------------------------------------------------------
# Parsed command
# ---------------------------------------------------------------------------
@dataclass
class ParsedCommand:
    line_num:   int
    raw:        str
    cmd:        str          # DETECT, PRECHARGE, MEASURE, CHARGE, DISCHARGE, SETTLE
    param:      Optional[float] = None   # numeric value
    unit:       Optional[str]  = None    # V, H, M, S, A
    errors:     List[str]      = field(default_factory=list)
    warnings:   List[str]      = field(default_factory=list)
    clamped:    bool           = False

# ---------------------------------------------------------------------------
# Default hard limits (overridden by DB values at validation time)
# ---------------------------------------------------------------------------
DEFAULT_LIMITS = {
    "max_charge_v":         4.25,
    "min_discharge_v":      3.00,
    "max_settle_h":         48.0,
    "max_charge_current_a": 5.0,
    "stop_on_fan_fail":     True,
    "fan_fail_timeout_s":   5,
    "max_temp_c":           60,
    "stop_on_temp_exceed":  True,
}

# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
COMMAND_RE = re.compile(
    r'^\s*'
    r'(?P<cmd>DETECT|PRECHARGE|MEASURE|CHARGE|DISCHARGE|SETTLE|WAIT|LOOP|END)'
    r'(?:=(?P<val>[0-9]*\.?[0-9]+)(?P<unit>[VvHhMmSsAa]?))?'
    r'\s*(?:#.*)?$'
)

VALID_COMMANDS = {
    "DETECT":     "V",    # voltage threshold to start
    "PRECHARGE":  "V",    # pre-charge termination voltage (2.8 or 3.0)
    "MEASURE":    None,   # no param — IR measurement
    "CHARGE":     "V",    # target charge voltage
    "DISCHARGE":  "V",    # cutoff discharge voltage
    "SETTLE":     "H",    # rest time in hours
    "WAIT":       "M",    # wait in minutes (host-side delay)
    "LOOP":       None,   # loop back to start (no param)
    "END":        None,   # explicit end (optional)
}

def parse_duration_to_minutes(val: float, unit: str) -> float:
    u = unit.upper() if unit else "H"
    if u == "H": return val * 60
    if u == "M": return val
    if u == "S": return val / 60
    return val * 60  # default hours

def parse_program(body: str, limits: dict = None) -> Tuple[List[ParsedCommand], List[str]]:
    """
    Parse program body. Returns (commands, suggestions).
    Each command has .errors (blocking) and .warnings (non-blocking).
    """
    lim = {**DEFAULT_LIMITS, **(limits or {})}
    lines = body.strip().splitlines()
    commands: List[ParsedCommand] = []
    suggestions: List[str] = []

    has_detect     = False
    has_charge     = False
    has_discharge  = False
    has_measure    = False
    has_settle     = False
    has_precharge  = False
    last_cmd       = None

    for i, raw in enumerate(lines):
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue

        pc = ParsedCommand(line_num=i + 1, raw=stripped, cmd="")
        m = COMMAND_RE.match(stripped)

        if not m:
            pc.cmd = stripped.split("=")[0].upper()
            pc.errors.append(f"Unknown command '{stripped}'")
            commands.append(pc)
            continue

        cmd  = m.group("cmd").upper()
        val  = float(m.group("val")) if m.group("val") else None
        unit = (m.group("unit") or "").upper() or None
        pc.cmd  = cmd
        pc.param = val
        pc.unit  = unit

        expected_unit = VALID_COMMANDS.get(cmd)

        # Commands that must not have a param
        if cmd in ("MEASURE", "LOOP", "END") and val is not None:
            pc.errors.append(f"{cmd} takes no parameter")

        # Commands that must have a param
        if cmd in ("DETECT", "CHARGE", "DISCHARGE") and val is None:
            pc.errors.append(f"{cmd} requires a voltage parameter, e.g. {cmd}=3.7V")

        if cmd == "SETTLE" and val is None:
            pc.errors.append("SETTLE requires a duration, e.g. SETTLE=2H")

        # Unit validation
        if val is not None and expected_unit and unit not in (expected_unit, None, ""):
            pc.warnings.append(
                f"Expected unit {expected_unit} for {cmd}, got '{unit or '(none)'}'")

        # Range checks and clamping
        if cmd == "DETECT" and val is not None:
            has_detect = True
            if val < 0.2:
                pc.errors.append("DETECT voltage must be ≥ 0.2V (battery present threshold)")
            if val > 4.25:
                pc.errors.append("DETECT voltage > 4.25V makes no sense")

        elif cmd == "PRECHARGE" and val is not None:
            has_precharge = True
            if val not in (2.8, 3.0):
                if abs(val - 2.8) < abs(val - 3.0):
                    pc.warnings.append(f"PRECHARGE only supports 2.8V or 3.0V — will use 2.8V")
                    pc.param = 2.8
                else:
                    pc.warnings.append(f"PRECHARGE only supports 2.8V or 3.0V — will use 3.0V")
                    pc.param = 3.0
                pc.clamped = True
        elif cmd == "PRECHARGE" and val is None:
            has_precharge = True  # no-param = use BQ default (3.0V)

        elif cmd == "CHARGE" and val is not None:
            has_charge = True
            if val > lim["max_charge_v"]:
                pc.errors.append(
                    f"CHARGE={val}V exceeds hard limit {lim['max_charge_v']}V")
            if val < 3.5:
                pc.warnings.append("CHARGE voltage < 3.5V is unusual for Li-Ion")

        elif cmd == "DISCHARGE" and val is not None:
            has_discharge = True
            if val < lim["min_discharge_v"]:
                pc.errors.append(
                    f"DISCHARGE={val}V below hard limit {lim['min_discharge_v']}V")
            if val > 4.0:
                pc.errors.append("DISCHARGE cutoff > 4.0V — battery will barely discharge")

        elif cmd == "SETTLE" and val is not None:
            has_settle = True
            minutes = parse_duration_to_minutes(val, unit or "H")
            hours = minutes / 60
            if hours > lim["max_settle_h"]:
                pc.errors.append(
                    f"SETTLE={val}{unit or 'H'} ({hours:.1f}H) exceeds hard limit "
                    f"{lim['max_settle_h']}H")

        elif cmd == "MEASURE":
            has_measure = True

        # Consecutive MEASURE warning
        if cmd == last_cmd == "MEASURE":
            pc.warnings.append("Consecutive MEASURE commands — second one is redundant")

        last_cmd = cmd
        commands.append(pc)

    # --- Structural suggestions ---
    has_errors = any(e.errors for e in commands)

    if not has_detect:
        suggestions.append("💡 Consider adding DETECT=2.0V at the start to gate program entry on battery presence")

    if has_charge and not has_precharge:
        suggestions.append("💡 Consider adding PRECHARGE=3.0V before CHARGE for depleted batteries")

    if has_charge and not has_measure:
        suggestions.append("💡 Add MEASURE after CHARGE to capture post-charge IR")

    if has_discharge and not has_measure:
        suggestions.append("💡 Add MEASURE after DISCHARGE to capture end-of-discharge IR")

    if has_charge and has_discharge and not has_settle:
        suggestions.append("💡 Consider adding SETTLE=1H between CHARGE and DISCHARGE to let voltage stabilize")

    if has_discharge and not has_charge:
        suggestions.append("💡 Program ends on DISCHARGE — battery will be left depleted. Add CHARGE=4.2V to recharge")

    if not commands:
        suggestions.append("Program is empty — add commands to begin")

    # --- Template suggestions ---
    cmd_names = [c.cmd for c in commands if not c.errors]
    if not has_discharge and not has_charge:
        suggestions.append("📋 Template: Full Cycle = DETECT → PRECHARGE → MEASURE → CHARGE → MEASURE → SETTLE → DISCHARGE → MEASURE → CHARGE")
    if has_charge and has_discharge and has_measure and has_settle:
        suggestions.append("✅ Program looks like a complete cycle")

    return commands, suggestions

def program_is_valid(commands: List[ParsedCommand]) -> bool:
    return all(len(c.errors) == 0 for c in commands) and len(commands) > 0

def commands_to_steps(commands: List[ParsedCommand]) -> List[dict]:
    """Convert parsed commands to runtime step dicts for the state machine."""
    steps = []
    for pc in commands:
        if pc.errors:
            continue
        step = {
            "cmd":    pc.cmd,
            "param":  pc.param,
            "unit":   pc.unit,
            "raw":    pc.raw,
        }
        if pc.cmd == "SETTLE" and pc.param is not None:
            step["duration_min"] = parse_duration_to_minutes(pc.param, pc.unit or "H")
        steps.append(step)
    return steps
