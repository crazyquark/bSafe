# control/ntc.py
#
# NTC helper functions (Beta model).
# Works with divider ratio (Vts/Vref), so absolute Vref cancels.

import math

def ntc_resistance_from_ratio(*, ratio_vts_over_vref: float, r_pullup_ohm: float) -> float:
    """
    Given x = Vts/Vref for a divider:
      Vref -- Rpullup --(TS node)-- Rntc -- GND
    Solve for Rntc:
      x = Rntc / (Rpullup + Rntc)
      Rntc = Rpullup * x / (1 - x)
    """
    x = float(ratio_vts_over_vref)

    # Clamp away from 0 and 1 to avoid blowups
    if x <= 0.000001:
        x = 0.000001
    if x >= 0.999999:
        x = 0.999999

    rpu = float(r_pullup_ohm)
    return rpu * (x / (1.0 - x))


def ntc_temp_c_from_resistance(*, r_ohm: float, r0_ohm: float, beta_k: float, t0_c: float = 25.0) -> float:
    """
    Beta model:
      1/T = 1/T0 + (1/B) * ln(R/R0)
    where T and T0 are in Kelvin.
    """
    r = float(r_ohm)
    r0 = float(r0_ohm)
    b = float(beta_k)

    # guard
    if r <= 0 or r0 <= 0 or b <= 0:
        return float("nan")

    t0_k = float(t0_c) + 273.15
    inv_t = (1.0 / t0_k) + (1.0 / b) * math.log(r / r0)
    t_k = 1.0 / inv_t
    return t_k - 273.15


def ts_percent_to_temp_c(*, ts_percent: float, r_pullup_ohm: float, r0_ohm: float, beta_k: float, t0_c: float = 25.0) -> float:
    """
    ts_percent is Vts/Vref * 100, where Vref is REGN here.
    """
    ratio = float(ts_percent) / 100.0
    r_ntc = ntc_resistance_from_ratio(ratio_vts_over_vref=ratio, r_pullup_ohm=r_pullup_ohm)
    return ntc_temp_c_from_resistance(r_ohm=r_ntc, r0_ohm=r0_ohm, beta_k=beta_k, t0_c=t0_c)
