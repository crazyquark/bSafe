# control/ir_measure.py
#
# Internal Resistance (IR) measurement sequencer.
#
# Trigger model:
# - HealthMetrics emits {"type":"IR_STEP","phase":"charge|discharge","v_step_idx":..., "vbat_v":..., "ibat_a":...}
#
# Measurement:
# 1) capture loaded V/I (from trigger moment)
# 2) pause the path (stop charge or pause discharge PWM)
# 3) wait settle_ms
# 4) capture relaxed V/I (fresh read)
# 5) compute R = dV / dI (mOhm) and subtract circuit resistance
#
# Notes:
# - This is "best effort" without fancy filtering yet.
# - Assumes PWM discharge makes "average" current measurable via INA226.

import time

class IRMeasureManager:
    ST_IDLE = 0
    ST_PAUSE_APPLIED = 1
    ST_WAIT_SETTLE = 2
    ST_DONE = 3

    def __init__(self, *,
                 settle_ms=50,
                 min_delta_i_a=0.15,
                 ema_alpha=0.15,
                 r_circuit_charge_mohm=25,
                 r_circuit_discharge_mohm=25):
        self.settle_ms = int(settle_ms)
        self.min_delta_i_a = float(min_delta_i_a)
        self.ema_alpha = float(ema_alpha)

        self.r_circuit_charge_mohm = int(r_circuit_charge_mohm)
        self.r_circuit_discharge_mohm = int(r_circuit_discharge_mohm)

        self._state = self.ST_IDLE
        self._queue = []

        self._t0_ms = 0
        self._tr = None  # current trigger
        self._loaded = None
        self._relaxed = None

        # Smoothed outputs
        self.r_charge_mohm = None
        self.r_discharge_mohm = None

        # callbacks (set by integrator)
        self.pause_charge_fn = None
        self.resume_charge_fn = None
        self.pause_discharge_fn = None
        self.resume_discharge_fn = None

        self._resume_pending = None  # "charge" or "discharge" or None

    def set_callbacks(self, *,
                      pause_charge,
                      resume_charge,
                      pause_discharge,
                      resume_discharge):
        self.pause_charge_fn = pause_charge
        self.resume_charge_fn = resume_charge
        self.pause_discharge_fn = pause_discharge
        self.resume_discharge_fn = resume_discharge

    def submit(self, trigger: dict):
        """
        Adds an IR_STEP trigger to queue.
        """
        if not trigger or trigger.get("type") != "IR_STEP":
            return
        self._queue.append(trigger)

    def abort(self):
        """
        Abort current measurement and resume any paused path.
        """
        self._do_resume_if_needed()
        self._state = self.ST_IDLE
        self._queue = []
        self._tr = None
        self._loaded = None
        self._relaxed = None

    def busy(self) -> bool:
        return self._state != self.ST_IDLE

    def _now(self):
        return time.ticks_ms()

    def _ticks_since(self, t0):
        return time.ticks_diff(self._now(), t0)

    def _do_pause(self, phase):
        if phase == "charge":
            if self.pause_charge_fn:
                self.pause_charge_fn()
            self._resume_pending = "charge"
        else:
            if self.pause_discharge_fn:
                self.pause_discharge_fn()
            self._resume_pending = "discharge"

    def _do_resume_if_needed(self):
        if self._resume_pending == "charge":
            if self.resume_charge_fn:
                self.resume_charge_fn()
        elif self._resume_pending == "discharge":
            if self.resume_discharge_fn:
                self.resume_discharge_fn()
        self._resume_pending = None

    @staticmethod
    def _ema(prev, new, a):
        if prev is None:
            return new
        return (1.0 - a) * prev + a * new

    def update(self, *, vbat_v, ibat_a, allow_phase: str):
        """
        Call every tick. Returns result dict when a measurement completes, else None.
        allow_phase: what phase is valid right now ("charge", "discharge", or None)
        """
        # Start a measurement if idle and queue not empty
        if self._state == self.ST_IDLE:
            if not self._queue:
                return None
            tr = self._queue.pop(0)

            phase = tr.get("phase")
            if allow_phase not in ("charge", "discharge") or phase != allow_phase:
                # phase mismatch (state changed); drop trigger
                return None

            self._tr = tr
            self._loaded = {
                "v": float(tr.get("vbat_v", vbat_v)),
                "i": float(tr.get("ibat_a", ibat_a)),
            }
            self._relaxed = None

            # apply pause now
            self._do_pause(phase)
            self._t0_ms = self._now()
            self._state = self.ST_WAIT_SETTLE
            return None

        # Waiting settle time
        if self._state == self.ST_WAIT_SETTLE:
            if self._ticks_since(self._t0_ms) < self.settle_ms:
                return None

            # Capture relaxed
            self._relaxed = {
                "v": float(vbat_v),
                "i": float(ibat_a),
            }

            # Resume whichever path we paused
            self._do_resume_if_needed()

            # Compute result
            res = self._compute_result()
            self._state = self.ST_IDLE
            self._tr = None
            self._loaded = None
            self._relaxed = None
            return res

        # Should not get here normally
        return None

    def _compute_result(self):
        tr = self._tr or {}
        phase = tr.get("phase", "charge")
        step = tr.get("v_step_idx", -1)

        v_loaded = self._loaded["v"]
        i_loaded = self._loaded["i"]
        v_relax = self._relaxed["v"]
        i_relax = self._relaxed["i"]

        dV = v_relax - v_loaded
        dI = abs(i_loaded - i_relax)

        ok = True
        reason = None

        if dI < self.min_delta_i_a:
            ok = False
            reason = "deltaI_small"

        # Basic physical sanity: during discharge pause, relaxed voltage should rise (dV > 0)
        # during charge pause, relaxed voltage should drop (dV < 0)
        if ok:
            if phase == "discharge" and dV < 0:
                ok = False
                reason = "dV_sign"
            if phase == "charge" and dV > 0:
                ok = False
                reason = "dV_sign"

        r_mohm = None
        r_corr_mohm = None

        if ok:
            r_ohm = abs(dV) / dI
            r_mohm = r_ohm * 1000.0

            if phase == "charge":
                r_corr_mohm = max(0.0, r_mohm - float(self.r_circuit_charge_mohm))
                self.r_charge_mohm = self._ema(self.r_charge_mohm, r_corr_mohm, self.ema_alpha)
            else:
                r_corr_mohm = max(0.0, r_mohm - float(self.r_circuit_discharge_mohm))
                self.r_discharge_mohm = self._ema(self.r_discharge_mohm, r_corr_mohm, self.ema_alpha)

        return {
            "type": "IR_RESULT",
            "ok": ok,
            "reason": reason,
            "phase": phase,
            "step": step,
            "v_loaded": v_loaded,
            "i_loaded": i_loaded,
            "v_relaxed": v_relax,
            "i_relaxed": i_relax,
            "dV": dV,
            "dI": dI,
            "r_mohm_raw": r_mohm,
            "r_mohm_corr": r_corr_mohm,
            "r_charge_ema_mohm": self.r_charge_mohm,
            "r_discharge_ema_mohm": self.r_discharge_mohm,
        }
