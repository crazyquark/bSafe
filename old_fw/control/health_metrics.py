# control/health_metrics.py
#
# Health metrics scaffolding:
# - schedule internal resistance measurements every 100mV step
# - store results progressively
#
# The actual IR math will be added once we finalize:
# - discharge PWM burst timing
# - "paused" capture timing
# - circuit resistance compensation constants

class HealthMetrics:
    def __init__(self, *, step_v=0.100, r_circuit_charge_mohm=25, r_circuit_discharge_mohm=25):
        self.step_v = float(step_v)
        self.r_circuit_charge_mohm = int(r_circuit_charge_mohm)
        self.r_circuit_discharge_mohm = int(r_circuit_discharge_mohm)

        # Track last step index per phase
        self._last_step_idx = {
            "charge": None,
            "discharge": None,
        }

        # Accumulated results (placeholders)
        self.r_charge_mohm = 25
        self.r_discharge_mohm = 25

        # Store raw samples for later processing
        self._pending = []  # list of dict records

    def _step_index(self, v):
        # floor to 100mV step
        return int(float(v) / self.step_v)

    def on_voltage_sample(self, phase, vbat_v, ibat_a, paused=False):
        """
        phase: "charge" or "discharge"
        paused: True when current is paused (for delta-V capture)
        Returns: list of measurement triggers (each trigger is a dict)
        """
        phase = "charge" if phase == "charge" else "discharge"
        v = float(vbat_v)
        i = float(ibat_a)

        idx = self._step_index(v)
        last = self._last_step_idx.get(phase)

        triggers = []
        if last is None:
            self._last_step_idx[phase] = idx
        elif idx != last:
            # crossed into new 100mV bucket
            self._last_step_idx[phase] = idx
            triggers.append({
                "type": "IR_STEP",
                "phase": phase,
                "v_step_idx": idx,
                "vbat_v": v,
                "ibat_a": i,
            })

        # record sample for later processing
        self._pending.append({
            "phase": phase,
            "vbat_v": v,
            "ibat_a": i,
            "paused": bool(paused),
        })

        return triggers

    def consume_pending(self):
        """
        Returns and clears pending samples.
        """
        out = self._pending
        self._pending = []
        return out

    def set_results(self, *, r_charge_mohm=None, r_discharge_mohm=None):
        if r_charge_mohm is not None:
            self.r_charge_mohm = int(r_charge_mohm)
        if r_discharge_mohm is not None:
            self.r_discharge_mohm = int(r_discharge_mohm)
