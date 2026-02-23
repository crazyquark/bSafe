# control/mock_telemetry.py
#
# Fake telemetry generator for OLED-only testing:
# - simulates vbat/ibat evolution across charge/discharge states
# - lets you verify state transitions and IR triggering without hardware

class MockTelemetry:
    def __init__(self):
        self.vin_v = 12.0
        self.iin_a = 0.5
        self.vbat_v = 3.0
        self.ibat_a = 0.0
        self.ts_percent = 50.0  # mid divider => ~25C-ish for 10k/10k

    def step(self, state: str, dt_ms: int, *, target_v=3.6, charge_a=1.0, discharge_a=1.0):
        dt = float(dt_ms) / 1000.0

        if state in ("PRECONDITION", "CC", "CV", "RECHARGE"):
            # push vbat up slowly; ibat positive
            self.ibat_a = abs(charge_a) * 0.8
            self.vbat_v += 0.004 * dt  # ~0.24V/min at dt=1s
            if self.vbat_v > float(target_v) + 0.03:
                self.vbat_v = float(target_v) + 0.03

            # in CV, taper current
            if state == "CV":
                self.ibat_a *= 0.15

        elif state == "DISCHARGE":
            self.ibat_a = -abs(discharge_a) * 0.8
            self.vbat_v -= 0.006 * dt  # ~0.36V/min
            if self.vbat_v < 2.8:
                self.vbat_v = 2.8

        else:
            self.ibat_a = 0.0

        # fake TS percent drift a little
        self.ts_percent += 0.02 * dt
        if self.ts_percent > 80:
            self.ts_percent = 50

        return {
            "vin_v": self.vin_v,
            "iin_a": self.iin_a,
            "vbat_v": self.vbat_v,
            "ibat_a": self.ibat_a,
            "ts_percent": self.ts_percent,
        }
