# control/discharge.py
#
# PWM discharge control for an ON/OFF resistive dump load.
#
# Hardware:
# - DISCHARGE pin: active-high gate/enable for resistor module (~0.85 ohm).
#
# Math (ideal):
#   I_avg ≈ D * (Vbat / Rload)
#   D ≈ I_target * Rload / Vbat
#
# Practical notes:
# - Clamp duty to [0..1]
# - Use moderate PWM freq (default 1000 Hz) so INA226 averaging sees stable mean
# - Integrate discharged Ah for discharge_target percent logic

from machine import Pin, PWM

class DischargeController:
    def __init__(self, pin, *, r_load_ohm=0.85, pwm_hz=1000, max_duty=0.98):
        self.r_load = float(r_load_ohm)
        self.max_duty = float(max_duty)

        self.pwm = PWM(Pin(pin, Pin.OUT))
        self.pwm.freq(int(pwm_hz))

        self._use_ns = hasattr(self.pwm, "duty_ns")
        self._period_ns = int(1_000_000_000 // int(pwm_hz))

        self.enabled = False
        self.duty = 0.0
        self.i_target_a = 0.0

        # Coulomb counter (Ah)
        self.discharged_ah = 0.0
        self._target_ah = None

        self.set_enabled(False)

    def deinit(self):
        try:
            self.pwm.deinit()
        except:
            pass

    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def set_enabled(self, en: bool):
        self.enabled = bool(en)
        if not self.enabled:
            self.set_duty(0.0)

    def set_target_current(self, i_target_a: float):
        self.i_target_a = float(i_target_a)

    def set_target_ah(self, target_ah):
        """
        None disables target stop.
        """
        self._target_ah = None if target_ah is None else float(target_ah)
        self.discharged_ah = 0.0

    def reached_target(self) -> bool:
        return (self._target_ah is not None) and (self.discharged_ah >= self._target_ah)

    def compute_duty_from_vbat(self, vbat_v: float) -> float:
        v = float(vbat_v)
        if v <= 0.2:
            return 0.0
        # D ≈ I_target * R / V
        d = (self.i_target_a * self.r_load) / v
        d = self._clamp(d, 0.0, self.max_duty)
        return d

    def set_duty(self, duty_0_to_1: float):
        d = self._clamp(float(duty_0_to_1), 0.0, 1.0)
        self.duty = d

        if self._use_ns:
            # duty_ns expects high-time in ns
            high_ns = int(self._period_ns * d)
            self.pwm.duty_ns(high_ns)
        else:
            # duty_u16 expects 0..65535
            self.pwm.duty_u16(int(d * 65535))

    def update(self, *, vbat_v: float, ibat_a: float, dt_ms: int):
        """
        Call every tick in DISCHARGE state.

        vbat_v: present battery voltage
        ibat_a: measured battery current (negative when discharging ideally)
        dt_ms : elapsed milliseconds since last update
        """
        if not self.enabled:
            return

        # Update duty based on present Vbat
        d = self.compute_duty_from_vbat(vbat_v)
        self.set_duty(d)

        # Integrate discharged Ah using measured current if available.
        # We treat discharge current as positive magnitude:
        i = float(ibat_a)
        if i < 0:
            i = -i
        else:
            # if sign isn't negative (wiring / mock), still integrate magnitude
            i = abs(i)

        dt_s = float(dt_ms) / 1000.0
        self.discharged_ah += (i * dt_s) / 3600.0

    # Optional: pause for IR measurements
    def pause(self):
        self.set_duty(0.0)

    def resume(self, vbat_v: float):
        self.set_duty(self.compute_duty_from_vbat(vbat_v))
