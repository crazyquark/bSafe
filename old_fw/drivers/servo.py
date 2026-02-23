# drivers/servo.py
#
# SG90 servo driver (0..180 degrees).
# Default pulse range: 500us..2500us at 50Hz.

from machine import Pin, PWM

class ServoSG90:
    def __init__(self, pin, freq=50, min_us=500, max_us=2500, idle_deg=0):
        self.pin = Pin(pin, Pin.OUT)
        self.pwm = PWM(self.pin)
        self.pwm.freq(freq)

        self.freq = int(freq)
        self.min_us = int(min_us)
        self.max_us = int(max_us)

        self._period_ns = int(1_000_000_000 // self.freq)
        self._use_ns = hasattr(self.pwm, "duty_ns")

        self.write_deg(idle_deg)

    def deinit(self):
        try:
            self.pwm.deinit()
        except:
            pass

    def _clamp(self, v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def write_deg(self, deg):
        deg = float(deg)
        deg = self._clamp(deg, 0.0, 180.0)
        pulse_us = self.min_us + (deg / 180.0) * (self.max_us - self.min_us)
        self.write_us(pulse_us)

    def write_us(self, pulse_us):
        pulse_us = float(pulse_us)
        pulse_us = self._clamp(pulse_us, self.min_us, self.max_us)

        if self._use_ns:
            self.pwm.duty_ns(int(pulse_us * 1000))
        else:
            # duty_u16 uses 0..65535 across period
            duty = int((pulse_us / (1_000_000 / self.freq)) * 65535)
            duty = self._clamp(duty, 0, 65535)
            self.pwm.duty_u16(duty)
