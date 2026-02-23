# control/thermal.py
#
# Thermal policy: servo + (later) fan based on temperature thresholds.

class ThermalManager:
    def __init__(self, servo, *, threshold_c=45.0, idle_deg=0, hot_deg=180):
        self.servo = servo
        self.threshold_c = float(threshold_c)
        self.idle_deg = float(idle_deg)
        self.hot_deg = float(hot_deg)

        self._is_hot = False
        # ensure idle on boot
        try:
            self.servo.write_deg(self.idle_deg)
        except:
            pass

    def update(self, mcu_temp_c):
        """
        Call periodically with current MCU temperature.
        """
        t = float(mcu_temp_c)
        hot_now = t >= self.threshold_c
        if hot_now != self._is_hot:
            self._is_hot = hot_now
            try:
                self.servo.write_deg(self.hot_deg if self._is_hot else self.idle_deg)
            except:
                pass

    @property
    def is_hot(self):
        return self._is_hot
