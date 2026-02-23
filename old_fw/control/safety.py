# control/safety.py
#
# Safety checks. Keep it small and explicit.
# Later: integrate INA226 alerts + BQ fault codes.

class Safety:
    def __init__(self):
        # Defaults are conservative placeholders; later derived from settings/chemistry.
        self.max_vin_v = 20.0
        self.max_vbat_v = 4.6
        self.max_abs_charge_a = 6.0
        self.max_abs_discharge_a = 8.0
        self.max_mcu_temp_c = 80.0
        self.max_batt_temp_c = 60.0

        self.last_fault = None

    def configure_from_settings(self, settings, *, vbat_target_v=None, charge_a=None, discharge_a=None):
        """
        Hook for later. Keeps defaults if values not provided.
        """
        if vbat_target_v is not None:
            self.max_vbat_v = float(vbat_target_v) + 0.08  # small headroom
        if charge_a is not None:
            self.max_abs_charge_a = float(charge_a) * 1.15
        if discharge_a is not None:
            self.max_abs_discharge_a = float(discharge_a) * 1.15

    def check(self, *, vin_v, vbat_v, ibat_a, iin_a, mcu_temp_c, batt_temp_c):
        """
        Returns (ok: bool, fault: str|None)
        """
        # Reset previous fault each call unless we want latched behavior later
        self.last_fault = None

        if vin_v is not None and float(vin_v) > self.max_vin_v:
            self.last_fault = "VIN_OV"
        elif vbat_v is not None and float(vbat_v) > self.max_vbat_v:
            self.last_fault = "VBAT_OV"
        elif ibat_a is not None and float(ibat_a) > self.max_abs_charge_a:
            self.last_fault = "CHG_OC"
        elif ibat_a is not None and float(ibat_a) < -self.max_abs_discharge_a:
            self.last_fault = "DIS_OC"
        elif mcu_temp_c is not None and float(mcu_temp_c) > self.max_mcu_temp_c:
            self.last_fault = "MCU_OT"
        elif batt_temp_c is not None and float(batt_temp_c) > self.max_batt_temp_c:
            self.last_fault = "BATT_OT"

        return (self.last_fault is None, self.last_fault)
