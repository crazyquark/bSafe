# ui/model.py

class UIModel:
    def __init__(self):
        self.page_selected = 0
        self.page_entered = False

        self.status_subpage = 0
        self.status_seconds = 0
        self.device_state_label = "NO BATTERY"  # was "no_battery"

        self.input_voltage_v = 0.0
        self.input_current_a = 0.0
        self.mcu_temp_c = 0.0

        self.batt_voltage_v = 0.0
        self.batt_current_a = 0.0
        self.batt_temp_c = None

        self.cfg_chemistry = "LFP"
        self.cfg_voltage_offset_mv = 0
        self.cfg_capacity_mah = 3000
        self.cfg_parallel_cells = 1
        self.cfg_wait_mode = "NO_WAIT"
        self.cfg_discharge_target = "-100%"

        self.health_r_charge_mohm = 25
        self.health_r_discharge_mohm = 25

        self.mode_can = True
        self.mode_wifi = False
        self.mode_usb = True
        self.lock_settings_on_net = False
