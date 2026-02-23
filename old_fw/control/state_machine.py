# control/state_machine.py
#
# Charger state machine scaffold.
# Focus: structure + hooks. Real control details come next.

import time

import config
from control.battery_presence import BatteryPresence
from control.safety import Safety
from control.health_metrics import HealthMetrics
from control.session_logger import SessionLogger
from control.discharge import DischargeController
from control.ir_measure import IRMeasureManager
from control.charge_control import ChargeControl


STATE_NO_BATT      = "NO_BATT"
STATE_PRECOND      = "PRECONDITION"
STATE_CC           = "CC"
STATE_CV           = "CV"
STATE_WAIT         = "WAIT"
STATE_DISCHARGE    = "DISCHARGE"
STATE_RECHARGE     = "RECHARGE"
STATE_DONE         = "DONE"
STATE_FAULT        = "FAULT"


class ChargerStateMachine:
    def __init__(self, board, *, settings_obj, rgb=None, discharge_pin=None, thermal=None):
        self.board = board
        self.settings = settings_obj  # dict-like settings.settings
        self.rgb = rgb                # optional RGBStrip
        self.discharge_pin = discharge_pin
        self.thermal = thermal        # optional ThermalManager

        self.discharge = None
        if discharge_pin is not None:
            self.discharge = DischargeController(
                discharge_pin,
                r_load_ohm=config.DISCHARGE_RESISTOR_OHMS,
                pwm_hz=1000
            )

        self._discharge_target_ah = None

        self.presence = BatteryPresence()
        self.safety = Safety()
        self.health = HealthMetrics(
            step_v=0.100,
            r_circuit_charge_mohm=config.CIRCUIT_R_CHARGE_MOHM,
            r_circuit_discharge_mohm=config.CIRCUIT_R_DISCHG_MOHM
        )
        self.session = SessionLogger()

        # IR measurement sequencer
        self.ir = IRMeasureManager(
            settle_ms=getattr(config, "IR_SETTLE_MS", 50),
            min_delta_i_a=getattr(config, "IR_MIN_DELTA_I_A", 0.15),
            ema_alpha=getattr(config, "IR_EMA_ALPHA", 0.15),
            r_circuit_charge_mohm=config.CIRCUIT_R_CHARGE_MOHM,
            r_circuit_discharge_mohm=config.CIRCUIT_R_DISCHG_MOHM
        )
        self.ir.set_callbacks(
            pause_charge=self._ir_pause_charge,
            resume_charge=self._ir_resume_charge,
            pause_discharge=self._ir_pause_discharge,
            resume_discharge=self._ir_resume_discharge
        )

        # Charge decision engine
        self.charge_ctl = ChargeControl()

        self.state = STATE_NO_BATT
        self.state_started_ms = time.ticks_ms()

        # runtime targets
        self.target_vbat_v = None
        self.charge_a = None
        self.discharge_a = None
        self.cap_ah = None

        # wait mode timers
        self.wait_until_ms = None

        self.last_fault = None
        self._last_vbat_for_resume = 0.0

    # -------------------------
    # Helpers
    # -------------------------
    def _set_state(self, new_state):
        if new_state == self.state:
            return
        self.state = new_state
        self.state_started_ms = time.ticks_ms()
        self._update_rgb()

        # Reset charge control timers when changing among charge phases
        if self.state in (STATE_PRECOND, STATE_CC, STATE_CV, STATE_RECHARGE):
            # charge control treats RECHARGE like CC/CV; timers reset on entry
            self.charge_ctl.reset_phase_timers()

    def seconds_in_state(self):
        return time.ticks_diff(time.ticks_ms(), self.state_started_ms) // 1000

    def _update_rgb(self):
        if self.rgb is None:
            return
        if self.state == STATE_PRECOND:
            self.rgb.set_status_color(config.RGB_PRECONDITION)
        elif self.state in (STATE_CC, STATE_CV):
            self.rgb.set_status_color(config.RGB_CHARGE)
        elif self.state == STATE_WAIT:
            self.rgb.set_status_color(config.RGB_WAIT)
        elif self.state == STATE_DISCHARGE:
            self.rgb.set_status_color(config.RGB_DISCHARGE)
        elif self.state == STATE_RECHARGE:
            self.rgb.set_status_color(config.RGB_RECHARGE)
        elif self.state == STATE_DONE:
            self.rgb.set_status_color(config.RGB_DONE)
        else:
            self.rgb.set_status_color(config.RGB_IDLE)

    def _compute_targets_from_settings(self):
        chem = self.settings.get("chemistry", "LFP")
        base = config.CHEMISTRIES.get(chem, 3.60)
        dv = self.settings.get("voltage_offset_mv", 0) / 1000.0
        self.target_vbat_v = base + dv

        cap_mah = float(self.settings.get("cell_capacity_mah", 3000))
        p = int(self.settings.get("parallel_cells", 1))
        self.cap_ah = (cap_mah / 1000.0) * p

        self.charge_a = self.cap_ah * float(self.settings.get("charge_c_rate", 0.5))
        self.discharge_a = self.cap_ah * float(self.settings.get("discharge_c_rate", 2.0))

        self.safety.configure_from_settings(
            self.settings,
            vbat_target_v=self.target_vbat_v,
            charge_a=self.charge_a,
            discharge_a=self.discharge_a
        )

    def _apply_bq_charge_settings(self, *, current_a_override=None):
        if getattr(self.board, "bq", None) is None:
            return

        try:
            self.board.bq.set_charge_enable_i2c(True)
        except Exception:
            pass
        try:
            self.board.set_ce_enabled(True)
        except Exception:
            pass

        if self.target_vbat_v is not None:
            try:
                self.board.bq.set_charge_voltage_mv(int(self.target_vbat_v * 1000))
            except Exception:
                pass

        i_a = current_a_override if current_a_override is not None else self.charge_a
        if i_a is not None:
            try:
                self.board.bq.set_fast_charge_current_ma(int(float(i_a) * 1000))
            except Exception:
                pass

    def _stop_charging(self):
        try:
            self.board.bq.set_charge_enable_i2c(False)
        except Exception:
            pass
        try:
            self.board.set_ce_enabled(False)
        except Exception:
            pass

    def _stop_discharging(self):
        if self.discharge is None:
            return
        try:
            self.discharge.set_enabled(False)
        except Exception:
            pass

    def _stop_all_power_paths(self):
        self._stop_charging()
        self._stop_discharging()

    # -------------------------
    # IR callbacks
    # -------------------------
    def _is_charge_state(self):
        return self.state in (STATE_PRECOND, STATE_CC, STATE_CV, STATE_RECHARGE)

    def _is_discharge_state(self):
        return self.state == STATE_DISCHARGE

    def _ir_pause_charge(self):
        if self._is_charge_state():
            self._stop_charging()

    def _ir_resume_charge(self):
        if self._is_charge_state():
            # resume with normal current limit
            self._apply_bq_charge_settings()

    def _ir_pause_discharge(self):
        if self.discharge is not None and self._is_discharge_state():
            try:
                self.discharge.pause()
            except Exception:
                pass

    def _ir_resume_discharge(self):
        if self.discharge is not None and self._is_discharge_state():
            try:
                self.discharge.resume(self._last_vbat_for_resume)
            except Exception:
                pass

    # -------------------------
    # Telemetry snapshot
    # -------------------------
    def _read_telemetry(self):
        t = {
            "vin_v": None,
            "iin_a": None,
            "vbat_v": None,
            "ibat_a": None,
            "bq_ts": None,
            "bq_chrg_stat": None,
        }
        try:
            t["vin_v"] = self.board.ina_sys.read_bus_voltage_v()
            t["iin_a"] = self.board.ina_sys.read_current_a()
        except Exception:
            pass
        try:
            t["vbat_v"] = self.board.ina_batt.read_bus_voltage_v()
            t["ibat_a"] = self.board.ina_batt.read_current_a()
        except Exception:
            pass
        if getattr(self.board, "has_bq", False):
            t["bq_ts"] = self._read_bq_ts_percent()
            t["bq_chrg_stat"] = self._read_bq_chrg_stat()
        return t

    def _read_bq_ts_percent(self):
        try:
            return self.board.bq.read_ts_percent_regn()
        except Exception:
            return None

    def _read_bq_chrg_stat(self):
        try:
            st = self.board.bq.read_status()
            return st.get("chrg_stat", None)
        except Exception:
            return None

    def _convert_ts_percent_to_temp_c(self, ts_percent):
        if ts_percent is None:
            return None
        try:
            from control.ntc import ts_percent_to_temp_c
            temp_c = ts_percent_to_temp_c(
                ts_percent=ts_percent,
                r_pullup_ohm=config.TS_PULLUP_OHMS,
                r0_ohm=config.TS_NTC_R0_OHMS,
                beta_k=config.TS_NTC_BETA_K,
                t0_c=config.TS_NTC_T0_C,
            )
            if temp_c != temp_c:
                return None
            return temp_c
        except Exception:
            return None

    # -------------------------
    # Main update
    # -------------------------
    def update(self, dt_ms, *, mcu_temp_c=None):
        dt_ms = int(dt_ms) if dt_ms is not None else 0

        tel = self._read_telemetry()
        vbat = tel["vbat_v"] if tel["vbat_v"] is not None else 0.0
        ibat = tel["ibat_a"] if tel["ibat_a"] is not None else 0.0
        vin  = tel["vin_v"]
        iin  = tel["iin_a"]
        # Convert TS% -> °C (best effort). If conversion fails, keep raw.
        batt_temp_c = None
        if getattr(self.board, "has_bq", False):
            batt_temp_c = self._convert_ts_percent_to_temp_c(tel["bq_ts"])


        self._last_vbat_for_resume = vbat

        # thermal manager hook
        if self.thermal is not None and mcu_temp_c is not None:
            try:
                self.thermal.update(mcu_temp_c)
            except Exception:
                pass

        # battery presence
        changed, present = self.presence.update(vbat)
        if changed:
            if present:
                self._compute_targets_from_settings()
                self.session.start()
                self.ir.abort()
                self.charge_ctl.reset_phase_timers()
                self._set_state(STATE_PRECOND)
            else:
                self.ir.abort()
                self._stop_all_power_paths()
                self.session.stop()
                self._set_state(STATE_NO_BATT)

        # safety
        ok, fault = self.safety.check(
            vin_v=vin, vbat_v=vbat, ibat_a=ibat, iin_a=iin,
            mcu_temp_c=mcu_temp_c, batt_temp_c=batt_temp_c
        )
        if not ok:
            self.last_fault = fault
            self.ir.abort()
            self._stop_all_power_paths()
            self._set_state(STATE_FAULT)

        # -------------------------
        # State behavior
        # -------------------------
        if self.state == STATE_NO_BATT:
            self._stop_all_power_paths()

        elif self.state in (STATE_PRECOND, STATE_CC, STATE_CV, STATE_RECHARGE):
            # Charging path. Ensure discharge off.
            self._stop_discharging()

            # Map RECHARGE to CC/CV decisions too.
            # For decisions, treat RECHARGE like CC unless we explicitly move to CV.
            dec_state = self.state if self.state != STATE_RECHARGE else STATE_CC

            dec = self.charge_ctl.decide(
                state=dec_state,
                vbat_v=vbat,
                ibat_a=ibat,
                target_vbat_v=self.target_vbat_v,
                cap_ah=self.cap_ah,
                charge_a=self.charge_a,
                settings=self.settings,
                config_module=config
            )

            # Apply charge settings (possibly reduced current in precondition)
            try:
                self._apply_bq_charge_settings(current_a_override=dec.get("set_current_a"))
            except Exception:
                pass

            # Handle transitions among PRECOND/CC/CV
            ns = dec.get("next_state")
            if ns == "PRECONDITION":
                self._set_state(STATE_PRECOND)
            elif ns == "CC":
                self._set_state(STATE_CC if self.state != STATE_RECHARGE else STATE_RECHARGE)
            elif ns == "CV":
                self._set_state(STATE_CV)

            # Done criteria from CV
            if dec.get("done"):
                if dec.get("stop_charging"):
                    self._stop_charging()
                self._enter_wait_state()

        elif self.state == STATE_WAIT:
            self._stop_all_power_paths()
            if self.wait_until_ms is None:
                self._enter_discharge_or_done()
            else:
                if time.ticks_diff(time.ticks_ms(), self.wait_until_ms) >= 0:
                    self._enter_discharge_or_done()

        elif self.state == STATE_DISCHARGE:
            self._stop_charging()

            if self.discharge is not None:
                if not self.discharge.enabled:
                    self.discharge.set_target_current(self.discharge_a if self.discharge_a is not None else 0.0)
                    self.discharge.set_target_ah(self._discharge_target_ah)
                    self.discharge.set_enabled(True)

                self.discharge.update(vbat_v=vbat, ibat_a=ibat, dt_ms=dt_ms)

                if self.discharge.reached_target():
                    self._stop_discharging()
                    self.ir.abort()
                    self._set_state(STATE_RECHARGE)
            else:
                self.ir.abort()
                self._set_state(STATE_RECHARGE)

        elif self.state == STATE_DONE:
            self._stop_all_power_paths()

        elif self.state == STATE_FAULT:
            self._stop_all_power_paths()

        # -------------------------
        # Health triggers + IR sequencing
        # -------------------------
        if present:
            phase = "charge" if self._is_charge_state() else "discharge"
            try:
                triggers = self.health.on_voltage_sample(phase, vbat, ibat, paused=False)
                for tr in triggers:
                    self.session.log_kv(evt=tr["type"], phase=tr["phase"], step=tr["v_step_idx"], v=vbat, i=ibat)
                    self.ir.submit(tr)
            except Exception:
                pass

        allow_phase = "charge" if self._is_charge_state() else ("discharge" if self._is_discharge_state() else None)
        ir_res = None
        try:
            ir_res = self.ir.update(vbat_v=vbat, ibat_a=ibat, allow_phase=allow_phase)
        except Exception:
            ir_res = None

        if ir_res is not None:
            if ir_res.get("ok"):
                if ir_res.get("phase") == "charge" and self.ir.r_charge_mohm is not None:
                    self.health.set_results(r_charge_mohm=int(self.ir.r_charge_mohm))
                if ir_res.get("phase") == "discharge" and self.ir.r_discharge_mohm is not None:
                    self.health.set_results(r_discharge_mohm=int(self.ir.r_discharge_mohm))

            self.session.log_kv(
                evt="IR_RESULT",
                ok=ir_res.get("ok"),
                phase=ir_res.get("phase"),
                step=ir_res.get("step"),
                r_raw=ir_res.get("r_mohm_raw"),
                r_corr=ir_res.get("r_mohm_corr"),
                dV=ir_res.get("dV"),
                dI=ir_res.get("dI"),
                reason=ir_res.get("reason")
            )

        # session logging
        if self.session.active:
            self.session.log_kv(
                state=self.state,
                t=self.seconds_in_state(),
                vin=vin, iin=iin,
                vbat=vbat, ibat=ibat,
                fault=self.last_fault
            )

        return {
            "state": self.state,
            "seconds": self.seconds_in_state(),
            "vin_v": vin,
            "iin_a": iin,
            "vbat_v": vbat,
            "ibat_a": ibat,
            "batt_temp_c": batt_temp_c,
            "fault": self.last_fault,
            "r_chg_mohm": self.health.r_charge_mohm,
            "r_dis_mohm": self.health.r_discharge_mohm,
            "discharge_ah": (self.discharge.discharged_ah if self.discharge is not None else 0.0),
            "discharge_target_ah": (self._discharge_target_ah if self._discharge_target_ah is not None else 0.0),
        }

    def _enter_wait_state(self):
        self._set_state(STATE_WAIT)
        mode = self.settings.get("wait_mode", "NO_WAIT")

        if mode == "NO_WAIT":
            self.wait_until_ms = None
            return

        hours_map = {"4H": 4, "8H": 8, "12H": 12, "16H": 16}
        h = hours_map.get(mode, 0)
        if h <= 0:
            self.wait_until_ms = None
            return
        self.wait_until_ms = time.ticks_add(time.ticks_ms(), int(h * 3600 * 1000))

    def _enter_discharge_or_done(self):
        target_ah = self._compute_discharge_target_ah()
        if target_ah is None:
            self._stop_all_power_paths()
            self.ir.abort()
            self._set_state(STATE_DONE)
            return

        self._discharge_target_ah = target_ah
        if self.discharge is not None:
            self.discharge.set_target_current(self.discharge_a if self.discharge_a is not None else 0.0)
            self.discharge.set_target_ah(target_ah)
            self.discharge.set_enabled(True)

        self.ir.abort()
        self._set_state(STATE_DISCHARGE)

    def _compute_discharge_target_ah(self):
        target = self.settings.get("discharge_target", "-100%")
        if target == "KEEP_FULL":
            return None

        try:
            pct = int(str(target).replace("%", "").strip())
        except Exception:
            pct = -100

        pct = abs(pct)
        if pct > 100:
            pct = 100

        cap_mah = float(self.settings.get("cell_capacity_mah", 3000))
        p = int(self.settings.get("parallel_cells", 1))
        cap_ah = (cap_mah / 1000.0) * p

        return cap_ah * (pct / 100.0)
