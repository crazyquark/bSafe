"""
engine.py — Per-device program execution state machine
Each DeviceSession runs the program steps, sends CAN commands,
tracks telemetry, and logs events to the database.
"""
import time
import json
import threading
import logging
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass, field

from parser import commands_to_steps, corrected_ir_uohm, circuit_resistance_mohm
import db

log = logging.getLogger("engine")

# ---------------------------------------------------------------------------
# Step status
# ---------------------------------------------------------------------------
BATTERY_PRESENT_V  = 0.2
BATTERY_DEAD_LOW_V = 2.0    # below this = dead battery
BATTERY_GOOD_V     = BATTERY_DEAD_LOW_V  # same threshold for DETECT

# ---------------------------------------------------------------------------
# Device telemetry snapshot (filled by CAN layer)
# ---------------------------------------------------------------------------
@dataclass
class DeviceState:
    address:      int
    # Type 0x00 — operational status
    mode:         int   = 4    # CAN_MODE_WAIT
    vbat_v:       float = 0.0
    ibat_a:       float = 0.0
    error:        bool  = False
    full:         bool  = False
    pwr_ok:       bool  = False
    batt_present: bool  = False
    mode_change_request: bool = False
    chrg_stat:    int   = 0
    dsc_pct:      int   = 0
    # Type 0x01 — extended telemetry
    rpm:          int   = 0
    bq_temp_c:    int   = 0
    ir_uohm:      int   = 0    # last measured, corrected
    # Type 0x03 — identity
    schema_ver:   int   = 0
    hw_ver_hash:  int   = 0
    # Host-side tracking
    last_rx:      float = 0.0
    last_telemetry_rx: float = 0.0
    online:       bool  = False
    prev_batt_present: bool = False

# ---------------------------------------------------------------------------
# Session state machine
# ---------------------------------------------------------------------------
class DeviceSession:
    def __init__(self, address: int, session_id: int,
                 steps: List[dict], send_cmd_fn: Callable,
                 request_frame_fn: Callable):
        self.address       = address
        self.session_id    = session_id
        self.steps         = steps
        self.send_cmd      = send_cmd_fn       # send_cmd(address, mode, **kwargs)
        self.request_frame = request_frame_fn  # request_frame(address, frame_type)
        self.status        = "running"         # running/paused/stopped/done/error
        self.step_idx      = 0
        self.step_start    = time.time()
        self.step_data: dict = {}              # transient per-step state
        self.lock          = threading.Lock()
        self._log_step_start()

    def _log_step_start(self):
        if self.step_idx >= len(self.steps):
            return
        step = self.steps[self.step_idx]
        db.log_event("step_start", address=self.address,
                     session_id=self.session_id,
                     data={"step": self.step_idx, "cmd": step["cmd"],
                           "param": step.get("param"), "unit": step.get("unit")})

    @property
    def current_step_cmd(self) -> Optional[str]:
        if self.step_idx < len(self.steps):
            return self.steps[self.step_idx]["cmd"]
        return None

    @property
    def current_step_raw(self) -> Optional[str]:
        if self.step_idx < len(self.steps):
            return self.steps[self.step_idx]["raw"]
        return None

    def pause(self):
        with self.lock:
            if self.status == "running":
                self.status = "paused"
                self.step_data["paused_remaining_min"] = self.step_data.get("remaining_min")
                db.update_session(self.session_id, status="paused", paused_at=time.time())
                db.log_event("program_pause", address=self.address,
                             session_id=self.session_id,
                             data={"step": self.step_idx})
                self.send_cmd(self.address, mode=4)  # CAN_MODE_WAIT

    def resume(self):
        with self.lock:
            if self.status == "paused":
                self.status = "running"
                self.step_start = time.time()  # reset step timer
                db.update_session(self.session_id, status="running")
                db.log_event("program_resume", address=self.address,
                             session_id=self.session_id,
                             data={"step": self.step_idx})

    def stop(self):
        with self.lock:
            self.status = "stopped"
            db.update_session(self.session_id, status="stopped",
                              completed_at=time.time())
            db.log_event("program_stop", address=self.address,
                         session_id=self.session_id,
                         data={"step": self.step_idx})
            self.send_cmd(self.address, mode=4)  # CAN_MODE_WAIT

    def _advance(self, result: dict = None):
        """Complete current step and move to next."""
        step = self.steps[self.step_idx]
        db.record_step_result(
            session_id=self.session_id,
            address=self.address,
            step_index=self.step_idx,
            command=step["raw"],
            started_at=self.step_start,
            result=result or {},
        )
        db.log_event("step_complete", address=self.address,
                     session_id=self.session_id,
                     data={"step": self.step_idx, "cmd": step["cmd"],
                           "result": result})
        self.step_idx  += 1
        self.step_start = time.time()
        self.step_data  = {}
        db.update_session(self.session_id, current_step=self.step_idx)
        if self.step_idx >= len(self.steps):
            self.status = "done"
            db.update_session(self.session_id, status="done",
                              completed_at=time.time())
            db.log_event("program_done", address=self.address,
                         session_id=self.session_id, data={})
            self.send_cmd(self.address, mode=0)  # CAN_MODE_IDLE
        else:
            self._log_step_start()

    def tick(self, state: DeviceState, limits: dict):
        """
        Called every poll cycle. Drives the state machine for this device.
        Returns a dict of UI state info.
        """
        with self.lock:
            if self.status != "running":
                return self._ui_state(state)

            if self.step_idx >= len(self.steps):
                return self._ui_state(state)

            step = self.steps[self.step_idx]
            cmd  = step["cmd"]
            now  = time.time()
            elapsed = now - self.step_start

            # --- Battery dead check (except during DETECT) ---
            if cmd != "DETECT":
                if (state.batt_present and
                        BATTERY_PRESENT_V < state.vbat_v < BATTERY_DEAD_LOW_V):
                    db.log_event("battery_dead", address=self.address,
                                 session_id=self.session_id,
                                 data={"vbat": state.vbat_v})
                    self.step_data["battery_dead"] = True
                else:
                    self.step_data["battery_dead"] = False

            # --- Fan fail check ---
            if limits.get("stop_on_fan_fail") and cmd in ("CHARGE", "DISCHARGE"):
                if state.rpm == 0:
                    fail_t = self.step_data.get("fan_fail_since")
                    if fail_t is None:
                        self.step_data["fan_fail_since"] = now
                    elif now - fail_t > limits.get("fan_fail_timeout_s", 5):
                        db.log_event("fan_fail", address=self.address,
                                     session_id=self.session_id,
                                     data={"step": self.step_idx})
                        self.stop()
                        return self._ui_state(state)
                else:
                    self.step_data.pop("fan_fail_since", None)

            # --- Temp exceed check ---
            if limits.get("stop_on_temp_exceed"):
                if state.bq_temp_c > limits.get("max_temp_c", 60):
                    db.log_event("temp_exceed", address=self.address,
                                 session_id=self.session_id,
                                 data={"temp": state.bq_temp_c})
                    self.stop()
                    return self._ui_state(state)

            # ================================================================
            # Step execution
            # ================================================================

            if cmd == "DETECT":
                threshold = step.get("param", BATTERY_GOOD_V)
                if state.vbat_v >= threshold:
                    self._advance({"detected_v": state.vbat_v})
                else:
                    self.send_cmd(self.address, mode=4)  # WAIT

            elif cmd == "PRECHARGE":
                # BQ handles pre-charge. We send CHARGE mode and wait for
                # chrg_stat to leave pre-charge (chrg_stat > 1) or
                # vbat to exceed threshold
                threshold = step.get("param", 3.0)
                self.send_cmd(self.address, mode=1)  # CAN_MODE_CHARGE
                if state.vbat_v >= threshold or state.chrg_stat > 1:
                    self._advance({"precharge_exit_v": state.vbat_v,
                                   "chrg_stat": state.chrg_stat})

            elif cmd == "CHARGE":
                target_v = min(step.get("param", 4.2),
                               limits.get("max_charge_v", 4.25))
                self.send_cmd(self.address, mode=1)  # CAN_MODE_CHARGE
                if state.full or state.mode_change_request or state.chrg_stat == 3:
                    self._advance({"final_v": state.vbat_v,
                                   "chrg_stat": state.chrg_stat,
                                   "target_v": target_v})

            elif cmd == "DISCHARGE":
                cutoff_v = max(step.get("param", 3.0),
                               limits.get("min_discharge_v", 3.0))
                duty = self.step_data.get("dsc_duty_pct", 80)
                self.send_cmd(self.address, mode=2,  # CAN_MODE_DISCHARGE
                              dsc_duty_pct=duty, dsc_pulse_ms=5000)
                # Integrate capacity
                if state.ibat_a < 0:
                    dt = now - self.step_data.get("_last_tick", now)
                    mah = self.step_data.get("capacity_mah", 0.0)
                    mah += abs(state.ibat_a) * dt / 3.6  # A·s → mAh
                    self.step_data["capacity_mah"] = mah
                self.step_data["_last_tick"] = now
                if state.vbat_v <= cutoff_v and state.vbat_v > BATTERY_PRESENT_V:
                    self.send_cmd(self.address, mode=0)  # IDLE
                    self._advance({"final_v": state.vbat_v,
                                   "cutoff_v": cutoff_v,
                                   "capacity_mah": self.step_data.get("capacity_mah", 0)})

            elif cmd == "MEASURE":
                # Request IR measurement frame from device
                if not self.step_data.get("measure_requested"):
                    self.send_cmd(self.address, mode=3)  # CAN_MODE_MEASURE
                    self.request_frame(self.address, 0x02)  # request telemetry type
                    self.step_data["measure_requested"] = True
                    self.step_data["measure_start"] = now
                # Wait for IR result in extended telemetry
                if state.ir_uohm > 0 and self.step_data.get("measure_requested"):
                    corrected = corrected_ir_uohm(
                        state.ir_uohm / 1000.0, state.vbat_v)
                    self._advance({"ir_uohm_raw": state.ir_uohm,
                                   "ir_uohm_corrected": corrected,
                                   "vbat_v": state.vbat_v})
                elif elapsed > 30:
                    # Timeout — record what we have
                    self._advance({"ir_uohm_raw": state.ir_uohm,
                                   "timeout": True})

            elif cmd == "SETTLE":
                duration_min = step.get("duration_min", 60)
                elapsed_min  = elapsed / 60.0
                remaining_min = max(0.0, duration_min - elapsed_min)
                self.step_data["remaining_min"] = remaining_min
                # Report to firmware (cap at 999)
                fw_remaining = min(999, int(remaining_min))
                self.send_cmd(self.address, mode=5,  # CAN_MODE_SETTLE
                              settle_minutes=fw_remaining)
                if remaining_min <= 0:
                    self._advance({"duration_min": duration_min,
                                   "vbat_at_end": state.vbat_v})

            elif cmd == "WAIT":
                duration_min = step.get("param", 1)
                if elapsed / 60.0 >= duration_min:
                    self._advance({"waited_min": duration_min})

            elif cmd in ("LOOP", "END"):
                if cmd == "LOOP":
                    self.step_idx = 0
                    self.step_start = time.time()
                    self.step_data = {}
                    db.update_session(self.session_id, current_step=0)
                else:
                    self.status = "done"
                    db.update_session(self.session_id, status="done",
                                      completed_at=time.time())

            return self._ui_state(state)

    def _ui_state(self, state: DeviceState) -> dict:
        step = self.steps[self.step_idx] if self.step_idx < len(self.steps) else {}
        remaining = self.step_data.get("remaining_min")
        return {
            "session_id":   self.session_id,
            "status":       self.status,
            "step_idx":     self.step_idx,
            "step_total":   len(self.steps),
            "step_cmd":     step.get("raw", ""),
            "remaining_min": remaining,
            "capacity_mah": self.step_data.get("capacity_mah"),
            "battery_dead": self.step_data.get("battery_dead", False),
        }

# ---------------------------------------------------------------------------
# Engine — manages all device sessions + CAN state
# ---------------------------------------------------------------------------
class Engine:
    def __init__(self, can_host):
        self.can_host = can_host
        self._sessions: Dict[int, DeviceSession] = {}   # address → session
        self._states: Dict[int, DeviceState] = {}       # address → DeviceState
        self._auto_program: Optional[dict] = None        # steps for auto mode
        self._auto_mode = False
        self._limits: dict = db.get_hard_limits()
        self._lock = threading.Lock()
        self._poll_thread = None
        self._running = False

    def start(self):
        self._running = True
        self._poll_thread = threading.Thread(target=self._poll_loop,
                                             daemon=True, name="engine-poll")
        self._poll_thread.start()
        log.info("Engine started")

    def stop(self):
        self._running = False

    def reload_limits(self):
        self._limits = db.get_hard_limits()

    # -------------------------------------------------------------------------
    # CAN state update (called by CAN RX thread)
    # -------------------------------------------------------------------------
    def on_status(self, status):
        """Called from bsafe_host.py BSafeHost RX thread with ChargerStatus."""
        addr = status.address
        with self._lock:
            if addr not in self._states:
                self._states[addr] = DeviceState(address=addr)
                db.log_event("device_seen", address=addr,
                             data={"first_seen": True})
            s = self._states[addr]
            prev_batt = s.batt_present
            prev_online = s.online

            s.mode         = int(status.mode)
            s.vbat_v       = status.vbat_v
            s.ibat_a       = status.ibat_a
            s.error        = status.error
            s.full         = status.full
            s.pwr_ok       = status.pwr_ok
            s.batt_present = status.batt_present
            s.mode_change_request = status.mode_change_request
            s.chrg_stat    = int(status.chrg_stat)
            s.dsc_pct      = status.dsc_pct
            s.last_rx      = time.time()
            s.online       = True

            session = self._sessions.get(addr)

            # Battery insertion/removal (only if not paused)
            if not (session and session.status == "paused"):
                vbat_below = s.vbat_v < BATTERY_PRESENT_V
                is_present = not vbat_below

                if is_present and not prev_batt:
                    db.log_event("battery_insert", address=addr,
                                 session_id=session.session_id if session else None,
                                 data={"vbat": s.vbat_v})
                    # Auto mode: start program if battery above DETECT threshold
                    if self._auto_mode and self._auto_program and not session:
                        self._start_session_locked(addr, self._auto_program)

                elif not is_present and prev_batt:
                    db.log_event("battery_remove", address=addr,
                                 session_id=session.session_id if session else None,
                                 data={"vbat": s.vbat_v})
                    # Stop running session on battery removal
                    if session and session.status == "running":
                        session.stop()

            # Device online/offline logging
            if not prev_online:
                db.log_event("device_online", address=addr,
                             data={"vbat": s.vbat_v})

    def on_telemetry(self, address: int, rpm: int, bq_temp_c: int,
                     ir_uohm: int, vbat_v: float, ibat_a: float):
        with self._lock:
            if address not in self._states:
                return
            s = self._states[address]
            s.rpm           = rpm
            s.bq_temp_c     = bq_temp_c
            s.ir_uohm       = ir_uohm
            s.last_telemetry_rx = time.time()
            session = self._sessions.get(address)
            db.log_telemetry(address,
                             session.session_id if session else None,
                             vbat_v, ibat_a, rpm, bq_temp_c, ir_uohm)

    def on_identity(self, address: int, schema_ver: int, hw_hash: int):
        with self._lock:
            if address not in self._states:
                self._states[address] = DeviceState(address=address)
            s = self._states[address]
            s.schema_ver  = schema_ver
            s.hw_ver_hash = hw_hash
        db.upsert_identity(address, schema_ver, hw_hash)
        db.log_event("identity_rx", address=address,
                     data={"schema": schema_ver, "hw_hash": hw_hash})

    # -------------------------------------------------------------------------
    # Poll loop
    # -------------------------------------------------------------------------
    def _poll_loop(self):
        OFFLINE_TIMEOUT = 10.0
        while self._running:
            now = time.time()
            with self._lock:
                for addr, s in list(self._states.items()):
                    # Offline detection
                    if s.online and (now - s.last_rx) > OFFLINE_TIMEOUT:
                        s.online = False
                        db.log_event("device_offline", address=addr,
                                     data={"last_rx": s.last_rx})
                        session = self._sessions.get(addr)
                        if session and session.status == "running":
                            session.stop()
                    # Tick sessions
                    session = self._sessions.get(addr)
                    if session and session.status == "running":
                        session.tick(s, self._limits)
                    # Clean up done/stopped sessions from active dict
                    if session and session.status in ("done", "stopped", "error"):
                        # Keep reference for UI, don't delete yet
                        pass
            time.sleep(0.5)

    # -------------------------------------------------------------------------
    # Session management
    # -------------------------------------------------------------------------
    def _start_session_locked(self, address: int, steps: list,
                               program_id=None, program_title=None,
                               program_body=None):
        """Must be called with self._lock held."""
        existing = self._sessions.get(address)
        if existing and existing.status in ("running", "paused"):
            return None  # already running

        session_id = db.create_session(
            address, program_id,
            program_title or "ad-hoc",
            program_body or json.dumps(steps),
        )
        session = DeviceSession(
            address=address,
            session_id=session_id,
            steps=steps,
            send_cmd_fn=self._send_cmd,
            request_frame_fn=self._request_frame,
        )
        self._sessions[address] = session
        db.log_event("program_start", address=address,
                     session_id=session_id,
                     data={"program": program_title, "steps": len(steps)})
        return session_id

    def start_program(self, addresses: List[int], steps: list,
                      program_id=None, program_title=None, program_body=None):
        started = []
        with self._lock:
            for addr in addresses:
                sid = self._start_session_locked(
                    addr, steps, program_id, program_title, program_body)
                if sid:
                    started.append(addr)
        return started

    def start_auto_mode(self, steps: list, program_id=None,
                        program_title=None, program_body=None):
        with self._lock:
            self._auto_mode    = True
            self._auto_program = steps
        log.info("Auto mode enabled")

    def stop_auto_mode(self):
        with self._lock:
            self._auto_mode    = False
            self._auto_program = None

    def stop_all(self):
        with self._lock:
            self.stop_auto_mode()
            for session in self._sessions.values():
                if session.status in ("running", "paused"):
                    session.stop()

    def pause_device(self, address: int):
        with self._lock:
            s = self._sessions.get(address)
            if s:
                s.pause()

    def resume_device(self, address: int):
        with self._lock:
            s = self._sessions.get(address)
            if s:
                s.resume()

    def stop_device(self, address: int):
        with self._lock:
            s = self._sessions.get(address)
            if s:
                s.stop()

    def unload_device(self, address: int):
        with self._lock:
            s = self._sessions.get(address)
            if s:
                s.stop()
            self._sessions.pop(address, None)

    # -------------------------------------------------------------------------
    # CAN wrappers
    # -------------------------------------------------------------------------
    def _send_cmd(self, address, mode, dsc_duty_pct=30,
                  dsc_pulse_ms=5000, settle_minutes=0,
                  req_immediate_vbat=False, req_frame_type=0):
        from bsafe_host import Mode as CanMode
        try:
            self.can_host.send_cmd(
                address       = address,
                mode          = CanMode(mode),
                dsc_duty_pct  = dsc_duty_pct,
                dsc_pulse_ms  = dsc_pulse_ms,
                settle_minutes= settle_minutes,
                req_immediate_vbat = req_immediate_vbat,
                req_frame_type= req_frame_type,
            )
        except Exception as e:
            log.warning(f"CAN send error [{address}]: {e}")

    def _request_frame(self, address: int, frame_type: int):
        self._send_cmd(address, mode=4,  # WAIT — don't change mode
                       req_frame_type=frame_type)

    # -------------------------------------------------------------------------
    # UI state snapshot
    # -------------------------------------------------------------------------
    def get_ui_snapshot(self) -> dict:
        with self._lock:
            devices = {}
            for addr, s in self._states.items():
                session = self._sessions.get(addr)
                sess_state = None
                if session:
                    sess_state = {
                        "session_id":    session.session_id,
                        "status":        session.status,
                        "step_idx":      session.step_idx,
                        "step_total":    len(session.steps),
                        "step_cmd":      session.current_step_raw or "",
                        "remaining_min": session.step_data.get("remaining_min"),
                        "capacity_mah":  session.step_data.get("capacity_mah"),
                        "battery_dead":  session.step_data.get("battery_dead", False),
                    }
                devices[addr] = {
                    "address":       addr,
                    "online":        s.online,
                    "mode":          s.mode,
                    "vbat_v":        round(s.vbat_v, 3),
                    "ibat_a":        round(s.ibat_a, 3),
                    "error":         s.error,
                    "full":          s.full,
                    "pwr_ok":        s.pwr_ok,
                    "batt_present":  s.batt_present,
                    "chrg_stat":     s.chrg_stat,
                    "dsc_pct":       s.dsc_pct,
                    "rpm":           s.rpm,
                    "bq_temp_c":     s.bq_temp_c,
                    "ir_uohm":       s.ir_uohm,
                    "schema_ver":    s.schema_ver,
                    "hw_ver_hash":   s.hw_ver_hash,
                    "last_rx":       s.last_rx,
                    "session":       sess_state,
                }
            return {
                "devices":    devices,
                "auto_mode":  self._auto_mode,
                "device_count": len(devices),
                "online_count": sum(1 for s in self._states.values() if s.online),
            }
