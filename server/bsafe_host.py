#!/usr/bin/env python3
# =============================================================================
# bsafe_host.py — bSafe V1 CAN host controller
# Runs on Raspberry Pi with MCP2515 via can0 (python-can)
#
# STATUS frames  charger → host  ID = 0x100 + address  (8 bytes)
# All STATUS frames share byte [0] as frame_type:
#
#   Type 0x00 — Operational (autonomous 2 Hz)
#     [0]   frame_type = 0x00
#     [1-2] vbat_mv    uint16 BE
#     [3-4] ibat_ma    int16  BE (signed)
#     [5]   flags      bit0=error  bit1=full  bit2=pwr_ok
#                      bit3=batt_present  bit4=mode_change_request
#     [6]   chrg_stat  0=none 1=pre 2=fast 3=done
#     [7]   dsc_pct    0-100
#
#   Type 0x01 — Extended telemetry (on request)
#     [0]   frame_type = 0x01
#     [1-2] vbat_mv    uint16 BE
#     [3-4] ibat_ma    int16  BE (signed)
#     [5]   rpm/50     uint8  (multiply by 50 for actual RPM, 0-12750)
#     [6]   bq_temp_c  int8
#     [7]   reserved
#
#   Type 0x02 — IR + MCU temp (on request)
#     [0]   frame_type = 0x02
#     [1-4] ir_uohm    uint32 BE (raw, host applies Rdson correction)
#     [5]   mcu_temp_c int8
#     [6-7] reserved
#
#   Type 0x03 — Identity (on request + CAN page entry)
#     [0]   frame_type = 0x03
#     [1]   schema_version
#     [2]   min_compat_schema
#     [3]   hw_ver_hash  (FNV of hw_version string, low byte)
#     [4]   fw_major
#     [5]   fw_minor
#     [6-7] fw_build    uint16 BE
#
# CMD frame  host → charger  ID = 0x200 + address  (8 bytes)
#   [0]   mode byte         0=IDLE 1=CHARGE 2=DISCHARGE 3=MEASURE 4=WAIT 5=SETTLE
#   [1]   dsc_duty_pct      0-100
#   [2-3] dsc_pulse_ms      uint16 BE
#   [4-5] settle_minutes    uint16 BE (0-999)
#   [6]   flags             bits[2:0]=req_frame_type  bit[3]=request_flag
#   [7]   precharge_thresh  0=no change  1=2.8V  2=3.0V
# =============================================================================

import can
import struct
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict
from enum import IntEnum

# ---------------------------------------------------------------------------
# Shared frame constants, enums, and dataclasses — imported from bsafe_frames
# (no CAN dependency there; WiFi host imports directly from bsafe_frames too)
# ---------------------------------------------------------------------------
from bsafe_frames import (
    CAN_ID_STATUS_BASE, CAN_ID_CMD_BASE,
    FRAME_TYPE_OP, FRAME_TYPE_TELEM, FRAME_TYPE_IR, FRAME_TYPE_IDENTITY,
    Mode, ChrgStat,
    FLAG_ERROR, FLAG_FULL, FLAG_PWR_OK, FLAG_BATT_PRESENT, FLAG_MODE_CHANGE_REQUEST,
    circuit_resistance_mohm, corrected_ir_uohm,
    OperationalStatus, TelemetryStatus, IrStatus, IdentityStatus,
    ChargerStatus,
)


# ---------------------------------------------------------------------------
# CMD frame builder
# ---------------------------------------------------------------------------

def _build_cmd_bytes(
    mode:               Mode  = Mode.WAIT,
    dsc_duty_pct:       int   = 30,
    dsc_pulse_ms:       int   = 5000,
    settle_minutes:     int   = 0,
    req_frame_type:     int   = 0,    # bits[2:0] — which frame type to request
    request_flag:       bool  = False, # bit[3]   — set to trigger TX on device
    precharge_thresh:   int   = 0,    # 0=no change  1=2.8V  2=3.0V
) -> bytes:
    """
    Pack an 8-byte CMD frame matching firmware _can_parse_cmd().
      [0]   mode
      [1]   dsc_duty_pct
      [2-3] dsc_pulse_ms  (uint16 BE)
      [4-5] settle_minutes (uint16 BE, 0-999)
      [6]   bits[2:0]=req_frame_type  bit[3]=request_flag
      [7]   precharge_thresh (0=no change, 1=2.8V, 2=3.0V)
    """
    dsc_duty_pct   = max(0, min(100, dsc_duty_pct))
    dsc_pulse_ms   = max(0, min(65535, dsc_pulse_ms))
    settle_minutes = max(0, min(999, settle_minutes))
    flags = (req_frame_type & 0x07) | (0x08 if request_flag else 0x00)
    data = bytearray(8)
    data[0] = int(mode)
    data[1] = dsc_duty_pct
    data[2] = (dsc_pulse_ms >> 8) & 0xFF
    data[3] = dsc_pulse_ms & 0xFF
    data[4] = (settle_minutes >> 8) & 0xFF
    data[5] = settle_minutes & 0xFF
    data[6] = flags
    data[7] = precharge_thresh & 0x03
    return bytes(data)


# ---------------------------------------------------------------------------
# BSafeHost — manages one CAN bus, N chargers by address
# ---------------------------------------------------------------------------

class BSafeHost:
    def __init__(self, channel: str = 'can0', bitrate: int = 500000):
        self._channel = channel
        self._bitrate = bitrate
        self._bus: Optional[can.BusABC] = None
        self._lock    = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False

        # Per-address latest frames
        self._op:       Dict[int, OperationalStatus] = {}
        self._telem:    Dict[int, TelemetryStatus]   = {}
        self._ir:       Dict[int, IrStatus]           = {}
        self._identity: Dict[int, IdentityStatus]     = {}

        # Callbacks
        self._on_op:       Dict[int, Callable] = {}
        self._on_telem:    Dict[int, Callable] = {}
        self._on_ir:       Dict[int, Callable] = {}
        self._on_identity: Dict[int, Callable] = {}
        self._on_any_frame:  Optional[Callable] = None  # called for every frame
        self._on_any_status: Optional[Callable] = None  # legacy: OP frames only

        self.log = logging.getLogger('BSafeHost')

    # -------------------------------------------------------------------------
    # Connection
    # -------------------------------------------------------------------------
    def connect(self):
        self._bus = can.interface.Bus(
            channel=self._channel,
            bustype='socketcan',
            bitrate=self._bitrate,
        )
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name='bsafe-rx')
        self._rx_thread.start()
        self.log.info(f"Connected to {self._channel} @ {self._bitrate} bps")

    def disconnect(self):
        self._running = False
        if self._bus:
            self._bus.shutdown()
            self._bus = None
        self.log.info("Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    # -------------------------------------------------------------------------
    # RX loop
    # -------------------------------------------------------------------------
    def _rx_loop(self):
        while self._running:
            try:
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue
                arb = msg.arbitration_id
                if not (CAN_ID_STATUS_BASE <= arb < CAN_ID_STATUS_BASE + 0x40):
                    continue
                address   = arb - CAN_ID_STATUS_BASE
                data      = msg.data
                if len(data) < 1:
                    continue
                frame_type = data[0]
                self._dispatch(address, frame_type, data)
            except can.CanError as e:
                if self._running:
                    self.log.error(f"CAN rx error: {e}")
                    time.sleep(0.05)
            except Exception as e:
                self.log.warning(f"RX parse error: {e}")

    def _dispatch(self, address: int, frame_type: int, data: bytes):
        try:
            if frame_type == FRAME_TYPE_OP:
                frame = OperationalStatus.parse(address, data)
                with self._lock:
                    self._op[address] = frame
                if cb := self._on_op.get(address): cb(frame)
                if self._on_any_status: self._on_any_status(frame)

            elif frame_type == FRAME_TYPE_TELEM:
                frame = TelemetryStatus.parse(address, data)
                with self._lock:
                    self._telem[address] = frame
                if cb := self._on_telem.get(address): cb(frame)

            elif frame_type == FRAME_TYPE_IR:
                frame = IrStatus.parse(address, data)
                with self._lock:
                    self._ir[address] = frame
                if cb := self._on_ir.get(address): cb(frame)

            elif frame_type == FRAME_TYPE_IDENTITY:
                frame = IdentityStatus.parse(address, data)
                with self._lock:
                    self._identity[address] = frame
                if cb := self._on_identity.get(address): cb(frame)

            else:
                self.log.debug(f"Unknown frame type 0x{frame_type:02X} from [{address}]")
                return

            if self._on_any_frame:
                self._on_any_frame(address, frame_type, frame)

        except Exception as e:
            self.log.warning(f"Frame parse error [{address}] type=0x{frame_type:02X}: {e}")

    # -------------------------------------------------------------------------
    # Send CMD
    # -------------------------------------------------------------------------
    def send_cmd(self, address: int, mode: Mode,
                 dsc_duty_pct:     int  = 30,
                 dsc_pulse_ms:     int  = 5000,
                 settle_minutes:   int  = 0,
                 req_frame_type:   int  = 0,
                 request_flag:     bool = False,
                 precharge_thresh: int  = 0):
        if not self._bus:
            raise RuntimeError("Not connected")
        data = _build_cmd_bytes(
            mode             = mode,
            dsc_duty_pct     = dsc_duty_pct,
            dsc_pulse_ms     = dsc_pulse_ms,
            settle_minutes   = settle_minutes,
            req_frame_type   = req_frame_type,
            request_flag     = request_flag,
            precharge_thresh = precharge_thresh,
        )
        msg = can.Message(
            arbitration_id = CAN_ID_CMD_BASE + (address & 0x3F),
            data           = data,
            is_extended_id = False,
        )
        with self._lock:
            self._bus.send(msg)
        self.log.debug(
            f"CMD → [{address}] mode={mode.name} duty={dsc_duty_pct}% "
            f"pulse={dsc_pulse_ms}ms settle={settle_minutes}m "
            f"req_ft={req_frame_type} req={request_flag} pre={precharge_thresh}")

    # -------------------------------------------------------------------------
    # Frame request helpers
    # -------------------------------------------------------------------------
    def request_frame(self, address: int, frame_type: int):
        """Ask device to immediately transmit a specific STATUS frame type."""
        s = self.get_op(address)
        mode = s.mode if s else Mode.WAIT
        # Preserve current mode, just set request bits
        self.send_cmd(address, mode,
                      req_frame_type=frame_type, request_flag=True)

    def request_telemetry(self, address: int):
        self.request_frame(address, FRAME_TYPE_TELEM)

    def request_ir(self, address: int):
        self.request_frame(address, FRAME_TYPE_IR)

    def request_identity(self, address: int):
        self.request_frame(address, FRAME_TYPE_IDENTITY)

    # -------------------------------------------------------------------------
    # Convenience mode wrappers
    # -------------------------------------------------------------------------
    def cmd_charge(self, address: int, precharge_thresh: int = 0):
        """
        precharge_thresh: 0=no change, 1=set BATLOWV to 2.8V, 2=set to 3.0V
        """
        self.send_cmd(address, Mode.CHARGE, precharge_thresh=precharge_thresh)

    def cmd_precharge(self, address: int, threshold_v: float = 3.0):
        """
        Start charge with BATLOWV set to threshold_v (2.8 or 3.0 only).
        BQ25895 will pre-charge at reduced current until BATLOWV is reached,
        then automatically transition to fast charge.
        """
        thresh_code = 2 if threshold_v >= 3.0 else 1
        self.send_cmd(address, Mode.CHARGE, precharge_thresh=thresh_code)

    def cmd_discharge(self, address: int, duty_pct: int = 30, pulse_ms: int = 5000):
        self.send_cmd(address, Mode.DISCHARGE,
                      dsc_duty_pct=duty_pct, dsc_pulse_ms=pulse_ms)

    def cmd_settle(self, address: int, minutes: int):
        self.send_cmd(address, Mode.SETTLE, settle_minutes=min(999, minutes))

    def cmd_measure(self, address: int):
        self.send_cmd(address, Mode.MEASURE)

    def cmd_idle(self, address: int):
        self.send_cmd(address, Mode.IDLE)

    def cmd_wait(self, address: int):
        self.send_cmd(address, Mode.WAIT)

    # -------------------------------------------------------------------------
    # Status accessors
    # -------------------------------------------------------------------------
    def get_op(self, address: int) -> Optional[OperationalStatus]:
        with self._lock:
            return self._op.get(address)

    def get_telem(self, address: int) -> Optional[TelemetryStatus]:
        with self._lock:
            return self._telem.get(address)

    def get_ir(self, address: int) -> Optional[IrStatus]:
        with self._lock:
            return self._ir.get(address)

    def get_identity(self, address: int) -> Optional[IdentityStatus]:
        with self._lock:
            return self._identity.get(address)

    # Legacy alias used by engine.py
    def get_status(self, address: int) -> Optional[OperationalStatus]:
        return self.get_op(address)

    def get_all_statuses(self) -> Dict[int, OperationalStatus]:
        with self._lock:
            return dict(self._op)

    def is_online(self, address: int, timeout_s: float = 5.0) -> bool:
        s = self.get_op(address)
        return s is not None and (time.time() - s.rx_time) < timeout_s

    def wait_for_charger(self, address: int, timeout_s: float = 10.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.get_op(address) is not None:
                return True
            time.sleep(0.1)
        return False

    def known_addresses(self) -> list:
        with self._lock:
            return sorted(self._op.keys())

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def on_operational(self, address: int, callback: Callable):
        self._on_op[address] = callback

    def on_telemetry(self, address: int, callback: Callable):
        self._on_telem[address] = callback

    def on_ir(self, address: int, callback: Callable):
        self._on_ir[address] = callback

    def on_identity_rx(self, address: int, callback: Callable):
        self._on_identity[address] = callback

    def on_any_status(self, callback: Callable):
        """Legacy: called for every OP frame. Also used by engine.py."""
        self._on_any_status = callback

    def on_any_frame(self, callback: Callable):
        """Called for every STATUS frame of any type: (address, frame_type, frame)."""
        self._on_any_frame = callback

    # -------------------------------------------------------------------------
    # High-level: full charge cycle with settle
    # -------------------------------------------------------------------------
    def run_charge_cycle(self, address: int, settle_minutes: int = 60,
                         precharge_thresh: int = 2,
                         poll_interval_s: float = 2.0,
                         on_update: Optional[Callable] = None):
        """
        Blocking full charge cycle:
          1. CHARGE (with optional BATLOWV setting for pre-charge)
          2. Poll until full flag or error
          3. SETTLE with countdown
          4. IDLE
        """
        self.log.info(f"[{address}] Starting charge cycle (settle={settle_minutes}m)")
        self.cmd_charge(address, precharge_thresh=precharge_thresh)

        self.log.info(f"[{address}] Phase 1: CHARGING")
        while True:
            time.sleep(poll_interval_s)
            s = self.get_op(address)
            if s is None:
                self.log.warning(f"[{address}] No OP status — charger offline?")
                continue
            if on_update:
                on_update(s)
            if s.error:
                self.log.error(f"[{address}] Charge error — aborting")
                self.cmd_idle(address)
                return
            if s.full or s.mode_change_request:
                self.log.info(f"[{address}] Charge complete at {s.vbat_v:.3f}V")
                break

        self.log.info(f"[{address}] Phase 2: SETTLING ({settle_minutes}m)")
        remaining = settle_minutes
        self.cmd_settle(address, remaining)
        while remaining > 0:
            time.sleep(60)
            remaining -= 1
            self.cmd_settle(address, remaining)
            s = self.get_op(address)
            if s and on_update:
                on_update(s)
            self.log.info(
                f"[{address}] Settling: {remaining}m left  "
                + (f"Vbat={s.vbat_v:.3f}V" if s else ""))

        self.cmd_idle(address)
        self.log.info(f"[{address}] Cycle complete")


# ---------------------------------------------------------------------------
# CLI demo
# ---------------------------------------------------------------------------
def _demo():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )
    log = logging.getLogger('demo')

    with BSafeHost(channel='can0', bitrate=500000) as host:
        log.info("Listening for chargers... (Ctrl+C to stop)")
        log.info("Will request telemetry + IR from any device that appears.")

        seen = set()

        def on_op(s: OperationalStatus):
            print(f"  {s}")
            if s.address not in seen:
                seen.add(s.address)
                # Request identity and extended telemetry on first contact
                host.request_identity(s.address)
                host.request_telemetry(s.address)

        def on_telem(address, _, frame):
            if isinstance(frame, TelemetryStatus):
                print(f"  {frame}")

        def on_ir(address, _, frame):
            if isinstance(frame, IrStatus):
                op = host.get_op(address)
                vbat = op.vbat_v if op else 3.7
                corr = frame.corrected_ir(vbat)
                print(f"  {frame}  corrected={corr:.0f}µΩ")

        def on_id(address, _, frame):
            if isinstance(frame, IdentityStatus):
                print(f"  {frame}")

        host.on_any_status(on_op)
        host.on_any_frame(lambda addr, ft, fr: (
            on_telem(addr, ft, fr) if ft == FRAME_TYPE_TELEM else
            on_ir(addr, ft, fr)    if ft == FRAME_TYPE_IR     else
            on_id(addr, ft, fr)    if ft == FRAME_TYPE_IDENTITY else None
        ))

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            log.info("Stopped")


if __name__ == '__main__':
    _demo()
