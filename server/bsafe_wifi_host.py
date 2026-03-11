"""
bsafe_wifi_host.py — TCP server for WiFi-connected bSafe devices.

Frame format over TCP (same binary payload as CAN STATUS/CMD frames):
  [length : 2 bytes LE] [frame_type : 1 byte] [address : 1 byte] [payload : N bytes]

STATUS frame types (device → host):
  0x00  OP status       — OperationalStatus  (autonomous, 2 Hz)
  0x01  Telemetry       — TelemetryStatus    (on request)
  0x02  IR result       — IrStatus           (on request)
  0x03  Identity        — IdentityStatus     (on request)
  0x12  WIFI_NET_REPORT — one known network  (ssid only, repeats for each)

CMD frame types (host → device):
  0x80  CMD             — mode/request command (same payload as CAN CMD)
  0x10  WIFI_CFG_PUSH   — push one network entry {idx, ssid, password}
  0x11  WIFI_CFG_ACK    — ack from device {idx, result}  (device→host)
  0x13  WIFI_NET_REQ    — request full network list       (host→device)

On connect:
  1. Host sends 0x13 (WIFI_NET_REQ) — device reports all known networks as 0x12 frames
  2. Host diffs against master wifi_networks list in db
  3. Host pushes missing entries as 0x10 (WIFI_CFG_PUSH), one per entry
  4. Each pushed entry acknowledged by device with 0x11 (WIFI_CFG_ACK)
  5. Host marks device sync_status=synced in db

All engine callbacks (on_status, on_telemetry, etc.) are identical to CAN host —
the engine sees no difference between a WiFi device and a CAN device.
"""

import struct
import time
import socket
import threading
import logging
from typing import Optional, Callable, Dict

import db
from bsafe_frames import (
    OperationalStatus, TelemetryStatus, IrStatus, IdentityStatus,
    ChrgStat, Mode,
    FRAME_TYPE_OP, FRAME_TYPE_TELEM, FRAME_TYPE_IR, FRAME_TYPE_IDENTITY,
)

log = logging.getLogger("wifi_host")

# WiFi-only frame types
FRAME_WIFI_IDENTITY   = 0x04   # device → host: identity + 6-byte WiFi MAC appended
FRAME_WIFI_CFG_PUSH   = 0x10   # host → device: push network entry
FRAME_WIFI_CFG_ACK    = 0x11   # device → host: ack/nack
FRAME_WIFI_NET_REPORT = 0x12   # device → host: one known network
FRAME_WIFI_NET_REQ    = 0x13   # host → device: request full network list

FRAME_CMD             = 0x80   # host → device: mode/request command

# TCP framing
HDR_FMT    = "<HBB"   # length(2LE), frame_type(1), address(1)
HDR_SIZE   = struct.calcsize(HDR_FMT)   # 4 bytes
MAX_FRAME  = 512


# ---------------------------------------------------------------------------
# Frame builders
# ---------------------------------------------------------------------------
def _build_frame(frame_type: int, address: int, payload: bytes) -> bytes:
    header = struct.pack(HDR_FMT, len(payload), frame_type, address)
    return header + payload


def _build_cmd_frame(address: int, mode: int, dsc_duty_pct: int = 30,
                     dsc_pulse_ms: int = 5000, settle_minutes: int = 0,
                     request_flag: bool = False, req_frame_type: int = 0) -> bytes:
    """Mirror of CAN CMD frame payload for TCP transport."""
    flags = (req_frame_type & 0x07) | (0x08 if request_flag else 0x00)
    payload = struct.pack(">BBHIB",
                          mode & 0xFF,
                          dsc_duty_pct & 0xFF,
                          dsc_pulse_ms & 0xFFFF,
                          settle_minutes & 0xFFFFFFFF,
                          flags)
    return _build_frame(FRAME_CMD, address, payload)


def _build_net_req(address: int) -> bytes:
    return _build_frame(FRAME_WIFI_NET_REQ, address, b"")


def _build_cfg_push(address: int, idx: int, ssid: str, password: str) -> bytes:
    """Push one network entry. SSID max 32 bytes, password max 64 bytes."""
    ssid_b = ssid.encode()[:32].ljust(33, b"\x00")
    pass_b = password.encode()[:64].ljust(65, b"\x00")
    payload = struct.pack("B", idx & 0xFF) + ssid_b + pass_b
    return _build_frame(FRAME_WIFI_CFG_PUSH, address, payload)


# ---------------------------------------------------------------------------
# Per-device TCP session
# ---------------------------------------------------------------------------
class DeviceConnection:
    """
    Manages one TCP connection from one device.
    Runs an RX loop in a real OS thread (not green thread — same reason as CAN).
    """

    def __init__(self, sock: socket.socket, addr_str: str,
                 wifi_host: "BSafeWiFiHost"):
        self._sock      = sock
        self._addr_str  = addr_str   # "ip:port"
        self._host      = wifi_host
        self._device_addr: Optional[int] = None   # set from first frame
        self._device_addr_mac: str = ""            # set when WIFI_IDENTITY received
        self._lock      = threading.Lock()
        self._running   = False
        self._buf       = b""
        self._log       = logging.getLogger(f"wifi_conn[{addr_str}]")
        self._sync_done = False

    def start(self):
        self._running = True
        try:
            import eventlet.patcher
            RealThread = eventlet.patcher.original("threading").Thread
        except Exception:
            RealThread = threading.Thread
        t = RealThread(target=self._rx_loop, daemon=True,
                       name=f"wifi-rx-{self._addr_str}")
        t.start()

    def close(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass

    def send_raw(self, data: bytes):
        with self._lock:
            try:
                self._sock.sendall(data)
            except Exception as e:
                self._log.warning(f"Send error: {e}")
                self._running = False

    def send_cmd(self, mode: int, **kwargs):
        addr = self._device_addr or 0
        frame = _build_cmd_frame(addr, mode, **kwargs)
        self.send_raw(frame)

    def request_identity(self):
        """Ask device to re-send its WIFI_IDENTITY frame (0x04) including MAC."""
        self.send_cmd(mode=4, request_flag=True,
                      req_frame_type=FRAME_TYPE_IDENTITY)  # 0x03 → firmware sends 0x04

    def request_network_list(self):
        addr = self._device_addr or 0
        self.send_raw(_build_net_req(addr))

    def push_network(self, idx: int, ssid: str, password: str):
        addr = self._device_addr or 0
        self.send_raw(_build_cfg_push(addr, idx, ssid, password))

    # -------------------------------------------------------------------------
    # RX loop
    # -------------------------------------------------------------------------
    def _rx_loop(self):
        self._log.info("RX loop started")
        while self._running:
            try:
                chunk = self._sock.recv(256)
                if not chunk:
                    break
                self._buf += chunk
                self._drain_frames()
            except OSError:
                break
            except Exception as e:
                self._log.warning(f"RX error: {e}")
                time.sleep(0.05)

        self._log.info("Connection closed")
        self._host._on_disconnect(self)

    def _drain_frames(self):
        while len(self._buf) >= HDR_SIZE:
            length, ftype, address = struct.unpack_from(HDR_FMT, self._buf)
            total = HDR_SIZE + length
            if len(self._buf) < total:
                break   # wait for more data
            payload    = self._buf[HDR_SIZE:total]
            self._buf  = self._buf[total:]

            # First frame from this connection sets the device address
            if self._device_addr is None:
                self._device_addr = address
                self._log.info(f"Device address identified: {address}")
                db.upsert_device_wifi_state(
                    address,
                    transport="wifi",
                    ip_addr=self._addr_str.split(":")[0],
                    last_wifi_seen=time.time())
                # Register with host so engine can send commands
                self._host._register(address, self)
                # Start sync after address known
                threading.Thread(target=self._do_sync, daemon=True,
                                 name=f"wifi-sync-{address}").start()

            self._dispatch(ftype, address, payload)

    def _dispatch(self, ftype: int, address: int, payload: bytes):
        host = self._host
        try:
            if ftype == FRAME_TYPE_OP:
                frame = OperationalStatus.parse(address, payload)
                if host._on_any_status:
                    host._on_any_status(frame)

            elif ftype == FRAME_TYPE_TELEM:
                frame = TelemetryStatus.parse(address, payload)
                if host._on_telem:
                    # TelemetryStatus carries rpm, bq_temp, vbat, ibat — no ir_uohm
                    # (ir_uohm arrives separately via FRAME_TYPE_IR → _on_ir)
                    host._on_telem(address, frame.rpm, frame.bq_temp_c,
                                   0, frame.vbat_v, frame.ibat_a)

            elif ftype == FRAME_TYPE_IR:
                frame = IrStatus.parse(address, payload)
                if host._on_ir:
                    host._on_ir(address, frame)

            elif ftype == FRAME_TYPE_IDENTITY:
                frame = IdentityStatus.parse(address, payload)
                if host._on_identity:
                    host._on_identity(address, frame.schema_version, frame.hw_ver_hash)

            elif ftype == FRAME_WIFI_IDENTITY:
                # payload[0:8] = standard identity bytes (payload[0]=frame_type, [1..7]=identity)
                # payload[8:14] = 6-byte WiFi STA MAC
                self._log.info(f"WIFI_IDENTITY: addr={address} len={len(payload)} raw={payload.hex()}")
                frame = IdentityStatus.parse(address, payload[:8])
                if host._on_identity:
                    host._on_identity(address, frame.schema_version, frame.hw_ver_hash)
                if len(payload) >= 14 and host._on_wifi_identity:
                    mac_str = ":".join(f"{b:02X}" for b in payload[8:14])
                    self._log.info(f"WiFi MAC: {mac_str} (addr={address})")
                    self._device_addr_mac = mac_str
                    host._on_wifi_identity(address, mac_str)
                else:
                    self._log.warning(f"WIFI_IDENTITY too short or no callback: len={len(payload)} cb={host._on_wifi_identity}")

            elif ftype == FRAME_WIFI_NET_REPORT:
                # payload: [idx:1][ssid:33 null-padded]
                if len(payload) >= 34:
                    idx  = payload[0]
                    ssid = payload[1:34].rstrip(b"\x00").decode(errors="replace")
                    self._host._on_net_report(address, idx, ssid)

            elif ftype == FRAME_WIFI_CFG_ACK:
                if len(payload) >= 2:
                    idx, result = payload[0], payload[1]
                    self._log.info(f"CFG ACK idx={idx} result={result}")
                    self._host._on_cfg_ack(address, idx, result)

            else:
                self._log.debug(f"Unknown frame type 0x{ftype:02X}")

        except Exception as e:
            self._log.warning(f"Dispatch error ftype=0x{ftype:02X}: {e}")

    # -------------------------------------------------------------------------
    # Network sync
    # -------------------------------------------------------------------------
    def _do_sync(self):
        """Called once after device address identified. Syncs network list."""
        addr = self._device_addr
        if addr is None:
            return

        self._log.info("Starting network sync")
        db.upsert_device_wifi_state(addr, sync_status="syncing")

        # 0. Re-request identity if MAC not yet stored (belt-and-suspenders:
        #    the WIFI_IDENTITY frame should already have arrived on connect,
        #    but request it again in case it was missed or the db row is new)
        import db as _db
        state = _db.get_device_wifi_state(addr)
        if not state or not state.get("mac"):
            self._log.info(f"[{addr}] MAC not yet known — requesting identity")
            self.request_identity()
            time.sleep(0.3)   # give device time to respond before network sync

        # 1. Request device's current network list
        self.request_network_list()

        # 2. Wait briefly for WIFI_NET_REPORT frames to arrive
        time.sleep(2.0)

        # 3. Diff master list vs device's known networks
        master   = db.get_wifi_networks()
        state    = db.get_device_wifi_state(addr)
        known_raw = state.get("known_networks") or "[]"
        try:
            import json
            known_ssids = set(json.loads(known_raw))
        except Exception:
            known_ssids = set()

        to_push = [n for n in master if n["ssid"] not in known_ssids]

        if not to_push:
            self._log.info("Device already has all networks — synced")
            db.upsert_device_wifi_state(addr, sync_status="synced")
            return

        self._log.info(f"Pushing {len(to_push)} network(s) to device {addr}")
        for idx, net in enumerate(to_push):
            self.push_network(idx, net["ssid"], net["password"])
            time.sleep(0.3)   # brief gap between pushes

        # Optimistically mark synced — acks update individual entries
        db.upsert_device_wifi_state(addr, sync_status="synced")
        self._sync_done = True


# ---------------------------------------------------------------------------
# BSafeWiFiHost — TCP server
# ---------------------------------------------------------------------------
class BSafeWiFiHost:
    """
    Drop-in companion to BSafeHost (CAN). Same callback interface.
    engine.py calls engine._send_cmd → host.send_cmd() → routed to right transport.
    """

    def __init__(self, bind_ip: str = "0.0.0.0", port: int = 7000):
        self._bind_ip    = bind_ip
        self._port       = port
        self._running    = False
        self._server     = None
        self._conns: Dict[int, DeviceConnection] = {}   # address → connection
        self._lock       = threading.Lock()

        # Callbacks — same signature as BSafeHost
        self._on_any_status: Optional[Callable] = None
        self._on_telem:      Optional[Callable] = None
        self._on_ir:         Optional[Callable] = None
        self._on_identity:   Optional[Callable] = None

        # Per-device net-report accumulator
        self._net_reports: Dict[int, list] = {}   # address → [ssid, ...]

        # Outbound retry management: address → Event (set = stop retrying)
        self._outbound_stops: Dict[int, threading.Event] = {}

    # -------------------------------------------------------------------------
    # Real-thread helper (bypasses eventlet monkey-patching for blocking I/O)
    # -------------------------------------------------------------------------
    def _make_thread(self, **kwargs) -> threading.Thread:
        try:
            import eventlet.patcher
            RealThread = eventlet.patcher.original("threading").Thread
        except Exception:
            RealThread = threading.Thread
        return RealThread(**kwargs)

    # -------------------------------------------------------------------------
    # Callback registration (mirrors BSafeHost API)
    # -------------------------------------------------------------------------
    def on_any_status(self, cb: Callable):
        self._on_any_status = cb

    def on_telemetry(self, cb: Callable):
        self._on_telem = cb

    def on_ir(self, cb: Callable):
        self._on_ir = cb

    def on_wifi_identity(self, cb: Callable):
        """cb(address: int, mac_str: str) — called when device sends its WiFi MAC."""
        self._on_wifi_identity = cb

    def on_identity(self, cb: Callable):
        self._on_identity = cb

    # -------------------------------------------------------------------------
    # Server lifecycle
    # -------------------------------------------------------------------------
    def start(self):
        self._running = True
        t = self._make_thread(target=self._accept_loop, daemon=True,
                              name="wifi-accept")
        t.start()
        log.info(f"WiFi TCP server on {self._bind_ip}:{self._port}")

    def stop(self):
        self._running = False
        if self._server:
            try:
                self._server.close()
            except Exception:
                pass
        with self._lock:
            for conn in list(self._conns.values()):
                conn.close()
            self._conns.clear()

    def rebind(self, new_ip: str):
        """Called by NetworkManager when IP changes (mode switch)."""
        log.info(f"WiFi host rebinding to {new_ip}:{self._port}")
        if self._server:
            try:
                self._server.close()
            except Exception:
                pass
        self._bind_ip = new_ip
        t = self._make_thread(target=self._accept_loop, daemon=True,
                              name="wifi-accept-rebind")
        t.start()

    def _accept_loop(self):
        try:
            self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((self._bind_ip, self._port))
            self._server.listen(32)
            log.info(f"WiFi TCP listening on {self._bind_ip}:{self._port}")
            while self._running:
                try:
                    self._server.settimeout(2.0)
                    sock, addr = self._server.accept()
                    addr_str = f"{addr[0]}:{addr[1]}"
                    log.info(f"Device connected from {addr_str}")
                    conn = DeviceConnection(sock, addr_str, self)
                    conn.start()
                except socket.timeout:
                    continue
                except OSError:
                    break
        except Exception as e:
            log.error(f"Accept loop error: {e}", exc_info=True)

    # -------------------------------------------------------------------------
    # CMD sending (called by engine via host.send_cmd)
    # -------------------------------------------------------------------------
    def send_cmd(self, address: int, mode, dsc_duty_pct: int = 30,
                 dsc_pulse_ms: int = 5000, settle_minutes: int = 0,
                 request_flag: bool = False, req_frame_type: int = 0,
                 precharge_thresh: int = 0):
        mode_int = int(mode)
        with self._lock:
            conn = self._conns.get(address)
        if conn:
            conn.send_cmd(
                mode_int,
                dsc_duty_pct=dsc_duty_pct,
                dsc_pulse_ms=dsc_pulse_ms,
                settle_minutes=settle_minutes,
                request_flag=request_flag,
                req_frame_type=req_frame_type)
        else:
            log.debug(f"send_cmd: no WiFi connection for address {address}")

    def connected_addresses(self) -> list:
        with self._lock:
            return list(self._conns.keys())

    def is_connected(self, address: int) -> bool:
        with self._lock:
            return address in self._conns

    def push_networks_to_device(self, address: int):
        """Manually trigger a network sync to a specific device."""
        with self._lock:
            conn = self._conns.get(address)
        if conn:
            threading.Thread(target=conn._do_sync, daemon=True).start()

    # -------------------------------------------------------------------------
    # mDNS-driven outbound connect (called by mdns_discovery.MdnsBrowser)
    # -------------------------------------------------------------------------
    def notify_discovered(self, address: int, ip: str, port: int,
                          hostname: str):
        """
        Called when the mDNS browser discovers a device.

        If the device is already connected inbound, just update the DB metadata
        and return — no second connection is opened.  Otherwise start an
        outbound TCP retry loop with exponential backoff.
        """
        if self.is_connected(address):
            log.info(f"mDNS addr={address} already connected inbound; "
                     f"updating hostname={hostname}")
            db.upsert_device_wifi_state(address, hostname=hostname)
            return

        # Cancel any previous retry loop for this address before starting fresh
        with self._lock:
            old_stop = self._outbound_stops.get(address)
            if old_stop:
                old_stop.set()
            stop_evt = threading.Event()
            self._outbound_stops[address] = stop_evt

        t = self._make_thread(
            target=self._outbound_retry,
            args=(address, ip, port, hostname, stop_evt),
            daemon=True,
            name=f"wifi-out-{address}",
        )
        t.start()
        log.info(f"mDNS: starting outbound connect to addr={address} "
                 f"({hostname} @ {ip}:{port})")

    def notify_removed(self, address: int):
        """
        Called when the mDNS browser loses a device advertisement.
        Cancels any pending outbound retry for that address.
        """
        with self._lock:
            stop_evt = self._outbound_stops.pop(address, None)
        if stop_evt:
            stop_evt.set()
            log.info(f"mDNS: addr={address} removed — outbound retry cancelled")

    def _outbound_retry(self, address: int, ip: str, port: int,
                        hostname: str, stop_evt: threading.Event):
        """
        Attempt an outbound TCP connection to (ip, port).  On success the
        resulting DeviceConnection is started and registers itself via the
        normal _drain_frames / _register path when the first status frame
        arrives from the device.

        Retries with exponential backoff up to 30 s between attempts.
        Stops when stop_evt is set (mDNS removed) or device becomes connected
        (inbound connection raced ahead of outbound).
        """
        backoff = 1.0
        while not stop_evt.is_set() and self._running:
            if self.is_connected(address):
                log.debug(f"Outbound addr={address}: already connected, stopping")
                break
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((ip, port))
                sock.settimeout(None)
                addr_str = f"{ip}:{port}"
                log.info(f"Outbound TCP connected to {addr_str} "
                         f"(expected bSafe addr={address})")
                conn = DeviceConnection(sock, addr_str, self)
                conn.start()
                db.upsert_device_wifi_state(address, hostname=hostname,
                                            ip_addr=ip)
                # Clean up stop event; the DeviceConnection takes it from here
                with self._lock:
                    self._outbound_stops.pop(address, None)
                return   # success — exit retry loop
            except Exception as e:
                log.warning(f"Outbound {ip}:{port} (addr={address}) failed: "
                            f"{e} — retry in {backoff:.0f}s")
                if stop_evt.wait(backoff):
                    break   # stop event fired during sleep
                backoff = min(backoff * 2, 30.0)

    # -------------------------------------------------------------------------
    # Internal callbacks from DeviceConnection
    # -------------------------------------------------------------------------
    def _register(self, address: int, conn: DeviceConnection):
        with self._lock:
            old = self._conns.get(address)
            if old and old is not conn:
                old.close()
            self._conns[address] = conn
        log.info(f"Registered WiFi device {address}")
        # Safety net: if the on-connect WIFI_IDENTITY frame was missed for any
        # reason, request it again after a short delay.  _do_sync also does this
        # but only if MAC is absent from db; this handles the in-memory case.
        def _delayed_identity_request():
            time.sleep(1.0)
            with self._lock:
                conn = self._conns.get(address)
            if conn and not conn._device_addr_mac:
                log.info(f"[{address}] Delayed identity request (MAC not seen yet)")
                conn.request_identity()
        threading.Thread(target=_delayed_identity_request, daemon=True,
                         name=f"id-req-{address}").start()

    def _on_disconnect(self, conn: DeviceConnection):
        addr = conn._device_addr
        if addr is not None:
            with self._lock:
                if self._conns.get(addr) is conn:
                    del self._conns[addr]
            db.upsert_device_wifi_state(addr, transport="wifi",
                                        last_wifi_seen=time.time())
            log.info(f"WiFi device {addr} disconnected")

    def _on_net_report(self, address: int, idx: int, ssid: str):
        """Accumulate network reports from device."""
        import json
        if address not in self._net_reports:
            self._net_reports[address] = []
        if ssid and ssid not in self._net_reports[address]:
            self._net_reports[address].append(ssid)
        # Persist to db
        db.upsert_device_wifi_state(
            address,
            known_networks=json.dumps(self._net_reports[address]))
        log.debug(f"Net report [{address}] idx={idx} ssid='{ssid}'")

    def _on_cfg_ack(self, address: int, idx: int, result: int):
        if result == 0:
            log.info(f"CFG push [{address}] idx={idx} accepted")
        else:
            log.warning(f"CFG push [{address}] idx={idx} rejected (result={result})")
