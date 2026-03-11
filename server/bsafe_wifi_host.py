"""
bsafe_wifi_host.py — TCP server for WiFi-connected bSafe devices.

Frame format over TCP:
  [length : 2 bytes LE] [frame_type : 1 byte] [address : 1 byte] [payload : N bytes]

Connection lifecycle:
  1. Device connects — added to _pending keyed by "pending:ip:port"
  2. First WIFI_IDENTITY (0x04) arrives → MAC extracted →
       _promote(conn, mac) moves it from _pending → _conns[mac]
       Any prior stale connection for the same MAC is closed.
  3. engine.on_wifi_identity(can_address, mac) called → state promoted in engine
  4. Network sync runs in a real OS thread
  5. If WIFI_IDENTITY never arrives within IDENTITY_TIMEOUT_S → connection dropped

_conns is keyed by MAC string.  Two devices with Address=0 in their NVS
will each get their own slot because they have distinct MACs.

engine._send_cmd routes to WiFi via wifi_host.send_cmd(mac=...).
The engine looks up WiFi states by the int address byte from OP frames
(_state_for_address_locked), so the int CAN address byte is still important
for matching OP/TELEM/IR frames to the right DeviceState — it just isn't
the connection key any more.
"""

import struct
import time
import socket
import threading
import logging
import json
from typing import Optional, Callable, Dict

import db
from bsafe_frames import (
    OperationalStatus, TelemetryStatus, IrStatus, IdentityStatus,
    ChrgStat, Mode,
    FRAME_TYPE_OP, FRAME_TYPE_TELEM, FRAME_TYPE_IR, FRAME_TYPE_IDENTITY,
)

log = logging.getLogger("wifi_host")

FRAME_WIFI_IDENTITY   = 0x04
FRAME_WIFI_CFG_PUSH   = 0x10
FRAME_WIFI_CFG_ACK    = 0x11
FRAME_WIFI_NET_REPORT = 0x12
FRAME_WIFI_NET_REQ    = 0x13
FRAME_CMD             = 0x80

HDR_FMT   = "<HBB"
HDR_SIZE  = struct.calcsize(HDR_FMT)   # 4 bytes
MAX_FRAME = 512

IDENTITY_TIMEOUT_S = 6.0   # drop connection if no WIFI_IDENTITY within this time


# ---------------------------------------------------------------------------
# Frame builders
# ---------------------------------------------------------------------------
def _build_frame(frame_type: int, address: int, payload: bytes) -> bytes:
    return struct.pack(HDR_FMT, len(payload), frame_type, address) + payload


def _build_cmd_frame(address: int, mode: int, dsc_duty_pct: int = 30,
                     dsc_pulse_ms: int = 5000, settle_minutes: int = 0,
                     request_flag: bool = False, req_frame_type: int = 0) -> bytes:
    flags = (req_frame_type & 0x07) | (0x08 if request_flag else 0x00)
    payload = struct.pack(">BBHIB", mode & 0xFF, dsc_duty_pct & 0xFF,
                          dsc_pulse_ms & 0xFFFF, settle_minutes & 0xFFFFFFFF, flags)
    return _build_frame(FRAME_CMD, address, payload)


def _build_net_req(address: int) -> bytes:
    return _build_frame(FRAME_WIFI_NET_REQ, address, b"")


def _build_cfg_push(address: int, idx: int, ssid: str, password: str) -> bytes:
    ssid_b = ssid.encode()[:32].ljust(33, b"\x00")
    pass_b = password.encode()[:64].ljust(65, b"\x00")
    return _build_frame(FRAME_WIFI_CFG_PUSH, address,
                        struct.pack("B", idx & 0xFF) + ssid_b + pass_b)


# ---------------------------------------------------------------------------
# Per-device TCP connection
# ---------------------------------------------------------------------------
class DeviceConnection:
    """
    One TCP connection from one bSafe device.

    _can_address  — the address byte from frames (used for framing only;
                    engine looks up states by this int)
    _mac          — WiFi STA MAC string, set after WIFI_IDENTITY arrives;
                    used as the connection key in BSafeWiFiHost._conns
    """

    def __init__(self, sock: socket.socket, addr_str: str,
                 wifi_host: "BSafeWiFiHost"):
        self._sock         = sock
        self._addr_str     = addr_str      # "ip:port"
        self._host         = wifi_host
        self._can_address  = 0             # int address byte from frames
        self._mac: str     = ""            # set on WIFI_IDENTITY
        self._lock         = threading.Lock()
        self._running      = False
        self._buf          = b""
        self._log          = logging.getLogger(f"wifi_conn[{addr_str}]")
        self._sync_done    = False

    @property
    def provisional_key(self) -> str:
        return f"pending:{self._addr_str}"

    def start(self):
        self._running = True
        self._host._add_pending(self)
        # Watchdog: close if identity never arrives
        self._host._make_thread(target=self._identity_watchdog, daemon=True,
                                name=f"wifi-wdog-{self._addr_str}").start()
        try:
            import eventlet.patcher
            RealThread = eventlet.patcher.original("threading").Thread
        except Exception:
            RealThread = threading.Thread
        RealThread(target=self._rx_loop, daemon=True,
                   name=f"wifi-rx-{self._addr_str}").start()

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
        self.send_raw(_build_cmd_frame(self._can_address, mode, **kwargs))

    def request_identity(self):
        self.send_cmd(mode=4, request_flag=True,
                      req_frame_type=FRAME_TYPE_IDENTITY)

    def request_network_list(self):
        self.send_raw(_build_net_req(self._can_address))

    def push_network(self, idx: int, ssid: str, password: str):
        self.send_raw(_build_cfg_push(self._can_address, idx, ssid, password))

    # ------------------------------------------------------------------
    # Identity watchdog
    # ------------------------------------------------------------------
    def _identity_watchdog(self):
        time.sleep(IDENTITY_TIMEOUT_S)
        if self._running and not self._mac:
            self._log.warning(
                f"No WIFI_IDENTITY in {IDENTITY_TIMEOUT_S}s — requesting and waiting")
            # One retry before giving up
            self.request_identity()
            time.sleep(3.0)
            if self._running and not self._mac:
                self._log.warning("Still no identity — dropping connection")
                self.close()

    # ------------------------------------------------------------------
    # RX loop (real OS thread)
    # ------------------------------------------------------------------
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
                break
            payload   = self._buf[HDR_SIZE:total]
            self._buf = self._buf[total:]

            # Track the CAN address byte (first non-zero wins)
            if not self._can_address and address:
                self._can_address = address

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
                self._log.info(
                    f"WIFI_IDENTITY: addr={address} len={len(payload)} "
                    f"raw={payload.hex()}")

                if len(payload) < 14:
                    self._log.warning(f"WIFI_IDENTITY too short ({len(payload)}B)")
                    return

                frame = IdentityStatus.parse(address, payload[:8])
                mac_str = ":".join(f"{b:02X}" for b in payload[8:14])
                self._log.info(f"WiFi MAC: {mac_str} (can_addr={address})")

                self._can_address = address
                self._mac = mac_str

                # Promote: move from _pending → _conns[mac]
                host._promote(self, mac_str)

                # Fire callbacks — engine.on_wifi_identity promotes its state
                if host._on_identity:
                    host._on_identity(address, frame.schema_version,
                                      frame.hw_ver_hash)
                if host._on_wifi_identity:
                    host._on_wifi_identity(address, mac_str)

                # Update DB
                db.upsert_device_wifi_state_by_mac(
                    mac_str,
                    frame_address=address,
                    transport="wifi",
                    ip_addr=self._addr_str.split(":")[0],
                    last_wifi_seen=time.time())

                # Start network sync
                if not self._sync_done:
                    host._make_thread(target=self._do_sync, daemon=True,
                                      name=f"wifi-sync-{mac_str}").start()

            elif ftype == FRAME_WIFI_NET_REPORT:
                if len(payload) >= 34:
                    idx  = payload[0]
                    ssid = payload[1:34].rstrip(b"\x00").decode(errors="replace")
                    host._on_net_report(self._mac or str(address), idx, ssid)

            elif ftype == FRAME_WIFI_CFG_ACK:
                if len(payload) >= 2:
                    idx, result = payload[0], payload[1]
                    self._log.info(f"CFG ACK idx={idx} result={result}")
                    host._on_cfg_ack(self._mac or str(address), idx, result)

            else:
                self._log.debug(f"Unknown frame 0x{ftype:02X}")

        except Exception as e:
            self._log.warning(f"Dispatch error 0x{ftype:02X}: {e}", exc_info=True)

    # ------------------------------------------------------------------
    # Network sync (real OS thread)
    # ------------------------------------------------------------------
    def _do_sync(self):
        mac = self._mac
        if not mac:
            return

        self._log.info(f"Network sync for {mac}")
        db.upsert_device_wifi_state_by_mac(mac, sync_status="syncing")

        self.request_network_list()
        time.sleep(2.0)

        state     = db.get_device_wifi_state_by_mac(mac)
        known_raw = (state or {}).get("known_networks") or "[]"
        try:
            known_ssids = set(json.loads(known_raw))
        except Exception:
            known_ssids = set()

        master  = db.get_wifi_networks()
        to_push = [n for n in master if n["ssid"] not in known_ssids]

        if not to_push:
            self._log.info(f"[{mac}] Already has all networks — synced")
            db.upsert_device_wifi_state_by_mac(mac, sync_status="synced")
            return

        self._log.info(f"[{mac}] Pushing {len(to_push)} network(s)")
        for idx, net in enumerate(to_push):
            self.push_network(idx, net["ssid"], net["password"])
            time.sleep(0.3)

        db.upsert_device_wifi_state_by_mac(mac, sync_status="synced")
        self._sync_done = True


# ---------------------------------------------------------------------------
# BSafeWiFiHost
# ---------------------------------------------------------------------------
class BSafeWiFiHost:
    """
    TCP server for WiFi-connected bSafe devices.
    _conns  : Dict[str, DeviceConnection]  — keyed by MAC (after promotion)
    _pending: Dict[str, DeviceConnection]  — keyed by "pending:ip:port" (pre-identity)
    """

    def __init__(self, bind_ip: str = "0.0.0.0", port: int = 7000):
        self._bind_ip  = bind_ip
        self._port     = port
        self._running  = False
        self._server   = None

        self._conns: Dict[str, DeviceConnection]   = {}   # mac → conn
        self._pending: Dict[str, DeviceConnection] = {}   # provisional key → conn
        self._lock     = threading.Lock()

        self._on_any_status:    Optional[Callable] = None
        self._on_telem:         Optional[Callable] = None
        self._on_ir:            Optional[Callable] = None
        self._on_identity:      Optional[Callable] = None
        self._on_wifi_identity: Optional[Callable] = None

        self._net_reports: Dict[str, list] = {}   # mac → [ssid, ...]

        self._outbound_stops: Dict[str, threading.Event] = {}

    # -------------------------------------------------------------------------
    # Real-thread helper
    # -------------------------------------------------------------------------
    def _make_thread(self, **kwargs) -> threading.Thread:
        try:
            import eventlet.patcher
            RealThread = eventlet.patcher.original("threading").Thread
        except Exception:
            RealThread = threading.Thread
        return RealThread(**kwargs)

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def on_any_status(self, cb):  self._on_any_status    = cb
    def on_telemetry(self, cb):   self._on_telem         = cb
    def on_ir(self, cb):          self._on_ir            = cb
    def on_identity(self, cb):    self._on_identity      = cb
    def on_wifi_identity(self, cb): self._on_wifi_identity = cb

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def start(self):
        self._running = True
        self._make_thread(target=self._accept_loop, daemon=True,
                          name="wifi-accept").start()
        log.info(f"WiFi TCP server on {self._bind_ip}:{self._port}")

    def stop(self):
        self._running = False
        if self._server:
            try: self._server.close()
            except Exception: pass
        with self._lock:
            for c in list(self._conns.values()) + list(self._pending.values()):
                c.close()
            self._conns.clear()
            self._pending.clear()

    def rebind(self, new_ip: str):
        log.info(f"WiFi host rebinding to {new_ip}:{self._port}")
        if self._server:
            try: self._server.close()
            except Exception: pass
        self._bind_ip = new_ip
        self._make_thread(target=self._accept_loop, daemon=True,
                          name="wifi-accept-rebind").start()

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
                    DeviceConnection(sock, addr_str, self).start()
                except socket.timeout:
                    continue
                except OSError:
                    break
        except Exception as e:
            log.error(f"Accept loop error: {e}", exc_info=True)

    # -------------------------------------------------------------------------
    # CMD routing — keyed by MAC string
    # -------------------------------------------------------------------------
    def send_cmd(self, mac: str, mode, dsc_duty_pct: int = 30,
                 dsc_pulse_ms: int = 5000, settle_minutes: int = 0,
                 request_flag: bool = False, req_frame_type: int = 0,
                 precharge_thresh: int = 0):
        with self._lock:
            conn = self._conns.get(mac)
        if conn:
            conn.send_cmd(int(mode), dsc_duty_pct=dsc_duty_pct,
                          dsc_pulse_ms=dsc_pulse_ms, settle_minutes=settle_minutes,
                          request_flag=request_flag, req_frame_type=req_frame_type)
        else:
            log.debug(f"send_cmd: no WiFi connection for MAC {mac}")

    def connected_macs(self) -> list:
        with self._lock:
            return list(self._conns.keys())

    # Legacy alias — some call sites use connected_addresses()
    def connected_addresses(self) -> list:
        return self.connected_macs()

    def is_connected(self, mac: str) -> bool:
        with self._lock:
            return mac in self._conns

    def push_networks_to_device(self, mac: str):
        with self._lock:
            conn = self._conns.get(mac)
        if conn:
            self._make_thread(target=conn._do_sync, daemon=True,
                              name=f"wifi-sync-push-{mac}").start()

    # -------------------------------------------------------------------------
    # mDNS outbound
    # -------------------------------------------------------------------------
    def notify_discovered(self, mac: str, ip: str, port: int, hostname: str):
        key = mac if mac else f"{ip}:{port}"
        if mac and self.is_connected(mac):
            log.info(f"mDNS {mac} already connected; updating hostname")
            db.upsert_device_wifi_state_by_mac(mac, hostname=hostname)
            return
        with self._lock:
            old = self._outbound_stops.get(key)
            if old: old.set()
            stop_evt = threading.Event()
            self._outbound_stops[key] = stop_evt
        self._make_thread(target=self._outbound_retry,
                          args=(key, ip, port, hostname, stop_evt),
                          daemon=True, name=f"wifi-out-{key}").start()
        log.info(f"mDNS: outbound to {hostname} @ {ip}:{port}")

    def notify_removed(self, mac: str):
        with self._lock:
            stop = self._outbound_stops.pop(mac, None)
        if stop:
            stop.set()
            log.info(f"mDNS: {mac} removed")

    def _outbound_retry(self, key, ip, port, hostname, stop_evt):
        backoff = 1.0
        while not stop_evt.is_set() and self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((ip, port))
                sock.settimeout(None)
                log.info(f"Outbound TCP connected to {ip}:{port}")
                DeviceConnection(sock, f"{ip}:{port}", self).start()
                with self._lock:
                    self._outbound_stops.pop(key, None)
                return
            except Exception as e:
                log.warning(f"Outbound {ip}:{port} failed: {e} — retry in {backoff:.0f}s")
                if stop_evt.wait(backoff): break
                backoff = min(backoff * 2, 30.0)

    # -------------------------------------------------------------------------
    # Internal connection management
    # -------------------------------------------------------------------------
    def _add_pending(self, conn: DeviceConnection):
        with self._lock:
            self._pending[conn.provisional_key] = conn
        log.debug(f"Pending connection: {conn.provisional_key}")

    def _promote(self, conn: DeviceConnection, mac: str):
        """
        Move conn from _pending → _conns[mac].
        Close any existing stale connection for this MAC.
        """
        with self._lock:
            self._pending.pop(conn.provisional_key, None)
            old = self._conns.get(mac)
            if old and old is not conn:
                log.info(f"Closing stale connection for {mac} "
                         f"(prev={old._addr_str}, new={conn._addr_str})")
                old.close()
            self._conns[mac] = conn
        log.info(f"Promoted {conn._addr_str} → MAC {mac}")

    def _on_disconnect(self, conn: DeviceConnection):
        mac = conn._mac
        with self._lock:
            self._pending.pop(conn.provisional_key, None)
            if mac and self._conns.get(mac) is conn:
                del self._conns[mac]
        if mac:
            db.upsert_device_wifi_state_by_mac(mac, last_wifi_seen=time.time())
            log.info(f"WiFi device {mac} disconnected")
        else:
            log.info(f"Pending conn {conn._addr_str} disconnected (no MAC)")

    def _on_net_report(self, mac: str, idx: int, ssid: str):
        if mac not in self._net_reports:
            self._net_reports[mac] = []
        if ssid and ssid not in self._net_reports[mac]:
            self._net_reports[mac].append(ssid)
        try:
            db.upsert_device_wifi_state_by_mac(
                mac, known_networks=json.dumps(self._net_reports[mac]))
        except Exception:
            pass
        log.debug(f"Net report [{mac}] idx={idx} ssid='{ssid}'")

    def _on_cfg_ack(self, mac: str, idx: int, result: int):
        if result == 0:
            log.info(f"CFG push [{mac}] idx={idx} accepted")
        else:
            log.warning(f"CFG push [{mac}] idx={idx} rejected (result={result})")
