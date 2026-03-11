"""
network_manager.py — Pi-side network state machine.

Two modes:
  CARRIER — connected to external WiFi as a client (DHCP IP)
  SOFTAP  — Pi runs its own access point via hostapd + dnsmasq

Startup sequence:
  1. force_softap flag set (CLI --softap or db) → SOFTAP immediately
  2. Otherwise attempt carrier (timeout 15s)
  3. Carrier fail → SOFTAP fallback
  4. Background monitor checks health every 30s, retriggers if carrier drops

Public API:
  start()            — begin state machine (non-blocking, starts thread)
  stop()             — tear down AP, stop thread
  force_softap()     — switch to AP, latch until released
  release_softap()   — clear latch, attempt carrier
  retry_carrier()    — try carrier without touching latch
  status()           — current state dict for UI
  current_ip()       — IP on wlan0 right now
"""

import os
import sys
import time
import threading
import subprocess
import logging
from enum import Enum
from typing import Optional

import db

log = logging.getLogger("netmgr")

# Network management syscalls only work on Linux (Raspberry Pi target).
# On Windows/macOS the TCP WiFi server still runs; the state machine
# enters passive mode and skips all ip/hostapd/nmcli/dhclient calls.
IS_LINUX = sys.platform.startswith("linux")

WLAN_IFACE       = os.environ.get("BSAFE_WLAN",  "wlan0")
AP_IP            = os.environ.get("BSAFE_AP_IP", "192.168.4.1")
DHCP_RANGE_START = "192.168.4.10"
DHCP_RANGE_END   = "192.168.4.50"
HOSTAPD_CONF     = "/tmp/bsafe_hostapd.conf"
DNSMASQ_CONF     = "/tmp/bsafe_dnsmasq.conf"
CARRIER_TIMEOUT  = 15   # seconds to wait for DHCP


class NetMode(str, Enum):
    UNKNOWN = "unknown"
    CARRIER = "carrier"
    SOFTAP  = "softap"
    OFFLINE = "offline"


class NetworkManager:

    def __init__(self, force_softap: bool = False, socketio=None):
        self._lock           = threading.Lock()
        self._mode           = NetMode.UNKNOWN
        self._ip: Optional[str] = None
        self._ssid: Optional[str] = None
        self._force          = force_softap
        self._socketio       = socketio
        self._hostapd        = None
        self._dnsmasq        = None
        self._running        = False
        self._trigger        = threading.Event()

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def start(self):
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True, name="netmgr")
        t.start()
        log.info(f"NetworkManager started (force={self._force})")

    def stop(self):
        self._running = False
        self._trigger.set()
        self._teardown_ap()

    # -------------------------------------------------------------------------
    # Public controls
    # -------------------------------------------------------------------------
    def force_softap(self):
        with self._lock:
            self._force = True
        db.save_wifi_config(force_softap=1)
        log.info("SoftAP forced")
        self._trigger.set()

    def release_softap(self):
        with self._lock:
            self._force = False
        db.save_wifi_config(force_softap=0)
        log.info("SoftAP released, retrying carrier")
        self._trigger.set()

    def retry_carrier(self):
        log.info("Carrier retry requested")
        self._trigger.set()

    def current_ip(self) -> Optional[str]:
        with self._lock:
            return self._ip

    def current_mode(self) -> NetMode:
        with self._lock:
            return self._mode

    def status(self) -> dict:
        with self._lock:
            cfg = db.get_wifi_config()
            return {
                "mode":         self._mode.value,
                "ip":           self._ip,
                "ssid":         self._ssid,
                "force_softap": self._force,
                "wlan_iface":   WLAN_IFACE,
                "ap_ip":        AP_IP,
                "ap_ssid":      cfg.get("ap_ssid", "bsafe-net"),
                "carrier_ssid": cfg.get("carrier_ssid"),
                "tcp_port":     cfg.get("tcp_port", 7000),
                "ap_clients":   self._ap_clients(),
            }

    # -------------------------------------------------------------------------
    # Background loop
    # -------------------------------------------------------------------------
    def _loop(self):
        self._transition()
        while self._running:
            fired = self._trigger.wait(timeout=30.0)
            if fired:
                self._trigger.clear()
            if not self._running:
                break
            self._health_check()

    def _health_check(self):
        with self._lock:
            mode   = self._mode
            forced = self._force
        if mode == NetMode.CARRIER:
            if not self._wlan_ip():
                log.warning("Carrier lost — falling back to softAP")
                self._transition()
        elif mode == NetMode.SOFTAP and not forced:
            # Periodic carrier retry while in unforced softAP
            self._transition()

    def _transition(self):
        with self._lock:
            forced = self._force
        if forced:
            self._start_ap()
        else:
            if not self._try_carrier():
                log.info("Carrier unavailable — starting softAP")
                self._start_ap()

    # -------------------------------------------------------------------------
    # Carrier
    # -------------------------------------------------------------------------
    def _try_carrier(self) -> bool:
        if not IS_LINUX:
            log.debug("Non-Linux platform — carrier management skipped")
            return False
        cfg  = db.get_wifi_config()
        ssid = cfg.get("carrier_ssid", "")
        pw   = cfg.get("carrier_pass", "")
        if not ssid:
            log.debug("No carrier SSID configured")
            return False

        log.info(f"Connecting to carrier '{ssid}'")
        self._teardown_ap()

        connected = False
        if self._has_cmd("nmcli"):
            r = subprocess.run(
                ["nmcli", "device", "wifi", "connect", ssid,
                 "password", pw, "ifname", WLAN_IFACE],
                capture_output=True, timeout=20)
            connected = r.returncode == 0
        else:
            # Minimal wpa_supplicant fallback
            conf = (f'network={{\n  ssid="{ssid}"\n  psk="{pw}"\n}}\n')
            with open("/tmp/bsafe_wpa.conf", "w") as f:
                f.write(conf)
            subprocess.run(["wpa_supplicant", "-B", "-i", WLAN_IFACE,
                            "-c", "/tmp/bsafe_wpa.conf"],
                           capture_output=True, timeout=10)
            subprocess.run(["dhclient", WLAN_IFACE],
                           capture_output=True, timeout=15)
            connected = True   # optimistic — confirmed below by IP check

        if not connected:
            return False

        # Wait for DHCP
        ip = None
        deadline = time.time() + CARRIER_TIMEOUT
        while time.time() < deadline:
            ip = self._wlan_ip()
            if ip:
                break
            time.sleep(1.0)

        if ip:
            with self._lock:
                self._mode = NetMode.CARRIER
                self._ip   = ip
                self._ssid = ssid
            log.info(f"Carrier up: {ssid} @ {ip}")
            self._emit("net_mode", {"mode": "carrier", "ip": ip, "ssid": ssid})
            return True

        log.warning(f"No IP from carrier '{ssid}' after {CARRIER_TIMEOUT}s")
        return False

    # -------------------------------------------------------------------------
    # SoftAP
    # -------------------------------------------------------------------------
    def _start_ap(self):
        if not IS_LINUX:
            log.debug("Non-Linux platform — softAP management skipped")
            return
        self._teardown_ap()
        cfg     = db.get_wifi_config()
        ap_ssid = cfg.get("ap_ssid", "bsafe-net")
        ap_pass = cfg.get("ap_pass", "bsafe1234")
        channel = int(cfg.get("ap_channel", 6))

        log.info(f"Starting softAP '{ap_ssid}' ch{channel} on {WLAN_IFACE}")

        try:
            # Static IP on wlan0
            subprocess.run(["ip", "addr", "flush", "dev", WLAN_IFACE], capture_output=True)
            subprocess.run(["ip", "addr", "add", f"{AP_IP}/24", "dev", WLAN_IFACE], capture_output=True)
            subprocess.run(["ip", "link", "set", WLAN_IFACE, "up"], capture_output=True)

            # hostapd
            with open(HOSTAPD_CONF, "w") as f:
                f.write(
                    f"interface={WLAN_IFACE}\n"
                    f"driver=nl80211\n"
                    f"ssid={ap_ssid}\n"
                    f"hw_mode=g\n"
                    f"channel={channel}\n"
                    f"wmm_enabled=0\n"
                    f"auth_algs=1\n"
                    f"wpa=2\n"
                    f"wpa_passphrase={ap_pass}\n"
                    f"wpa_key_mgmt=WPA-PSK\n"
                    f"rsn_pairwise=CCMP\n"
                )
            self._hostapd = subprocess.Popen(
                ["hostapd", HOSTAPD_CONF],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.2)

            # dnsmasq
            with open(DNSMASQ_CONF, "w") as f:
                f.write(
                    f"interface={WLAN_IFACE}\n"
                    f"dhcp-range={DHCP_RANGE_START},{DHCP_RANGE_END},12h\n"
                    f"dhcp-option=3,{AP_IP}\n"
                    f"dhcp-option=6,{AP_IP}\n"
                    f"no-resolv\n"
                )
            self._dnsmasq = subprocess.Popen(
                ["dnsmasq", "-C", DNSMASQ_CONF, "--no-daemon"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            with self._lock:
                self._mode = NetMode.SOFTAP
                self._ip   = AP_IP
                self._ssid = ap_ssid

            log.info(f"SoftAP up: ssid={ap_ssid} ip={AP_IP}")
            self._emit("net_mode", {"mode": "softap", "ip": AP_IP, "ssid": ap_ssid})

            # Ensure AP network is in master list so devices always get it
            db.upsert_wifi_network(ap_ssid, ap_pass, priority=100, is_ap=True)

        except Exception as e:
            log.error(f"SoftAP start failed: {e}", exc_info=True)
            with self._lock:
                self._mode = NetMode.OFFLINE
                self._ip   = None

    def _teardown_ap(self):
        for proc_attr in ("_hostapd", "_dnsmasq"):
            proc = getattr(self, proc_attr)
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                setattr(self, proc_attr, None)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _wlan_ip(self) -> Optional[str]:
        if not IS_LINUX:
            return None
        try:
            r = subprocess.run(
                ["ip", "-4", "addr", "show", WLAN_IFACE],
                capture_output=True, text=True, timeout=5)
            for line in r.stdout.splitlines():
                ln = line.strip()
                if ln.startswith("inet "):
                    return ln.split()[1].split("/")[0]
        except Exception:
            pass
        return None

    def _has_cmd(self, cmd: str) -> bool:
        import shutil
        return shutil.which(cmd) is not None

    def _ap_clients(self) -> list:
        """Read dnsmasq lease file for connected AP clients."""
        if self._mode != NetMode.SOFTAP:
            return []
        clients = []
        for path in ("/var/lib/misc/dnsmasq.leases", "/tmp/dnsmasq.leases"):
            try:
                with open(path) as f:
                    for line in f:
                        p = line.strip().split()
                        if len(p) >= 4:
                            clients.append({
                                "mac":      p[1],
                                "ip":       p[2],
                                "hostname": p[3] if p[3] != "*" else "",
                            })
                break
            except FileNotFoundError:
                continue
            except Exception as e:
                log.debug(f"Lease file: {e}")
        return clients

    def _emit(self, event: str, data: dict):
        if self._socketio:
            try:
                self._socketio.emit(event, data)
            except Exception as e:
                log.debug(f"emit error: {e}")
