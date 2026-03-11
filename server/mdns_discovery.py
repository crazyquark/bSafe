"""
mdns_discovery.py — mDNS browser for _bsafe._tcp.local. services.

Browses the local network for ESP32-C3 bSafe devices that advertise themselves
via mDNS.  Each device advertises:
  service type : _bsafe._tcp.local.
  hostname     : bsafeXXXXX.local  (XXXXX = MAC-derived suffix)
  port         : 7000  (or as advertised)
  TXT records  : addr=N  (bSafe bus address 0-63, set in NVS)

On service added/updated:
  - Parse addr TXT record
  - Persist hostname + IP to wifi_devices table
  - Call wifi_host.notify_discovered(address, ip, port, hostname)

On service removed:
  - Remove from cache
  - Call wifi_host.notify_removed(address) to cancel any outbound retry

All zeroconf imports are guarded; if the package is absent the browser
simply does not start, so the server remains functional on dev machines.
"""

import socket
import time
import threading
import logging
from typing import Optional, Dict, TYPE_CHECKING

import db

if TYPE_CHECKING:
    from bsafe_wifi_host import BSafeWiFiHost

log = logging.getLogger("mdns")

# ---------------------------------------------------------------------------
# Optional zeroconf import
# ---------------------------------------------------------------------------
_ZEROCONF_AVAILABLE = False
try:
    from zeroconf import Zeroconf, ServiceBrowser, ServiceInfo  # noqa: F401
    _ZEROCONF_AVAILABLE = True
except ImportError:
    pass

SERVICE_TYPE = "_bsafe._tcp.local."


# ---------------------------------------------------------------------------
# ServiceBrowser listener
# ---------------------------------------------------------------------------
class _Listener:
    """Thin adapter between zeroconf's ServiceBrowser and MdnsBrowser."""

    def __init__(self, browser: "MdnsBrowser"):
        self._b = browser

    # zeroconf calls these on its own internal thread — offload to avoid
    # blocking the browser's event loop during the info-request round trip.
    def add_service(self, zc, type_, name):
        threading.Thread(target=self._b._handle_added, args=(zc, type_, name),
                         daemon=True, name="mdns-add").start()

    def update_service(self, zc, type_, name):
        threading.Thread(target=self._b._handle_added, args=(zc, type_, name),
                         daemon=True, name="mdns-upd").start()

    def remove_service(self, zc, type_, name):      # noqa: ARG002
        self._b._handle_removed(name)


# ---------------------------------------------------------------------------
# MdnsBrowser
# ---------------------------------------------------------------------------
class MdnsBrowser:
    """
    Browses _bsafe._tcp.local., maintains a cache of visible services, and
    notifies BSafeWiFiHost when devices appear or disappear.
    """

    def __init__(self):
        self._zc = None
        self._browser = None
        self._wifi_host: Optional["BSafeWiFiHost"] = None
        self._lock = threading.Lock()
        # address (int) → {address, hostname, ip, port, name, last_seen}
        self._cache: Dict[int, dict] = {}

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def start(self, wifi_host: "BSafeWiFiHost") -> bool:
        """
        Start browsing.  Returns True on success, False if zeroconf unavailable.
        Safe to call on non-Pi hardware — will just return False.
        """
        if not _ZEROCONF_AVAILABLE:
            log.warning("python-zeroconf not installed — mDNS browser disabled")
            return False

        self._wifi_host = wifi_host
        self._zc = Zeroconf()
        self._browser = ServiceBrowser(self._zc, SERVICE_TYPE, _Listener(self))
        log.info("mDNS browser started for %s", SERVICE_TYPE)
        return True

    def stop(self):
        if self._zc:
            try:
                self._zc.close()
            except Exception:
                pass
            self._zc = None
        self._browser = None

    # -------------------------------------------------------------------------
    # Public read-only cache
    # -------------------------------------------------------------------------
    def get_cache(self) -> list:
        """Return a snapshot list of currently visible services."""
        with self._lock:
            return list(self._cache.values())

    # -------------------------------------------------------------------------
    # Internal handlers (run on daemon threads)
    # -------------------------------------------------------------------------
    def _handle_added(self, zc, type_: str, name: str):
        try:
            info = zc.get_service_info(type_, name, timeout=3000)
            if info is None:
                log.warning("mDNS: could not resolve info for '%s'", name)
                return

            # --- IP address ---
            ip = self._parse_ip(info)
            if not ip:
                log.warning("mDNS: no IP address for '%s'", name)
                return

            port = info.port or 7000

            # --- Hostname (strip .local. suffix) ---
            hostname = info.server or ""
            for suffix in (".local.", ".local"):
                if hostname.endswith(suffix):
                    hostname = hostname[:-len(suffix)]
                    break
            if not hostname:
                hostname = name.split(".")[0]

            # --- TXT record: addr=N ---
            props = self._parse_txt(info)
            if "addr" not in props:
                log.warning("mDNS: '%s' missing required 'addr' TXT record", name)
                return
            try:
                address = int(props["addr"])
            except ValueError:
                log.warning("mDNS: '%s' has non-integer addr='%s'",
                            name, props["addr"])
                return
            if not (0 <= address <= 63):
                log.warning("mDNS: '%s' addr=%d out of valid range 0-63",
                            name, address)
                return

            log.info("mDNS discovered: addr=%d host=%s ip=%s port=%d",
                     address, hostname, ip, port)

            entry = {
                "address":   address,
                "hostname":  hostname,
                "ip":        ip,
                "port":      port,
                "name":      name,
                "last_seen": time.time(),
            }
            with self._lock:
                self._cache[address] = entry

            # Persist to DB
            db.upsert_device_wifi_state(
                address,
                hostname=hostname,
                ip_addr=ip,
                discovered_at=time.time(),
            )
            db.log_event("mdns_discovered", address=address,
                         data={"hostname": hostname, "ip": ip, "port": port})

            # Notify WiFi host → triggers outbound connect if not yet connected
            if self._wifi_host:
                self._wifi_host.notify_discovered(address, ip, port, hostname)

        except Exception:
            log.exception("mDNS: error processing service '%s'", name)

    def _handle_removed(self, name: str):
        address = None
        with self._lock:
            for addr, entry in list(self._cache.items()):
                if entry["name"] == name:
                    address = addr
                    del self._cache[addr]
                    break

        if address is not None:
            log.info("mDNS removed: addr=%d (%s)", address, name)
            db.log_event("mdns_removed", address=address, data={"name": name})
            if self._wifi_host:
                self._wifi_host.notify_removed(address)
        else:
            log.debug("mDNS removed unknown service: %s", name)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def _parse_ip(info) -> Optional[str]:
        try:
            addrs = info.parsed_addresses()
            if addrs:
                return addrs[0]
        except Exception:
            pass
        # Fallback: raw bytes
        if info.addresses:
            try:
                return socket.inet_ntoa(info.addresses[0])
            except Exception:
                pass
        return None

    @staticmethod
    def _parse_txt(info) -> dict:
        props = {}
        for k, v in (info.properties or {}).items():
            ks = k.decode(errors="replace") if isinstance(k, bytes) else str(k)
            vs = v.decode(errors="replace") if isinstance(v, bytes) else str(v)
            props[ks] = vs
        return props


# ---------------------------------------------------------------------------
# Module-level singleton + public API
# ---------------------------------------------------------------------------
_browser: Optional[MdnsBrowser] = None


def start_browser(wifi_host: "BSafeWiFiHost") -> Optional[MdnsBrowser]:
    """
    Start the global mDNS browser.  Called from network_blueprint.init_network()
    on Pi hardware only.  Returns the MdnsBrowser if started, None otherwise.
    """
    global _browser
    _browser = MdnsBrowser()
    if _browser.start(wifi_host):
        return _browser
    return None


def get_browser() -> Optional[MdnsBrowser]:
    """Return the running browser instance (or None if not started)."""
    return _browser
