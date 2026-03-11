"""
sim_blueprint.py — Mock device simulator for UI development (no CAN hardware)

Injects fake OperationalStatus frames into the engine at 2 Hz, mimicking the
behaviour of real bSafe chargers on Windows / x86 dev machines.
Only activates when no real CAN host is present.

REST API at /sim/:
  GET  /sim/api/devices            — list simulated devices
  POST /sim/api/devices            — add a device {address, vbat_v, batt_present}
  PATCH /sim/api/devices/<addr>    — update device fields
  DELETE /sim/api/devices/<addr>   — remove a device
"""

from flask import Blueprint, jsonify, request
from dataclasses import dataclass
import threading
import time
import logging

log = logging.getLogger("sim")


# Minimal stand-in for bsafe_host.OperationalStatus / ChrgStat.
# Avoids importing bsafe_host (which pulls in python-can, unavailable on Windows).
# Fields match what engine.on_status() reads.
@dataclass
class _SimStatus:
    address:             int   = 0
    vbat_v:              float = 0.0
    ibat_a:              float = 0.0
    error:               bool  = False
    full:                bool  = False
    pwr_ok:              bool  = False
    batt_present:        bool  = False
    mode_change_request: bool  = False
    chrg_stat:           int   = 0   # 0=none 1=pre 2=fast 3=done
    dsc_pct:             int   = 0

sim_bp = Blueprint("sim", __name__, url_prefix="/sim")

_engine  = None
_devices: dict = {}   # address → state dict
_lock    = threading.Lock()
_running = False


def init_sim(engine):
    """Called from app.py startup(). No-op when a real CAN host is active."""
    global _engine, _running

    if engine.can_host is not None:
        log.info("Simulator disabled — real CAN host present")
        return

    _engine  = engine
    _running = True

    # Seed mock devices here to test the UI without hardware:
    # with _lock:
    #     _devices[1] = _make_device(1, vbat_v=3.72, batt_present=True)
    #     _devices[2] = _make_device(2, vbat_v=0.0,  batt_present=False)

    t = threading.Thread(target=_sim_loop, daemon=True, name="sim")
    t.start()
    log.info("Simulator started — no mock devices (add via POST /sim/api/devices)")


def _make_device(address: int, vbat_v: float = 3.7,
                 batt_present: bool = True) -> dict:
    return {
        "address":             address,
        "vbat_v":              vbat_v,
        "ibat_a":              0.0,
        "error":               False,
        "full":                False,
        "pwr_ok":              True,
        "batt_present":        batt_present,
        "mode_change_request": False,
        "chrg_stat":           0,
        "dsc_pct":             0,
    }


def _sim_loop():
    while _running:
        with _lock:
            snapshot = list(_devices.values())
        for dev in snapshot:
            status = _SimStatus(
                address             = dev["address"],
                vbat_v              = dev["vbat_v"],
                ibat_a              = dev["ibat_a"],
                error               = dev["error"],
                full                = dev["full"],
                pwr_ok              = dev["pwr_ok"],
                batt_present        = dev["batt_present"],
                mode_change_request = dev["mode_change_request"],
                chrg_stat           = dev["chrg_stat"],
                dsc_pct             = dev["dsc_pct"],
            )
            if _engine:
                _engine.on_status(status)
        time.sleep(0.5)  # 2 Hz — matches real firmware TX rate


# ---------------------------------------------------------------------------
# REST API
# ---------------------------------------------------------------------------

@sim_bp.route("/api/devices", methods=["GET"])
def sim_list_devices():
    with _lock:
        return jsonify([
            {**d, "chrg_stat": int(d["chrg_stat"])}
            for d in _devices.values()
        ])


@sim_bp.route("/api/devices", methods=["POST"])
def sim_add_device():
    data    = request.json or {}
    addr    = int(data.get("address", 3))
    vbat    = float(data.get("vbat_v", 3.7))
    present = bool(data.get("batt_present", True))
    with _lock:
        _devices[addr] = _make_device(addr, vbat_v=vbat, batt_present=present)
    return jsonify({"ok": True, "address": addr})


@sim_bp.route("/api/devices/<int:addr>", methods=["PATCH"])
def sim_update_device(addr):
    data = request.json or {}
    with _lock:
        if addr not in _devices:
            return jsonify({"error": "not found"}), 404
        dev = _devices[addr]
        for key in ("vbat_v", "ibat_a", "error", "full", "pwr_ok",
                    "batt_present", "mode_change_request", "dsc_pct"):
            if key in data:
                dev[key] = data[key]
        if "chrg_stat" in data:
            dev["chrg_stat"] = int(data["chrg_stat"]) & 0x03
    return jsonify({"ok": True})


@sim_bp.route("/api/devices/<int:addr>", methods=["DELETE"])
def sim_remove_device(addr):
    with _lock:
        _devices.pop(addr, None)
    return jsonify({"ok": True})
