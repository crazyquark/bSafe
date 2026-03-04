"""
app.py — bSafe web server
Flask + Flask-SocketIO backend, serves the control UI and exposes REST + WS API.
"""
# Must be the very first executable lines — before any other imports.
# Without monkey_patch, eventlet cannot intercept stdlib blocking calls
# (time.sleep, socket, threading) and the hub busy-spins at 100% CPU.
import eventlet
eventlet.monkey_patch()

import os
import sys
import json
import time
import logging
import threading
from flask import Flask, render_template, request, jsonify, Response
from flask_socketio import SocketIO, emit

import db
from parser import parse_program, program_is_valid, commands_to_steps
from engine import Engine

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("app")

# ---------------------------------------------------------------------------
# CAN host (optional — runs without CAN for UI development)
# ---------------------------------------------------------------------------
can_host = None
try:
    from bsafe_host import BSafeHost, Mode as CanMode
    can_host = BSafeHost(channel=os.environ.get("CAN_CHANNEL", "can0"),
                         bitrate=int(os.environ.get("CAN_BITRATE", "500000")))
    log.info("CAN host initialised")
except Exception as e:
    log.warning(f"CAN host unavailable: {e} — running in UI-only mode")

# ---------------------------------------------------------------------------
# Flask + SocketIO
# ---------------------------------------------------------------------------
app = Flask(__name__)
app.config["SECRET_KEY"] = os.environ.get("SECRET_KEY", "bsafe-dev-key-change-me")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

# ---------------------------------------------------------------------------
# Engine
# ---------------------------------------------------------------------------
engine = Engine(can_host)

# ---------------------------------------------------------------------------
# CAN callbacks → engine
# ---------------------------------------------------------------------------
if can_host:
    def _on_status(status):
        engine.on_status(status)
    def _on_telemetry(address, rpm, bq_temp_c, ir_uohm, vbat_v, ibat_a):
        engine.on_telemetry(address, rpm, bq_temp_c, ir_uohm, vbat_v, ibat_a)
    def _on_identity(address, schema_ver, hw_hash):
        engine.on_identity(address, schema_ver, hw_hash)
    can_host.on_any_status(_on_status)
    # Extended frame callbacks registered on BSafeHost when it gains multi-frame support

# ---------------------------------------------------------------------------
# Push updates via SocketIO (1 Hz)
# ---------------------------------------------------------------------------
def _push_loop():
    while True:
        try:
            snapshot = engine.get_ui_snapshot()
            socketio.emit("snapshot", snapshot)
        except Exception as e:
            log.debug(f"Push error: {e}")
        time.sleep(1.0)

# ---------------------------------------------------------------------------
# Emergency stop button — GPIO24, active low
# Runs in background thread, fires stop_all() when pressed
# ---------------------------------------------------------------------------
def _estop_loop():
    try:
        import gpiod
        chip = gpiod.Chip('gpiochip0')
        line = chip.get_line(24)
        line.request(consumer='bsafe-estop',
                     type=gpiod.LINE_REQ_EV_FALLING_EDGE,
                     flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        log.info("E-stop button monitoring GPIO24")
        while True:
            if line.event_wait(sec=1):
                ev = line.event_read()
                if ev.type == gpiod.LineEvent.FALLING_EDGE:
                    log.warning("E-STOP PRESSED — stopping all programs")
                    engine.stop_all()
                    socketio.emit("estop", {"source": "gpio24"})
    except Exception as e:
        log.warning(f"E-stop GPIO unavailable: {e}")

# ---------------------------------------------------------------------------
# App startup
# ---------------------------------------------------------------------------
def startup():
    db.init_db()
    engine.start()

    if can_host:
        can_host.connect()

    t = threading.Thread(target=_estop_loop, daemon=True, name="estop")
    t = threading.Thread(target=_push_loop, daemon=True, name="push-loop")
    t.start()

    log.info("bSafe listening on port 2026")

# ---------------------------------------------------------------------------
# Routes — UI
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")

# ---------------------------------------------------------------------------
# Routes — Programs
# ---------------------------------------------------------------------------
@app.route("/api/programs", methods=["GET"])
def api_programs():
    return jsonify(db.get_programs())

@app.route("/api/programs/<int:pid>", methods=["GET"])
def api_program(pid):
    p = db.get_program(pid)
    return jsonify(p) if p else ("Not found", 404)

@app.route("/api/programs", methods=["POST"])
def api_save_program():
    data = request.json or {}
    title = data.get("title", "").strip()
    body  = data.get("body", "").strip()
    pid   = data.get("id")
    if not title or not body:
        return jsonify({"error": "title and body required"}), 400
    limits = db.get_hard_limits()
    cmds, _ = parse_program(body, limits)
    valid = 1 if program_is_valid(cmds) else 0
    new_id = db.save_program(title, body, valid, pid)
    return jsonify({"id": new_id, "valid": valid})

@app.route("/api/programs/<int:pid>", methods=["DELETE"])
def api_delete_program(pid):
    db.delete_program(pid)
    return jsonify({"ok": True})

@app.route("/api/programs/validate", methods=["POST"])
def api_validate():
    data   = request.json or {}
    body   = data.get("body", "")
    limits = db.get_hard_limits()
    cmds, suggestions = parse_program(body, limits)
    valid = program_is_valid(cmds)
    return jsonify({
        "valid": valid,
        "commands": [
            {
                "line":     c.line_num,
                "raw":      c.raw,
                "cmd":      c.cmd,
                "param":    c.param,
                "unit":     c.unit,
                "errors":   c.errors,
                "warnings": c.warnings,
                "clamped":  c.clamped,
            }
            for c in cmds
        ],
        "suggestions": suggestions,
    })

# ---------------------------------------------------------------------------
# Routes — Hard limits
# ---------------------------------------------------------------------------
@app.route("/api/limits", methods=["GET"])
def api_get_limits():
    return jsonify(db.get_hard_limits())

@app.route("/api/limits", methods=["POST"])
def api_save_limits():
    data = request.json or {}
    db.save_hard_limits(data)
    engine.reload_limits()
    return jsonify({"ok": True})

# ---------------------------------------------------------------------------
# Routes — Program execution
# ---------------------------------------------------------------------------
def _get_steps_from_textarea(body: str, limits: dict):
    cmds, _ = parse_program(body, limits)
    if not program_is_valid(cmds):
        return None, "Program has errors — fix before running"
    return commands_to_steps(cmds), None

@app.route("/api/exec/start_all", methods=["POST"])
def api_start_all():
    data    = request.json or {}
    body    = data.get("body", "")
    pid     = data.get("program_id")
    title   = data.get("title", "ad-hoc")
    limits  = db.get_hard_limits()
    steps, err = _get_steps_from_textarea(body, limits)
    if err:
        return jsonify({"error": err}), 400
    snapshot = engine.get_ui_snapshot()
    addresses = [int(a) for a in snapshot["devices"].keys()]
    started = engine.start_program(addresses, steps, pid, title, body)
    return jsonify({"started": started})

@app.route("/api/exec/autostart", methods=["POST"])
def api_autostart():
    data   = request.json or {}
    body   = data.get("body", "")
    pid    = data.get("program_id")
    title  = data.get("title", "ad-hoc")
    limits = db.get_hard_limits()
    steps, err = _get_steps_from_textarea(body, limits)
    if err:
        return jsonify({"error": err}), 400
    engine.start_auto_mode(steps, pid, title, body)
    return jsonify({"ok": True})

@app.route("/api/exec/stop_all", methods=["POST"])
def api_stop_all():
    engine.stop_all()
    return jsonify({"ok": True})

@app.route("/api/exec/apply", methods=["POST"])
def api_apply():
    """Send current CMD (WAIT) to all online devices without starting a program."""
    snapshot = engine.get_ui_snapshot()
    addresses = [int(a) for a in snapshot["devices"].keys()
                 if snapshot["devices"][a]["online"]]
    for addr in addresses:
        engine._send_cmd(addr, mode=4)
    return jsonify({"applied": addresses})

@app.route("/api/device/<int:address>/start", methods=["POST"])
def api_device_start(address):
    data   = request.json or {}
    body   = data.get("body", "")
    pid    = data.get("program_id")
    title  = data.get("title", "ad-hoc")
    limits = db.get_hard_limits()
    steps, err = _get_steps_from_textarea(body, limits)
    if err:
        return jsonify({"error": err}), 400
    started = engine.start_program([address], steps, pid, title, body)
    return jsonify({"started": started})

@app.route("/api/device/<int:address>/pause", methods=["POST"])
def api_device_pause(address):
    engine.pause_device(address)
    return jsonify({"ok": True})

@app.route("/api/device/<int:address>/resume", methods=["POST"])
def api_device_resume(address):
    engine.resume_device(address)
    return jsonify({"ok": True})

@app.route("/api/device/<int:address>/stop", methods=["POST"])
def api_device_stop(address):
    engine.stop_device(address)
    return jsonify({"ok": True})

@app.route("/api/device/<int:address>/unload", methods=["POST"])
def api_device_unload(address):
    engine.unload_device(address)
    return jsonify({"ok": True})

@app.route("/api/device/<int:address>/request_frame", methods=["POST"])
def api_request_frame(address):
    data = request.json or {}
    ftype = data.get("frame_type", 1)
    engine._request_frame(address, ftype)
    return jsonify({"ok": True})

# ---------------------------------------------------------------------------
# Routes — Events log
# ---------------------------------------------------------------------------
@app.route("/api/events", methods=["GET"])
def api_events():
    address = request.args.get("address", type=int)
    limit   = request.args.get("limit", 200, type=int)
    offset  = request.args.get("offset", 0, type=int)
    return jsonify(db.get_events(address, limit, offset))

# ---------------------------------------------------------------------------
# Routes — Snapshot (polling fallback)
# ---------------------------------------------------------------------------
@app.route("/api/snapshot", methods=["GET"])
def api_snapshot():
    return jsonify(engine.get_ui_snapshot())

# ---------------------------------------------------------------------------
# SocketIO events
# ---------------------------------------------------------------------------
@socketio.on("connect")
def on_connect():
    snapshot = engine.get_ui_snapshot()
    emit("snapshot", snapshot)

@socketio.on("request_snapshot")
def on_request_snapshot():
    emit("snapshot", engine.get_ui_snapshot())

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    startup()
    socketio.run(app, host="0.0.0.0", port=2026, debug=False)
