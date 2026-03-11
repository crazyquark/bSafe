"""
db.py — SQLite database layer for bSafe host server
All schema creation, migrations, and query helpers live here.
"""
import sqlite3
import time
import json
import os
from contextlib import contextmanager
from datetime import datetime

DB_PATH = os.environ.get("BSAFE_DB", os.path.join(os.path.dirname(__file__), "data", "bsafe.db"))

# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------
@contextmanager
def get_conn():
    conn = sqlite3.connect(DB_PATH, timeout=10, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()

# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------
SCHEMA = """
-- Programs: user-editable charge/discharge scripts
CREATE TABLE IF NOT EXISTS programs (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    title       TEXT    NOT NULL UNIQUE,
    body        TEXT    NOT NULL,         -- raw program text, one command per line
    valid       INTEGER NOT NULL DEFAULT 0, -- 1=green border, 0=red
    created_at  REAL    NOT NULL DEFAULT (unixepoch('now','subsec')),
    updated_at  REAL    NOT NULL DEFAULT (unixepoch('now','subsec'))
);

-- Hard limits: applied as clamps to all programs
CREATE TABLE IF NOT EXISTS hard_limits (
    id                      INTEGER PRIMARY KEY CHECK (id = 1),
    max_charge_v            REAL    NOT NULL DEFAULT 4.25,
    min_discharge_v         REAL    NOT NULL DEFAULT 3.00,
    max_settle_h            REAL    NOT NULL DEFAULT 48.0,
    max_charge_current_a    REAL    NOT NULL DEFAULT 5.0,
    stop_on_fan_fail        INTEGER NOT NULL DEFAULT 1,
    fan_fail_timeout_s      INTEGER NOT NULL DEFAULT 5,
    max_temp_c              INTEGER NOT NULL DEFAULT 60,
    stop_on_temp_exceed     INTEGER NOT NULL DEFAULT 1,
    updated_at              REAL    NOT NULL DEFAULT (unixepoch('now','subsec'))
);

-- Device sessions: one row per device program execution
CREATE TABLE IF NOT EXISTS sessions (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    address         INTEGER NOT NULL,
    program_id      INTEGER REFERENCES programs(id),
    program_title   TEXT,
    program_body    TEXT    NOT NULL,   -- snapshot at start time
    status          TEXT    NOT NULL DEFAULT 'idle',
                                        -- idle/running/paused/stopped/done/error
    current_step    INTEGER NOT NULL DEFAULT 0,
    step_data       TEXT,               -- JSON: step-specific state
    started_at      REAL,
    paused_at       REAL,
    completed_at    REAL,
    updated_at      REAL    NOT NULL DEFAULT (unixepoch('now','subsec'))
);

-- Step results: one row per completed step per session
CREATE TABLE IF NOT EXISTS step_results (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id      INTEGER NOT NULL REFERENCES sessions(id),
    address         INTEGER NOT NULL,
    step_index      INTEGER NOT NULL,
    command         TEXT    NOT NULL,   -- e.g. CHARGE=4.2V
    started_at      REAL    NOT NULL,
    completed_at    REAL,
    result          TEXT,               -- JSON: voltages, IR, capacity, etc.
    notes           TEXT
);

-- Events: all notable state changes and CAN events
CREATE TABLE IF NOT EXISTS events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL    NOT NULL DEFAULT (unixepoch('now','subsec')),
    address     INTEGER,                -- NULL for system events
    session_id  INTEGER REFERENCES sessions(id),
    event_type  TEXT    NOT NULL,
    -- event_type values:
    --   battery_insert, battery_remove
    --   mode_change, error, full, pwr_ok_change
    --   program_start, program_pause, program_resume, program_stop, program_done
    --   step_start, step_complete
    --   cmd_sent, status_rx
    --   fan_fail, temp_exceed
    --   host_start, host_stop
    --   identity_rx, telemetry_rx
    data        TEXT                    -- JSON payload
);

-- Telemetry: raw extended telemetry frames (type 0x01) for capacity integration
CREATE TABLE IF NOT EXISTS telemetry (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL    NOT NULL DEFAULT (unixepoch('now','subsec')),
    address     INTEGER NOT NULL,
    session_id  INTEGER REFERENCES sessions(id),
    vbat_v      REAL,
    ibat_a      REAL,
    rpm         INTEGER,
    bq_temp_c   INTEGER,
    ir_uohm     INTEGER
);

-- Identity frames: device firmware identity on connect
CREATE TABLE IF NOT EXISTS identities (
    address         INTEGER PRIMARY KEY,
    schema_version  INTEGER,
    hw_version_hash INTEGER,
    fw_version      TEXT,
    last_seen       REAL
);
-- Pi network configuration
CREATE TABLE IF NOT EXISTS wifi_config (
    id              INTEGER PRIMARY KEY CHECK (id = 1),
    carrier_ssid    TEXT,
    carrier_pass    TEXT,
    ap_ssid         TEXT    NOT NULL DEFAULT 'bsafe-net',
    ap_pass         TEXT    NOT NULL DEFAULT 'bsafe1234',
    ap_channel      INTEGER NOT NULL DEFAULT 6,
    force_softap    INTEGER NOT NULL DEFAULT 0,
    tcp_port        INTEGER NOT NULL DEFAULT 7000,
    updated_at      REAL    NOT NULL DEFAULT (unixepoch('now','subsec'))
);

-- Master list of WiFi networks to push to all devices
CREATE TABLE IF NOT EXISTS wifi_networks (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ssid        TEXT    NOT NULL UNIQUE,
    password    TEXT    NOT NULL,
    priority    INTEGER NOT NULL DEFAULT 50,
    is_ap       INTEGER NOT NULL DEFAULT 0,
    created_at  REAL    NOT NULL DEFAULT (unixepoch('now','subsec'))
);

-- Per-device WiFi state and pending push queue
CREATE TABLE IF NOT EXISTS device_wifi_state (
    address         INTEGER PRIMARY KEY,
    transport       TEXT    NOT NULL DEFAULT 'unknown',
    ip_addr         TEXT,
    mac             TEXT,
    hostname        TEXT,
    alias           TEXT,
    rssi_dbm        INTEGER,
    connected_ssid  TEXT,
    known_networks  TEXT,
    sync_status     TEXT    NOT NULL DEFAULT 'unknown',
    pending_push    TEXT,
    last_wifi_seen  REAL,
    updated_at      REAL    NOT NULL DEFAULT (unixepoch('now','subsec'))
);
"""

SEED_PROGRAMS = [
    (
        "Full Cycle",
        """DETECT=2.0V
PRECHARGE=3.0V
MEASURE
CHARGE=4.2V
MEASURE
SETTLE=2H
MEASURE
DISCHARGE=3.0V
MEASURE
SETTLE=1H
CHARGE=4.2V
MEASURE""",
    ),
    (
        "Capacity Test",
        """DETECT=2.0V
CHARGE=4.2V
MEASURE
SETTLE=1H
DISCHARGE=3.0V
MEASURE""",
    ),
    (
        "Storage Charge",
        """DETECT=2.0V
CHARGE=3.85V
MEASURE""",
    ),
    (
        "Quick IR Check",
        """DETECT=2.0V
MEASURE""",
    ),
    (
        "Deep Cycle",
        """DETECT=2.0V
PRECHARGE=3.0V
MEASURE
CHARGE=4.2V
MEASURE
SETTLE=8H
MEASURE
DISCHARGE=3.0V
MEASURE
SETTLE=1H
MEASURE
CHARGE=4.2V
MEASURE
SETTLE=2H
MEASURE""",
    ),
    (
        "Recharge Only",
        """DETECT=2.0V
PRECHARGE=3.0V
CHARGE=4.2V""",
    ),
]

def init_db():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    with get_conn() as conn:
        conn.executescript(SCHEMA)
        # Migrations — ADD COLUMN is idempotent via try/except (SQLite has no
        # IF NOT EXISTS for ALTER TABLE ADD COLUMN before version 3.37).
        for col, defn in [
            ("mac",      "TEXT"),
            ("hostname", "TEXT"),
            ("alias",    "TEXT"),
        ]:
            try:
                conn.execute(f"ALTER TABLE device_wifi_state ADD COLUMN {col} {defn}")
            except Exception:
                pass   # column already exists
        # Seed hard limits (single row, id=1)
        conn.execute("""
            INSERT OR IGNORE INTO hard_limits (id) VALUES (1)
        """)
        # Seed wifi_config singleton
        conn.execute("INSERT OR IGNORE INTO wifi_config (id) VALUES (1)")
        # Seed programs
        now = time.time()
        for title, body in SEED_PROGRAMS:
            conn.execute("""
                INSERT OR IGNORE INTO programs (title, body, valid, created_at, updated_at)
                VALUES (?, ?, 1, ?, ?)
            """, (title, body, now, now))
        # Log host start
        conn.execute("""
            INSERT INTO events (event_type, data) VALUES ('host_start', ?)
        """, (json.dumps({"ts": now}),))

# ---------------------------------------------------------------------------
# Program helpers
# ---------------------------------------------------------------------------
def get_programs():
    with get_conn() as conn:
        rows = conn.execute("""
            SELECT id, title, body, valid, created_at, updated_at
            FROM programs ORDER BY updated_at DESC
        """).fetchall()
        return [dict(r) for r in rows]

def get_program(program_id):
    with get_conn() as conn:
        r = conn.execute("SELECT * FROM programs WHERE id=?", (program_id,)).fetchone()
        return dict(r) if r else None

def save_program(title, body, valid=0, program_id=None):
    now = time.time()
    with get_conn() as conn:
        if program_id:
            conn.execute("""
                UPDATE programs SET title=?, body=?, valid=?, updated_at=?
                WHERE id=?
            """, (title, body, valid, now, program_id))
            return program_id
        else:
            cur = conn.execute("""
                INSERT INTO programs (title, body, valid, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?)
            """, (title, body, valid, now, now))
            return cur.lastrowid

def delete_program(program_id):
    with get_conn() as conn:
        # Detach historical sessions before deleting — keeps audit trail intact
        # but removes the FK reference that would otherwise block the delete.
        conn.execute("UPDATE sessions SET program_id=NULL WHERE program_id=?",
                     (program_id,))
        conn.execute("DELETE FROM programs WHERE id=?", (program_id,))

# ---------------------------------------------------------------------------
# Hard limits helpers
# ---------------------------------------------------------------------------
def get_hard_limits():
    with get_conn() as conn:
        r = conn.execute("SELECT * FROM hard_limits WHERE id=1").fetchone()
        return dict(r) if r else {}

def save_hard_limits(limits: dict):
    now = time.time()
    with get_conn() as conn:
        conn.execute("""
            UPDATE hard_limits SET
                max_charge_v=?, min_discharge_v=?, max_settle_h=?,
                max_charge_current_a=?, stop_on_fan_fail=?, fan_fail_timeout_s=?,
                max_temp_c=?, stop_on_temp_exceed=?, updated_at=?
            WHERE id=1
        """, (
            limits.get("max_charge_v", 4.25),
            limits.get("min_discharge_v", 3.00),
            limits.get("max_settle_h", 48.0),
            limits.get("max_charge_current_a", 5.0),
            limits.get("stop_on_fan_fail", 1),
            limits.get("fan_fail_timeout_s", 5),
            limits.get("max_temp_c", 60),
            limits.get("stop_on_temp_exceed", 1),
            now,
        ))

# ---------------------------------------------------------------------------
# Session helpers
# ---------------------------------------------------------------------------
def create_session(address, program_id, program_title, program_body):
    now = time.time()
    with get_conn() as conn:
        cur = conn.execute("""
            INSERT INTO sessions
                (address, program_id, program_title, program_body,
                 status, current_step, started_at, updated_at)
            VALUES (?, ?, ?, ?, 'running', 0, ?, ?)
        """, (address, program_id, program_title, program_body, now, now))
        return cur.lastrowid

def update_session(session_id, **kwargs):
    now = time.time()
    kwargs["updated_at"] = now
    cols = ", ".join(f"{k}=?" for k in kwargs)
    vals = list(kwargs.values()) + [session_id]
    with get_conn() as conn:
        conn.execute(f"UPDATE sessions SET {cols} WHERE id=?", vals)

def get_active_session(address):
    with get_conn() as conn:
        r = conn.execute("""
            SELECT * FROM sessions
            WHERE address=? AND status IN ('running','paused')
            ORDER BY started_at DESC LIMIT 1
        """, (address,)).fetchone()
        return dict(r) if r else None

def get_all_active_sessions():
    with get_conn() as conn:
        rows = conn.execute("""
            SELECT * FROM sessions WHERE status IN ('running','paused')
        """).fetchall()
        return [dict(r) for r in rows]

# ---------------------------------------------------------------------------
# Step result helpers
# ---------------------------------------------------------------------------
def record_step_result(session_id, address, step_index, command,
                       started_at, result: dict, notes=None):
    with get_conn() as conn:
        conn.execute("""
            INSERT INTO step_results
                (session_id, address, step_index, command,
                 started_at, completed_at, result, notes)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (session_id, address, step_index, command,
              started_at, time.time(), json.dumps(result), notes))

# ---------------------------------------------------------------------------
# Event logging
# ---------------------------------------------------------------------------
def log_event(event_type, address=None, session_id=None, data=None):
    with get_conn() as conn:
        conn.execute("""
            INSERT INTO events (address, session_id, event_type, data)
            VALUES (?, ?, ?, ?)
        """, (address, session_id, event_type,
              json.dumps(data) if data else None))

def get_events(address=None, limit=200, offset=0):
    with get_conn() as conn:
        if address is not None:
            rows = conn.execute("""
                SELECT * FROM events WHERE address=? OR address IS NULL
                ORDER BY ts DESC LIMIT ? OFFSET ?
            """, (address, limit, offset)).fetchall()
        else:
            rows = conn.execute("""
                SELECT * FROM events ORDER BY ts DESC LIMIT ? OFFSET ?
            """, (limit, offset)).fetchall()
        return [dict(r) for r in rows]

# ---------------------------------------------------------------------------
# Telemetry logging
# ---------------------------------------------------------------------------
def log_telemetry(address, session_id, vbat_v, ibat_a,
                  rpm=None, bq_temp_c=None, ir_uohm=None):
    with get_conn() as conn:
        conn.execute("""
            INSERT INTO telemetry
                (address, session_id, vbat_v, ibat_a, rpm, bq_temp_c, ir_uohm)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (address, session_id, vbat_v, ibat_a, rpm, bq_temp_c, ir_uohm))

# ---------------------------------------------------------------------------
# Identity
# ---------------------------------------------------------------------------
def upsert_identity(address, schema_version, hw_version_hash, fw_version=None):
    with get_conn() as conn:
        conn.execute("""
            INSERT INTO identities
                (address, schema_version, hw_version_hash, fw_version, last_seen)
            VALUES (?, ?, ?, ?, unixepoch('now','subsec'))
            ON CONFLICT(address) DO UPDATE SET
                schema_version=excluded.schema_version,
                hw_version_hash=excluded.hw_version_hash,
                fw_version=excluded.fw_version,
                last_seen=excluded.last_seen
        """, (address, schema_version, hw_version_hash, fw_version))


# ---------------------------------------------------------------------------
# WiFi config helpers
# ---------------------------------------------------------------------------

def get_wifi_config() -> dict:
    with get_conn() as conn:
        r = conn.execute("SELECT * FROM wifi_config WHERE id=1").fetchone()
        return dict(r) if r else {}

def save_wifi_config(**kwargs):
    """Update any subset of wifi_config columns."""
    if not kwargs:
        return
    cols = ", ".join(f"{k}=?" for k in kwargs)
    vals = list(kwargs.values())
    with get_conn() as conn:
        conn.execute(
            f"UPDATE wifi_config SET {cols}, updated_at=unixepoch('now','subsec') WHERE id=1",
            vals)

def get_wifi_networks() -> list:
    with get_conn() as conn:
        rows = conn.execute(
            "SELECT * FROM wifi_networks ORDER BY priority ASC, id ASC").fetchall()
        return [dict(r) for r in rows]

def upsert_wifi_network(ssid: str, password: str,
                        priority: int = 50, is_ap: bool = False) -> int:
    with get_conn() as conn:
        conn.execute("""
            INSERT INTO wifi_networks (ssid, password, priority, is_ap)
            VALUES (?, ?, ?, ?)
            ON CONFLICT(ssid) DO UPDATE SET
                password=excluded.password,
                priority=excluded.priority,
                is_ap=excluded.is_ap
        """, (ssid, password, priority, int(is_ap)))
        r = conn.execute("SELECT id FROM wifi_networks WHERE ssid=?", (ssid,)).fetchone()
        return r[0]

def delete_wifi_network(network_id: int):
    with get_conn() as conn:
        conn.execute("DELETE FROM wifi_networks WHERE id=? AND is_ap=0", (network_id,))

def get_device_wifi_state(address: int) -> dict:
    with get_conn() as conn:
        r = conn.execute(
            "SELECT * FROM device_wifi_state WHERE address=?", (address,)).fetchone()
        return dict(r) if r else {}

def upsert_device_wifi_state(address: int, **kwargs):
    with get_conn() as conn:
        conn.execute(
            "INSERT OR IGNORE INTO device_wifi_state (address) VALUES (?)", (address,))
        if kwargs:
            cols = ", ".join(f"{k}=?" for k in kwargs)
            vals = list(kwargs.values()) + [address]
            conn.execute(
                f"UPDATE device_wifi_state SET {cols}, updated_at=unixepoch('now','subsec') WHERE address=?",
                vals)

def get_pending_push(address: int) -> list:
    with get_conn() as conn:
        r = conn.execute(
            "SELECT pending_push FROM device_wifi_state WHERE address=?",
            (address,)).fetchone()
        if r and r[0]:
            return json.loads(r[0])
        return []

def set_pending_push(address: int, networks: list):
    upsert_device_wifi_state(
        address,
        pending_push=json.dumps(networks) if networks else None,
        sync_status="pending")

def clear_pending_push(address: int):
    upsert_device_wifi_state(address, pending_push=None, sync_status="synced")

def all_device_wifi_states() -> list:
    with get_conn() as conn:
        rows = conn.execute(
            "SELECT * FROM device_wifi_state ORDER BY address").fetchall()
        return [dict(r) for r in rows]

def get_device_alias(mac: str) -> str | None:
    """Return the user-set alias for a device MAC, or None."""
    with get_conn() as conn:
        row = conn.execute(
            "SELECT alias FROM device_wifi_state WHERE mac=?", (mac,)
        ).fetchone()
        return row["alias"] if row else None

def set_device_alias(mac: str, alias: str | None):
    """Set (or clear) the alias for a device identified by MAC."""
    with get_conn() as conn:
        conn.execute(
            "UPDATE device_wifi_state SET alias=?, updated_at=unixepoch('now','subsec') WHERE mac=?",
            (alias or None, mac)
        )
