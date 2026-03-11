"""
network_blueprint.py — Flask blueprint: /network

REST API:
  GET  /network/api/status              — Pi network status + AP clients
  POST /network/api/carrier             — set carrier SSID/password
  POST /network/api/softap              — set AP SSID/password/channel
  POST /network/api/force_softap        — force AP mode immediately
  POST /network/api/release_softap      — release force, retry carrier
  POST /network/api/retry_carrier       — retry carrier without releasing force

  GET  /network/api/networks            — master network list
  POST /network/api/networks            — add/update a network {ssid, password, priority}
  DELETE /network/api/networks/<id>     — remove a network (non-AP only)

  GET  /network/api/devices             — per-device WiFi state + sync status
  POST /network/api/devices/<addr>/sync — trigger manual sync to one device

  GET  /network/                        — HTML management UI
"""

from flask import Blueprint, jsonify, request
import logging
import db

log = logging.getLogger("net_bp")

net_bp = Blueprint("network", __name__, url_prefix="/network")

_net_mgr      = None   # NetworkManager — set by init_network()
_wifi_host    = None   # BSafeWiFiHost  — set by init_network()
_mdns_browser = None   # MdnsBrowser    — set by init_network() on Pi only


def init_network(engine, socketio=None, force_softap: bool = False):
    """
    Call once in app.py startup(), before registering blueprint.
    Returns (network_manager, wifi_host) for wiring into engine._send_cmd.
    """
    global _net_mgr, _wifi_host, _mdns_browser

    from network_manager import NetworkManager
    from bsafe_wifi_host import BSafeWiFiHost
    from mdns_discovery import start_browser as _start_mdns_browser

    # Check db for persisted force_softap flag
    cfg = db.get_wifi_config()
    db_force = bool(cfg.get("force_softap", 0))
    effective_force = force_softap or db_force

    _net_mgr = NetworkManager(force_softap=effective_force, socketio=socketio)
    _net_mgr.start()

    # WiFi TCP server — bind IP determined by network mode
    # Start on 0.0.0.0 initially; rebind after network_manager settles
    port = cfg.get("tcp_port", 7000)
    _wifi_host = BSafeWiFiHost(bind_ip="0.0.0.0", port=port)

    # Register WiFi callbacks into engine (same as CAN host)
    def _on_status(status, wifi_mac=""):
        engine.on_status(status, wifi_mac=wifi_mac)

    def _on_telem(address, rpm, bq_temp_c, ir_uohm, vbat_v, ibat_a, wifi_mac=""):
        engine.on_telemetry(address, rpm, bq_temp_c, ir_uohm, vbat_v, ibat_a,
                            wifi_mac=wifi_mac)

    def _on_ir(address, frame, wifi_mac=""):
        snap = engine.get_ui_snapshot()
        vbat_v = 0.0
        if wifi_mac:
            mac_int = int(wifi_mac.replace(":", ""), 16)
            dev = snap["devices"].get(mac_int)
            if dev:
                vbat_v = dev.get("vbat_v", 0.0)
        if not vbat_v:
            for dev in snap["devices"].values():
                if dev.get("address") == address:
                    vbat_v = dev.get("vbat_v", 0.0)
                    break
        from bsafe_frames import corrected_ir_uohm
        ir_corrected = int(corrected_ir_uohm(float(frame.ir_uohm_raw), vbat_v))
        engine.on_telemetry(address, rpm=0, bq_temp_c=0,
                            ir_uohm=ir_corrected, vbat_v=vbat_v, ibat_a=0.0,
                            wifi_mac=wifi_mac)

    def _on_identity(address, schema_ver, hw_hash):
        engine.on_identity(address, schema_ver, hw_hash)

    def _on_wifi_identity(address, mac_str):
        engine.on_wifi_identity(address, mac_str)

    _wifi_host.on_any_status(_on_status)
    _wifi_host.on_telemetry(_on_telem)
    _wifi_host.on_ir(_on_ir)
    _wifi_host.on_identity(_on_identity)
    _wifi_host.on_wifi_identity(_on_wifi_identity)
    _wifi_host.start()

    # Wire WiFi host into engine so _send_cmd routes to right transport
    engine.set_wifi_host(_wifi_host)

    # Start mDNS browser as daemon (no-op if zeroconf not installed)
    _mdns_browser = _start_mdns_browser(_wifi_host)
    if _mdns_browser:
        log.info("mDNS browser active")
    else:
        log.info("mDNS browser not available (install python-zeroconf on Pi)")

    log.info(f"Network init complete (force_softap={effective_force})")
    return _net_mgr, _wifi_host


def _require_mgr():
    if _net_mgr is None:
        return jsonify({"error": "Network manager not initialised"}), 503
    return None


# ---------------------------------------------------------------------------
# Network status
# ---------------------------------------------------------------------------
@net_bp.route("/api/status", methods=["GET"])
def api_status():
    err = _require_mgr()
    if err:
        return err
    return jsonify(_net_mgr.status())


# ---------------------------------------------------------------------------
# Carrier config
# ---------------------------------------------------------------------------
@net_bp.route("/api/carrier", methods=["POST"])
def api_set_carrier():
    data = request.json or {}
    ssid = data.get("ssid", "").strip()
    pw   = data.get("password", "").strip()
    if not ssid:
        return jsonify({"error": "ssid required"}), 400
    db.save_wifi_config(carrier_ssid=ssid, carrier_pass=pw)
    # Also add to master device network list so devices get it
    db.upsert_wifi_network(ssid, pw, priority=10, is_ap=False)
    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# SoftAP config
# ---------------------------------------------------------------------------
@net_bp.route("/api/softap", methods=["POST"])
def api_set_softap():
    data    = request.json or {}
    ssid    = data.get("ssid", "").strip()
    pw      = data.get("password", "").strip()
    channel = int(data.get("channel", 6))
    if ssid:
        db.save_wifi_config(ap_ssid=ssid, ap_pass=pw, ap_channel=channel)
    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# Force / release softAP
# ---------------------------------------------------------------------------
@net_bp.route("/api/force_softap", methods=["POST"])
def api_force_softap():
    err = _require_mgr()
    if err:
        return err
    _net_mgr.force_softap()
    return jsonify({"ok": True, "mode": "softap"})


@net_bp.route("/api/release_softap", methods=["POST"])
def api_release_softap():
    err = _require_mgr()
    if err:
        return err
    _net_mgr.release_softap()
    return jsonify({"ok": True})


@net_bp.route("/api/retry_carrier", methods=["POST"])
def api_retry_carrier():
    err = _require_mgr()
    if err:
        return err
    _net_mgr.retry_carrier()
    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# Master network list
# ---------------------------------------------------------------------------
@net_bp.route("/api/networks", methods=["GET"])
def api_networks():
    return jsonify(db.get_wifi_networks())


@net_bp.route("/api/networks", methods=["POST"])
def api_add_network():
    data     = request.json or {}
    ssid     = data.get("ssid", "").strip()
    pw       = data.get("password", "").strip()
    priority = int(data.get("priority", 50))
    if not ssid:
        return jsonify({"error": "ssid required"}), 400
    net_id = db.upsert_wifi_network(ssid, pw, priority=priority, is_ap=False)
    # Mark all WiFi-connected devices as pending re-sync
    _mark_devices_pending(ssid, pw)
    return jsonify({"ok": True, "id": net_id})


@net_bp.route("/api/networks/<int:nid>", methods=["DELETE"])
def api_delete_network(nid):
    db.delete_wifi_network(nid)
    return jsonify({"ok": True})


def _mark_devices_pending(ssid: str, pw: str):
    """Queue new network push to all known WiFi devices."""
    if _wifi_host is None:
        return
    import json
    connected = set(_wifi_host.connected_macs())
    for state in db.all_device_wifi_states():
        addr = state["address"]
        mac  = state.get("mac") or ""
        known_raw = state.get("known_networks") or "[]"
        try:
            known = set(json.loads(known_raw))
        except Exception:
            known = set()
        if ssid not in known:
            pending = db.get_pending_push(addr)
            if not any(p["ssid"] == ssid for p in pending):
                pending.append({"ssid": ssid, "password": pw})
                db.set_pending_push(addr, pending)
            if mac and mac in connected:
                _wifi_host.push_networks_to_device(mac)


# ---------------------------------------------------------------------------
# Per-device WiFi state
# ---------------------------------------------------------------------------
@net_bp.route("/api/devices", methods=["GET"])
def api_device_states():
    states = db.all_device_wifi_states()
    connected_macs = set(_wifi_host.connected_macs()) if _wifi_host else set()
    connected_addrs = set(_wifi_host.connected_addresses()) if _wifi_host else set()
    for s in states:
        mac = s.get("mac", "")
        s["currently_connected"] = (mac and mac in connected_macs) or s["address"] in connected_addrs
    return jsonify(states)


@net_bp.route("/api/devices/<int:addr>/sync", methods=["POST"])
def api_device_sync(addr):
    if _wifi_host is None:
        return jsonify({"error": "WiFi host not running"}), 503
    state = db.get_device_wifi_state(addr)
    mac = (state or {}).get("mac") or ""
    if not mac or not _wifi_host.is_connected(mac):
        return jsonify({"error": f"Device {addr} not connected over WiFi"}), 404
    _wifi_host.push_networks_to_device(mac)
    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# mDNS scan cache
# ---------------------------------------------------------------------------
@net_bp.route("/api/scan", methods=["GET"])
def api_scan():
    """
    Return the current mDNS browser cache — all _bsafe._tcp services currently
    visible on the network.  Does not trigger a new scan; reflects what the
    background ServiceBrowser has already resolved.
    Returns 200 with an empty services list if the browser is not running.
    """
    if _mdns_browser is None:
        return jsonify({"running": False, "services": []})
    return jsonify({"running": True, "services": _mdns_browser.get_cache()})


# ---------------------------------------------------------------------------
# HTML UI
# ---------------------------------------------------------------------------
_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>bSafe Network</title>
<style>
:root{--bg:#0f1117;--card:#1a1d27;--border:#2a2d3a;--accent:#4fc3f7;
      --red:#ef5350;--green:#66bb6a;--yellow:#ffa726;--text:#e0e0e0}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:monospace;padding:16px}
h1{color:var(--accent);margin-bottom:20px;font-size:1.2rem}
h2{font-size:0.95rem;color:var(--accent);margin-bottom:12px;
   border-bottom:1px solid var(--border);padding-bottom:6px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(340px,1fr));gap:16px;margin-bottom:20px}
.card{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:16px}
.row{display:flex;justify-content:space-between;align-items:center;
     margin-bottom:6px;font-size:0.83rem}
.label{color:#888}
.val{color:var(--text)}
.ok{color:var(--green)}.warn{color:var(--yellow)}.err{color:var(--red)}
button{background:var(--card);border:1px solid var(--border);color:var(--text);
       padding:6px 14px;border-radius:4px;cursor:pointer;font-family:monospace;
       font-size:0.82rem;margin-top:4px}
button:hover{border-color:var(--accent);color:var(--accent)}
button.red:hover{border-color:var(--red);color:var(--red)}
button.green:hover{border-color:var(--green);color:var(--green)}
.btn-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
input[type=text],input[type=password],input[type=number]{
  background:var(--bg);border:1px solid var(--border);color:var(--text);
  padding:5px 8px;border-radius:3px;font-family:monospace;font-size:0.82rem;
  width:100%;margin-top:3px;margin-bottom:8px}
label{font-size:0.82rem;color:#aaa}
.badge{font-size:0.7rem;padding:2px 6px;border-radius:3px;margin-left:6px}
.badge.ap{background:#1a2744;color:var(--accent);border:1px solid var(--accent)}
.badge.carrier{background:#1a2a1a;color:var(--green);border:1px solid var(--green)}
.badge.forced{background:#2a1a1a;color:var(--red);border:1px solid var(--red)}
table{width:100%;border-collapse:collapse;font-size:0.8rem;margin-top:8px}
th{text-align:left;color:#888;font-weight:normal;padding:4px 8px;
   border-bottom:1px solid var(--border)}
td{padding:4px 8px;border-bottom:1px solid #1a1d27}
tr:hover td{background:#1f2235}
.sep{height:1px;background:var(--border);margin:16px 0}
</style>
</head>
<body>
<h1>bSafe Network Management <a href="/" style="color:#555;font-size:0.8rem">← main UI</a></h1>

<div class="grid">

<!-- Pi Network Status -->
<div class="card" id="status-card">
  <h2>Pi Network Status</h2>
  <div id="status-body"><div class="row"><span class="label">Loading…</span></div></div>
  <div class="btn-row" id="status-btns"></div>
</div>

<!-- Carrier Config -->
<div class="card">
  <h2>Carrier Network</h2>
  <label>SSID<input type="text" id="c-ssid" placeholder="HomeRouter"></label>
  <label>Password<input type="password" id="c-pass" placeholder=""></label>
  <div class="btn-row">
    <button class="green" onclick="saveCarrier()">Save &amp; apply</button>
  </div>
</div>

<!-- SoftAP Config -->
<div class="card">
  <h2>SoftAP Settings</h2>
  <label>SSID<input type="text" id="ap-ssid" value="bsafe-net"></label>
  <label>Password<input type="password" id="ap-pass" value="bsafe1234"></label>
  <label>Channel<input type="number" id="ap-ch" value="6" min="1" max="13"></label>
  <div class="btn-row">
    <button onclick="saveSoftAP()">Save AP config</button>
  </div>
</div>

</div><!-- /grid -->

<!-- Master network list -->
<div class="card" style="margin-bottom:16px">
  <h2>Master Network List <span style="color:#555;font-size:0.75rem">pushed to all devices on sync</span></h2>
  <table>
    <thead><tr><th>SSID</th><th>Priority</th><th>Type</th><th></th></tr></thead>
    <tbody id="net-table"><tr><td colspan="4" style="color:#555">Loading…</td></tr></tbody>
  </table>
  <div class="sep"></div>
  <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:flex-end">
    <div><label>SSID<input type="text" id="n-ssid" style="width:180px" placeholder="NewNetwork"></label></div>
    <div><label>Password<input type="password" id="n-pass" style="width:160px"></label></div>
    <div><label>Priority<input type="number" id="n-pri" value="50" min="1" max="100" style="width:70px"></label></div>
    <button class="green" onclick="addNetwork()" style="margin-bottom:8px">Add network</button>
  </div>
</div>

<!-- Device WiFi state -->
<div class="card">
  <h2>Device WiFi State</h2>
  <table>
    <thead><tr><th>Addr</th><th>Transport</th><th>IP</th><th>Hostname</th>
                <th>SSID</th><th>Sync</th><th>Last seen</th><th></th></tr></thead>
    <tbody id="dev-table"><tr><td colspan="8" style="color:#555">Loading…</td></tr></tbody>
  </table>
</div>

<!-- mDNS scan results -->
<div class="card" style="margin-top:16px">
  <h2>mDNS Discovered Devices <span style="color:#555;font-size:0.75rem">_bsafe._tcp.local.</span></h2>
  <table>
    <thead><tr><th>Addr</th><th>Hostname</th><th>IP</th><th>Port</th><th>Last seen</th></tr></thead>
    <tbody id="scan-table"><tr><td colspan="5" style="color:#555">Loading…</td></tr></tbody>
  </table>
</div>

<script>
const api = (url,m,b) => fetch(url,{method:m||'GET',
  headers:b?{'Content-Type':'application/json'}:{},
  body:b?JSON.stringify(b):null}).then(r=>r.json());

function fmtTime(ts){
  if(!ts) return '—';
  const d=new Date(ts*1000);
  return d.toLocaleTimeString();
}

async function refreshStatus(){
  const s = await api('/network/api/status');
  const b = document.getElementById('status-body');
  const modeClass = s.mode==='carrier'?'ok':s.mode==='softap'?'warn':'err';
  b.innerHTML = `
    <div class="row"><span class="label">Mode</span>
      <span class="val ${modeClass}">${s.mode.toUpperCase()}
        ${s.force_softap?'<span class="badge forced">FORCED</span>':''}</span></div>
    <div class="row"><span class="label">IP</span>
      <span class="val">${s.ip||'—'}</span></div>
    <div class="row"><span class="label">SSID</span>
      <span class="val">${s.ssid||'—'}</span></div>
    <div class="row"><span class="label">Interface</span>
      <span class="val">${s.wlan_iface}</span></div>
    <div class="row"><span class="label">TCP port</span>
      <span class="val">${s.tcp_port}</span></div>
    ${s.mode==='softap'&&s.ap_clients.length?`
    <div style="margin-top:8px;font-size:0.8rem;color:#888">AP Clients</div>
    ${s.ap_clients.map(c=>`
      <div class="row"><span class="label">${c.ip}</span>
        <span class="val">${c.hostname||c.mac}</span></div>`).join('')}
    `:''}`;

  const btns = document.getElementById('status-btns');
  if(s.mode==='softap'&&!s.force_softap){
    btns.innerHTML = `<button class="green" onclick="retryCarrier()">↺ Retry carrier</button>
      <button class="red" onclick="forceSoftAP()">⚡ Force softAP</button>`;
  } else if(s.force_softap){
    btns.innerHTML = `<button class="green" onclick="releaseSoftAP()">✓ Release softAP</button>`;
  } else {
    btns.innerHTML = `<button class="red" onclick="forceSoftAP()">⚡ Force softAP</button>`;
  }

  // Pre-fill carrier and AP fields from status
  if(s.carrier_ssid) document.getElementById('c-ssid').value=s.carrier_ssid;
  if(s.ap_ssid)      document.getElementById('ap-ssid').value=s.ap_ssid;
}

async function refreshNetworks(){
  const nets = await api('/network/api/networks');
  const tb = document.getElementById('net-table');
  if(!nets.length){
    tb.innerHTML='<tr><td colspan="4" style="color:#555">No networks</td></tr>';
    return;
  }
  tb.innerHTML = nets.map(n=>`
    <tr>
      <td>${n.ssid}</td>
      <td>${n.priority}</td>
      <td>${n.is_ap?'<span class="badge ap">AP</span>':'carrier'}</td>
      <td>${n.is_ap?'':`<button class="red" onclick="delNetwork(${n.id})">✕</button>`}</td>
    </tr>`).join('');
}

async function refreshDevices(){
  const devs = await api('/network/api/devices');
  const tb = document.getElementById('dev-table');
  if(!devs.length){
    tb.innerHTML='<tr><td colspan="8" style="color:#555">No WiFi devices seen</td></tr>';
    return;
  }
  tb.innerHTML = devs.map(d=>{
    const sc = d.sync_status==='synced'?'ok':d.sync_status==='pending'?'warn':'';
    const cc = d.currently_connected?'<span class="badge carrier">LIVE</span>':'';
    return `<tr>
      <td>${d.address}${cc}</td>
      <td>${d.transport}</td>
      <td>${d.ip_addr||'—'}</td>
      <td>${d.hostname||'—'}</td>
      <td>${d.connected_ssid||'—'}</td>
      <td class="${sc}">${d.sync_status}</td>
      <td>${fmtTime(d.last_wifi_seen)}</td>
      <td>${d.currently_connected
        ?`<button onclick="syncDevice(${d.address})">↑ Sync</button>`
        :'offline'}</td>
    </tr>`;
  }).join('');
}

async function refreshScan(){
  const res = await api('/network/api/scan');
  const tb = document.getElementById('scan-table');
  if(!res.running){
    tb.innerHTML='<tr><td colspan="5" style="color:#555">mDNS browser not running</td></tr>';
    return;
  }
  if(!res.services.length){
    tb.innerHTML='<tr><td colspan="5" style="color:#555">No devices discovered yet</td></tr>';
    return;
  }
  tb.innerHTML = res.services.map(s=>`
    <tr>
      <td>${s.address}</td>
      <td>${s.hostname}</td>
      <td>${s.ip}</td>
      <td>${s.port}</td>
      <td>${fmtTime(s.last_seen)}</td>
    </tr>`).join('');
}

async function saveCarrier(){
  const ssid=document.getElementById('c-ssid').value.trim();
  const pass=document.getElementById('c-pass').value;
  if(!ssid){alert('SSID required');return;}
  await api('/network/api/carrier','POST',{ssid,password:pass});
  await api('/network/api/retry_carrier','POST');
  refreshStatus();
}

async function saveSoftAP(){
  const ssid=document.getElementById('ap-ssid').value.trim();
  const pass=document.getElementById('ap-pass').value;
  const ch=parseInt(document.getElementById('ap-ch').value)||6;
  await api('/network/api/softap','POST',{ssid,password:pass,channel:ch});
  alert('Saved — restart softAP to apply');
}

async function forceSoftAP(){
  await api('/network/api/force_softap','POST');
  setTimeout(refreshStatus,1500);
}
async function releaseSoftAP(){
  await api('/network/api/release_softap','POST');
  setTimeout(refreshStatus,2000);
}
async function retryCarrier(){
  await api('/network/api/retry_carrier','POST');
  setTimeout(refreshStatus,3000);
}

async function addNetwork(){
  const ssid=document.getElementById('n-ssid').value.trim();
  const pass=document.getElementById('n-pass').value;
  const pri=parseInt(document.getElementById('n-pri').value)||50;
  if(!ssid){alert('SSID required');return;}
  await api('/network/api/networks','POST',{ssid,password:pass,priority:pri});
  document.getElementById('n-ssid').value='';
  document.getElementById('n-pass').value='';
  refreshNetworks();
}

async function delNetwork(id){
  if(!confirm('Remove this network?'))return;
  await api(`/network/api/networks/${id}`,'DELETE');
  refreshNetworks();
}

async function syncDevice(addr){
  await api(`/network/api/devices/${addr}/sync`,'POST');
  setTimeout(refreshDevices,1500);
}

function refresh(){
  refreshStatus();
  refreshNetworks();
  refreshDevices();
  refreshScan();
}

refresh();
setInterval(refresh,5000);
</script>
</body>
</html>"""


@net_bp.route("/", methods=["GET"])
@net_bp.route("", methods=["GET"])
def net_ui():
    return _HTML
