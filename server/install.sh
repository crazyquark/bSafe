#!/usr/bin/env bash
# =============================================================================
# install.sh — bSafe server installer for Raspberry Pi
# Run as root: sudo bash install.sh
# =============================================================================
set -euo pipefail

INSTALL_DIR="/opt/bsafe"
SERVICE_USER="${SUDO_USER:-pi}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
error()   { echo -e "${RED}[ERR]${NC}  $*"; exit 1; }
confirm() { read -r -p "$1 [y/N] " ans; [[ "$ans" =~ ^[Yy]$ ]]; }

# -----------------------------------------------------------------------------
# Check root
# -----------------------------------------------------------------------------
[[ $EUID -eq 0 ]] || error "Run as root: sudo bash install.sh"

info "=== bSafe Control Server Installer ==="
info "Install directory : $INSTALL_DIR"
info "Service user      : $SERVICE_USER"

# -----------------------------------------------------------------------------
# System packages
# -----------------------------------------------------------------------------
info "Updating package lists..."
apt-get update -qq

info "Installing system dependencies..."
apt-get install -y -qq \
    python3 python3-pip python3-venv \
    can-utils \
    net-tools \
    sqlite3

# -----------------------------------------------------------------------------
# SPI + MCP2515 overlay in /boot/firmware/config.txt (Bookworm+)
# Falls back to /boot/config.txt (Buster/Bullseye)
# -----------------------------------------------------------------------------
BOOT_CFG=""
for f in /boot/firmware/config.txt /boot/config.txt; do
    [[ -f "$f" ]] && { BOOT_CFG="$f"; break; }
done

if [[ -z "$BOOT_CFG" ]]; then
    warn "Could not find boot config file — skipping SPI/CAN overlay setup"
else
    info "Boot config: $BOOT_CFG"

    if ! grep -q "dtparam=spi=on" "$BOOT_CFG"; then
        echo "dtparam=spi=on" >> "$BOOT_CFG"
        info "Added: dtparam=spi=on"
    else
        info "SPI already enabled"
    fi

    # Detect oscillator — ask user
    OSC="8000000"
    echo ""
    echo "MCP2515 oscillator frequency:"
    echo "  1) 8 MHz  (default, most common blue modules)"
    echo "  2) 16 MHz (some modules)"
    read -r -p "Select [1/2, default=1]: " osc_choice
    [[ "$osc_choice" == "2" ]] && OSC="16000000"
    info "Using oscillator: ${OSC} Hz"

    if ! grep -q "mcp2515-can0" "$BOOT_CFG"; then
        echo "dtoverlay=mcp2515-can0,oscillator=${OSC},interrupt=25" >> "$BOOT_CFG"
        echo "dtoverlay=spi-bcm2835-overlay" >> "$BOOT_CFG"
        info "Added MCP2515 overlay"
    else
        warn "MCP2515 overlay already present — not modifying"
    fi
fi

# -----------------------------------------------------------------------------
# systemd-networkd CAN interface config (optional, more resilient than rc.local)
# -----------------------------------------------------------------------------
CAN_NETWORK="/etc/systemd/network/80-can.network"
if [[ ! -f "$CAN_NETWORK" ]]; then
    cat > "$CAN_NETWORK" << 'EOF'
[Match]
Name=can0

[CAN]
BitRate=500000
RestartSec=100ms
EOF
    info "Created systemd-networkd CAN config: $CAN_NETWORK"
    systemctl enable systemd-networkd 2>/dev/null || true
    systemctl restart systemd-networkd 2>/dev/null || true
else
    info "CAN network config already exists"
fi

# -----------------------------------------------------------------------------
# Create install directory and copy files
# -----------------------------------------------------------------------------
info "Creating install directory: $INSTALL_DIR"
mkdir -p "$INSTALL_DIR/data"
mkdir -p "$INSTALL_DIR/templates"
mkdir -p "$INSTALL_DIR/static"

info "Copying server files..."
for f in app.py db.py engine.py parser.py bsafe_host.py requirements.txt; do
    src="$SCRIPT_DIR/$f"
    if [[ -f "$src" ]]; then
        cp "$src" "$INSTALL_DIR/$f"
        info "  Copied: $f"
    else
        warn "  Missing: $f — skipping"
    fi
done

# Templates
if [[ -d "$SCRIPT_DIR/templates" ]]; then
    cp -r "$SCRIPT_DIR/templates/." "$INSTALL_DIR/templates/"
    info "Copied templates/"
fi

# Static assets (if any)
if [[ -d "$SCRIPT_DIR/static" ]]; then
    cp -r "$SCRIPT_DIR/static/." "$INSTALL_DIR/static/"
    info "Copied static/"
fi

# Set ownership
chown -R "$SERVICE_USER:$SERVICE_USER" "$INSTALL_DIR"

# -----------------------------------------------------------------------------
# Python virtualenv
# -----------------------------------------------------------------------------
info "Creating Python virtualenv..."
sudo -u "$SERVICE_USER" python3 -m venv "$INSTALL_DIR/venv"

info "Installing Python packages..."
sudo -u "$SERVICE_USER" "$INSTALL_DIR/venv/bin/pip" install \
    --quiet --upgrade pip
sudo -u "$SERVICE_USER" "$INSTALL_DIR/venv/bin/pip" install \
    --quiet -r "$INSTALL_DIR/requirements.txt"

# -----------------------------------------------------------------------------
# Database initialisation
# -----------------------------------------------------------------------------
info "Initialising database..."
sudo -u "$SERVICE_USER" \
    BSAFE_DB="$INSTALL_DIR/data/bsafe.db" \
    "$INSTALL_DIR/venv/bin/python" -c "import db; db.init_db(); print('DB OK')"

# -----------------------------------------------------------------------------
# systemd service
# -----------------------------------------------------------------------------
SERVICE_SRC="$SCRIPT_DIR/bsafe.service"
SERVICE_DST="/etc/systemd/system/bsafe.service"

if [[ -f "$SERVICE_SRC" ]]; then
    # Patch user and path into service file
    sed \
        -e "s|User=pi|User=$SERVICE_USER|g" \
        -e "s|/opt/bsafe|$INSTALL_DIR|g" \
        "$SERVICE_SRC" > "$SERVICE_DST"
    info "Installed systemd service: $SERVICE_DST"
else
    warn "bsafe.service not found — writing inline"
    cat > "$SERVICE_DST" << EOF
[Unit]
Description=bSafe CAN Charger Control Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$INSTALL_DIR
Environment=PYTHONUNBUFFERED=1
Environment=CAN_CHANNEL=can0
Environment=CAN_BITRATE=500000
Environment=BSAFE_DB=$INSTALL_DIR/data/bsafe.db
ExecStartPre=/bin/bash -c 'ip link set can0 up type can bitrate 500000 2>/dev/null || true'
ExecStart=$INSTALL_DIR/venv/bin/python app.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=bsafe

[Install]
WantedBy=multi-user.target
EOF
fi

systemctl daemon-reload
systemctl enable bsafe
systemctl restart bsafe

# -----------------------------------------------------------------------------
# Verify
# -----------------------------------------------------------------------------
sleep 2
if systemctl is-active --quiet bsafe; then
    info "✓ bSafe service is running"
else
    warn "Service may have failed to start — check: journalctl -u bsafe -n 30"
fi

# -----------------------------------------------------------------------------
# Get local IP
# -----------------------------------------------------------------------------
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  bSafe Control Server installed${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "  Web UI : ${GREEN}http://${LOCAL_IP}:2026${NC}"
echo ""
echo "  Useful commands:"
echo "    sudo systemctl status bsafe"
echo "    sudo journalctl -u bsafe -f"
echo "    sudo systemctl restart bsafe"
echo ""
echo -e "${YELLOW}  If MCP2515 overlay was added, a REBOOT is required.${NC}"
echo ""

if confirm "Reboot now?"; then
    reboot
fi
