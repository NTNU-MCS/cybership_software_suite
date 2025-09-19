#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
SERVICE_SRC="$SCRIPT_DIR/../service/fastdds_discovery.service"
SYSTEMD_USER_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
SERVICE_DEST="$SYSTEMD_USER_DIR/fastdds_discovery.service"
FASTDDS_ENV_FILE="$HOME/.config/fastdds"
ROSPROFILE_FILE="$HOME/.rosrc"

echo "Installing Fast DDS Discovery Server user service..."

if [[ ! -f "$SERVICE_SRC" ]]; then
	echo "Error: Service file not found at $SERVICE_SRC" >&2
	exit 1
fi

# Optional sanity checks and friendly warnings
if ! command -v fastdds >/dev/null 2>&1; then
	echo "Warning: 'fastdds' command is not in PATH. The service may fail to start until it's installed." >&2
fi

if [[ ! -f "$FASTDDS_ENV_FILE" ]]; then
	echo "Creating default Fast DDS env file at $FASTDDS_ENV_FILE"
	mkdir -p "$HOME/.config"
	cat > "$FASTDDS_ENV_FILE" <<'EOF'
# Fast DDS Discovery Server environment
# Required: Unique discovery server ID (0-255). Choose per host.
export FASTDDS_DISCOVERY_ID="1"

# Optional: Discovery server port (defaults to 11811 if unset)
export FASTDDS_DISCOVERY_PORT="11811"
EOF
else
	echo "Found existing env file at $FASTDDS_ENV_FILE"
fi

# Ensure ~/.rosrc exists to satisfy ConditionPathExists from the unit
if [[ ! -f "$ROSPROFILE_FILE" ]]; then
	echo "Creating $ROSPROFILE_FILE (empty placeholder for service condition)"
	: > "$ROSPROFILE_FILE"
fi

mkdir -p "$SYSTEMD_USER_DIR"
install -m 0644 "$SERVICE_SRC" "$SERVICE_DEST"
echo "Installed unit to $SERVICE_DEST"

# Enable lingering so the user service can run without a login session
echo "Enabling lingering for user '$USER' (so the service runs at boot)..."
if ! loginctl enable-linger "$USER" >/dev/null 2>&1; then
	if command -v sudo >/dev/null 2>&1; then
		echo "Retrying with sudo to enable lingering..."
		sudo loginctl enable-linger "$USER"
	else
		echo "Warning: Could not enable lingering automatically. Run this manually:" >&2
		echo "  sudo loginctl enable-linger $USER" >&2
	fi
fi

# Reload, enable and start the user service
echo "Reloading systemd user units..."
systemctl --user daemon-reload

echo "Enabling and starting 'fastdds_discovery.service'..."
systemctl --user enable --now fastdds_discovery.service

echo "Service status (short):"
systemctl --user --no-pager --full status fastdds_discovery.service || true

echo "Done. The Fast DDS Discovery Server should now be active."
