#!/usr/bin/env bash
# =============================================================================
# install-cybership.sh
#
# Usage:
#   ./install-cybership.sh [OPTIONS] <vessel_name> [vessel_model]
#
#   vessel_model defaults to vessel_name if not provided.
#   At least one of --install-software or --install-service must be given.
#
# Options:
#   --install-software    Install the Cybership Software Suite via the
#                         upstream lazy_install.sh (curl)
#   --install-service     Install the systemd user unit, env file, and ~/.rosrc
#   -h, --help            Show this help message
#
# Both flags can be combined for a full install in one go:
#   ./install-cybership.sh --install-software --install-service voyager
#
# Examples:
#   ./install-cybership.sh --install-software voyager
#   ./install-cybership.sh --install-service voyager
#   ./install-cybership.sh --install-service voyager enterprise_mk2
#   ./install-cybership.sh --install-software --install-service voyager
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# Colour helpers
# -----------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }
die()     { error "$*"; exit 1; }

# -----------------------------------------------------------------------------
# Defaults
# -----------------------------------------------------------------------------
INSTALL_SOFTWARE=false
INSTALL_SERVICE=false

# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
usage() {
    echo -e "${BOLD}Usage:${NC} $(basename "$0") [OPTIONS] <vessel_name> [vessel_model]"
    echo ""
    echo -e "${BOLD}Arguments:${NC}"
    echo "  vessel_name            Name of the vessel (e.g. voyager)"
    echo "  vessel_model           Model of the vessel (defaults to vessel_name)"
    echo ""
    echo -e "${BOLD}Options:${NC}"
    echo "  --install-software     Install the Cybership Software Suite"
    echo "                         (runs the upstream lazy_install.sh via curl)"
    echo "  --install-service      Install the systemd user unit, ~/.rosrc,"
    echo "                         and ~/.config/cybership/environment"
    echo "  -h, --help             Show this help message"
    echo ""
    echo -e "${BOLD}Both flags can be combined for a full install:${NC}"
    echo "  $(basename "$0") --install-software --install-service voyager"
    echo ""
    echo -e "${BOLD}Examples:${NC}"
    echo "  $(basename "$0") --install-software voyager"
    echo "  $(basename "$0") --install-service voyager"
    echo "  $(basename "$0") --install-service voyager enterprise_mk2"
    echo "  $(basename "$0") --install-software --install-service voyager"
    exit 0
}

POSITIONAL=()
for arg in "$@"; do
    case "$arg" in
        --install-software) INSTALL_SOFTWARE=true ;;
        --install-service)  INSTALL_SERVICE=true ;;
        -h|--help)          usage ;;
        -*)                 die "Unknown option: $arg. Use --help for usage." ;;
        *)                  POSITIONAL+=("$arg") ;;
    esac
done

# Must have at least one action
if [[ "$INSTALL_SOFTWARE" == false && "$INSTALL_SERVICE" == false ]]; then
    error "No action specified. Use --install-software, --install-service, or both."
    echo ""
    usage
fi

# vessel_name is required when installing the service
if [[ "$INSTALL_SERVICE" == true && ${#POSITIONAL[@]} -lt 1 ]]; then
    die "Missing required argument: vessel_name (required for --install-service)"
fi

VESSEL_NAME="${POSITIONAL[0]:-}"
VESSEL_MODEL="${POSITIONAL[1]:-$VESSEL_NAME}"

SERVICE_NAME="ros2-cybership-${VESSEL_NAME}"
SERVICE_FILE="${SERVICE_NAME}.service"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"
ENV_DIR="${HOME}/.config/cybership"
ENV_FILE="${ENV_DIR}/environment"
ROSRC="${HOME}/.rosrc"

LAZY_INSTALL_URL="https://raw.githubusercontent.com/NTNU-MCS/cybership_software_suite/master/lazy_install.sh"

# -----------------------------------------------------------------------------
# Banner
# -----------------------------------------------------------------------------
echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║        Cybership Installer                   ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
echo ""
[[ -n "$VESSEL_NAME"  ]] && info "vessel_name       : ${VESSEL_NAME}"
[[ -n "$VESSEL_MODEL" ]] && info "vessel_model      : ${VESSEL_MODEL}"
info "install software  : ${INSTALL_SOFTWARE}"
info "install service   : ${INSTALL_SERVICE}"
echo ""

# =============================================================================
# BLOCK A – Install Cybership Software Suite
# =============================================================================
if [[ "$INSTALL_SOFTWARE" == true ]]; then
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BOLD} Install Cybership Software Suite                                          ${NC}"
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    info "This will run the upstream lazy_install.sh from:"
    info "  ${LAZY_INSTALL_URL}"
    echo ""
    warn "The script installs ROS 2 packages and builds a colcon workspace."
    warn "It requires sudo for apt installs."
    echo ""
    read -rp "$(echo -e "${CYAN}Proceed with software installation?${NC} [Y/n] ")" sw_confirm
    if [[ "${sw_confirm,,}" == "n" ]]; then
        info "Skipping software installation."
    else
        if ! command -v curl &>/dev/null; then
            die "curl is required but not installed. Install it with: sudo apt install curl"
        fi
        info "Fetching and running lazy_install.sh..."
        echo ""
        bash <(curl -fsSL "${LAZY_INSTALL_URL}")
        echo ""
        success "Cybership Software Suite installed."
    fi
    echo ""
fi

# =============================================================================
# BLOCK B – Install systemd service
# =============================================================================
if [[ "$INSTALL_SERVICE" == true ]]; then
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BOLD} Install systemd service                                                   ${NC}"
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""

    # -------------------------------------------------------------------------
    # Step B1: Write ~/.rosrc
    # -------------------------------------------------------------------------
    echo -e "${BOLD}── Step 1/4: ~/.rosrc ─────────────────────────────────────────────────────${NC}"
    echo ""

    write_rosrc=false
    if [[ -f "$ROSRC" ]]; then
        warn "${ROSRC} already exists."
        read -rp "$(echo -e "${YELLOW}Overwrite?${NC} [y/N] ")" rc_confirm
        [[ "${rc_confirm,,}" == "y" ]] && write_rosrc=true || info "Leaving existing ${ROSRC} untouched."
    else
        write_rosrc=true
    fi

    if [[ "$write_rosrc" == true ]]; then
        info "Writing ${ROSRC}..."
        cat > "$ROSRC" <<'EOF'
source /opt/ros/jazzy/setup.bash
source ~/ros_ws/install/setup.bash
source ~/ros_ws/venv/bin/activate
EOF
        success "${ROSRC} written."
    fi

    # -------------------------------------------------------------------------
    # Step B2: Write environment file
    # -------------------------------------------------------------------------
    echo ""
    echo -e "${BOLD}── Step 2/4: Environment file ─────────────────────────────────────────────${NC}"
    echo ""

    mkdir -p "${ENV_DIR}"

    write_env=false
    if [[ -f "$ENV_FILE" ]]; then
        warn "${ENV_FILE} already exists."
        read -rp "$(echo -e "${YELLOW}Overwrite?${NC} [y/N] ")" env_confirm
        if [[ "${env_confirm,,}" == "y" ]]; then
            write_env=true
        else
            sed -i "s/^export VESSEL_NAME=.*/export VESSEL_NAME=${VESSEL_NAME}/" "$ENV_FILE"
            sed -i "s/^export VESSEL_MODEL=.*/export VESSEL_MODEL=${VESSEL_MODEL}/" "$ENV_FILE"
            info "Updated VESSEL_NAME and VESSEL_MODEL in existing environment file."
        fi
    else
        write_env=true
    fi

    if [[ "$write_env" == true ]]; then
        info "Writing ${ENV_FILE}..."
        cat > "$ENV_FILE" <<EOF
export VESSEL_NAME=${VESSEL_NAME}
export VESSEL_MODEL=${VESSEL_MODEL}
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
export ROS_SUPER_CLIENT=true
EOF
        success "Environment file written."
    fi

    # -------------------------------------------------------------------------
    # Step B3: Linger – must be enabled before any systemctl --user calls
    # -------------------------------------------------------------------------
    echo ""
    echo -e "${BOLD}── Step 3/4: Linger (boot without login) ──────────────────────────────────${NC}"
    echo ""

    if command -v loginctl &>/dev/null; then
        LINGER_STATUS="$(loginctl show-user "$USER" --property=Linger --value 2>/dev/null || echo 'no')"
        if [[ "$LINGER_STATUS" == "yes" ]]; then
            success "Linger already enabled for ${USER}."
        else
            read -rp "$(echo -e "${CYAN}Enable linger${NC} so the service starts at boot without a login? [Y/n] ")" linger_confirm
            if [[ "${linger_confirm,,}" != "n" ]]; then
                loginctl enable-linger "$USER"
                success "Linger enabled for ${USER}."
            else
                warn "Linger not enabled – service will only start when you log in."
                warn "systemctl --user commands may also fail without an active session."
            fi
        fi
    else
        warn "loginctl not found – skipping linger setup."
    fi

    # -------------------------------------------------------------------------
    # Step B4: Write systemd user unit, enable, and start
    # -------------------------------------------------------------------------
    echo ""
    echo -e "${BOLD}── Step 4/4: systemd user unit ────────────────────────────────────────────${NC}"
    echo ""

    mkdir -p "${SYSTEMD_USER_DIR}"

    if [[ -f "${SYSTEMD_USER_DIR}/${SERVICE_FILE}" ]]; then
        warn "Service file already exists: ${SYSTEMD_USER_DIR}/${SERVICE_FILE}"
        read -rp "$(echo -e "${YELLOW}Overwrite?${NC} [y/N] ")" svc_confirm
        [[ "${svc_confirm,,}" == "y" ]] || die "Aborted – service file left unchanged."
        systemctl --user stop    "${SERVICE_NAME}.service" 2>/dev/null || true
        systemctl --user disable "${SERVICE_NAME}.service" 2>/dev/null || true
    fi

    info "Writing ${SYSTEMD_USER_DIR}/${SERVICE_FILE}..."
    cat > "${SYSTEMD_USER_DIR}/${SERVICE_FILE}" <<EOF
[Unit]
Description=Cybership Bringup – ${VESSEL_NAME}
After=network.target

[Service]
Type=simple
EnvironmentFile=${ENV_FILE}
ExecStart=/bin/bash -c 'source ${ROSRC} && ros2 launch cybership_bringup all.launch.py vessel_name:=\${VESSEL_NAME} vessel_model:=\${VESSEL_MODEL}'
Restart=on-failure
RestartSec=5s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF
    success "Service file written."

    info "Reloading systemd user daemon..."
    systemctl --user daemon-reload
    success "Daemon reloaded."

    info "Enabling service..."
    systemctl --user enable "${SERVICE_NAME}.service"
    success "Service enabled (will start on next login / boot)."

    echo ""
    read -rp "$(echo -e "${CYAN}Start the service now?${NC} [Y/n] ")" start_confirm
    if [[ "${start_confirm,,}" != "n" ]]; then
        info "Starting ${SERVICE_NAME}.service..."
        systemctl --user start "${SERVICE_NAME}.service"
        sleep 1
        STATUS="$(systemctl --user is-active "${SERVICE_NAME}.service" 2>/dev/null || true)"
        if [[ "$STATUS" == "active" ]]; then
            success "Service is running."
        else
            warn "Service status: ${STATUS}. Check logs with:"
            warn "  journalctl --user -u ${SERVICE_NAME} -f"
        fi
    fi

    echo ""
fi

# =============================================================================
# Summary
# =============================================================================
echo -e "${BOLD}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║              Done! 🎉                        ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
echo ""

if [[ "$INSTALL_SERVICE" == true ]]; then
    echo -e "  Service file  : ${CYAN}${SYSTEMD_USER_DIR}/${SERVICE_FILE}${NC}"
    echo -e "  Env file      : ${CYAN}${ENV_FILE}${NC}"
    echo -e "  .rosrc        : ${CYAN}${ROSRC}${NC}"
    echo -e "  vessel_name   : ${CYAN}${VESSEL_NAME}${NC}"
    echo -e "  vessel_model  : ${CYAN}${VESSEL_MODEL}${NC}"
    echo ""
    echo -e "${BOLD}Useful commands:${NC}"
    echo "  systemctl --user status  ${SERVICE_NAME}"
    echo "  systemctl --user start   ${SERVICE_NAME}"
    echo "  systemctl --user stop    ${SERVICE_NAME}"
    echo "  systemctl --user restart ${SERVICE_NAME}"
    echo "  journalctl --user -u ${SERVICE_NAME} -f"
fi
echo ""
