#!/usr/bin/env bash
set -e
set -o pipefail

##############################################
# USER‑CONFIGURABLE VARIABLES
##############################################

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_WORKSPACE="${ROS_WORKSPACE:-$HOME/ros_ws}"
REPO_URL="${REPO_URL:-https://github.com/NTNU-MCS/cybership_software_suite}"
REPO_NAME="${REPO_NAME:-cybership_software_suite}"
VENV_NAME="${VENV_NAME:-venv}"

##############################################
# HELPER FUNCTIONS
##############################################

msg() {
    echo -e "\n\033[1;32m[INFO]\033[0m $1"
}

warn() {
    echo -e "\n\033[1;33m[WARNING]\033[0m $1"
}

err() {
    echo -e "\n\033[1;31m[ERROR]\033[0m $1\n"
    exit 1
}

##############################################
# CHECK ENVIRONMENT
##############################################

msg "Checking system compatibility..."

# Ubuntu check
if [[ ! -f /etc/lsb-release ]]; then
    err "This script must be run on Ubuntu."
fi

# ROS installation check
if [[ ! -d "/opt/ros/$ROS_DISTRO" ]]; then
    err "ROS 2 ($ROS_DISTRO) is not installed. Install ROS before running this script."
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

msg "System looks good."

##############################################
# INSTALL REQUIRED PACKAGES
##############################################

msg "Installing required apt packages..."
sudo apt update
sudo apt install -y git python3-venv python3-pip build-essential \
    ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-ros-base ros-dev-tools

##############################################
# ROSDEP INIT
##############################################
msg "Initializing rosdep (safe to run multiple times)..."

sudo rosdep init 2>/dev/null || true
rosdep update

##############################################
# WORKSPACE SETUP
##############################################

msg "Checking ROS workspace at: $ROS_WORKSPACE"

if [[ -d "$ROS_WORKSPACE" ]]; then
    msg "Workspace already exists."

    if [[ ! -d "$ROS_WORKSPACE/src" ]]; then
        msg "src/ folder missing — creating it..."
        mkdir -p "$ROS_WORKSPACE/src"
    fi
else
    msg "Workspace does not exist. Creating it..."
    mkdir -p "$ROS_WORKSPACE/src"
fi

cd "$ROS_WORKSPACE"

##############################################
# CLONE OR UPDATE REPOSITORY
##############################################

if [[ ! -d "$ROS_WORKSPACE/src/$REPO_NAME" ]]; then
    msg "Cloning repository: $REPO_URL"
    git clone "$REPO_URL" "$ROS_WORKSPACE/src/$REPO_NAME"
else
    msg "Repository already exists — pulling latest changes..."
    cd "$ROS_WORKSPACE/src/$REPO_NAME"
    git pull --rebase --autostash || warn "Could not automatically rebase — repository may contain local changes."
fi

msg "Updating submodules..."
cd "$ROS_WORKSPACE/src/$REPO_NAME"
git submodule update --init --recursive

##############################################
# PYTHON VIRTUAL ENVIRONMENT
##############################################

cd "$ROS_WORKSPACE"

if [[ ! -d "$VENV_NAME" ]]; then
    msg "Creating Python virtual environment..."
    python3 -m venv "$VENV_NAME" --system-site-packages --symlinks
else
    msg "Python virtual environment already exists."
fi

source "$VENV_NAME/bin/activate"
touch "$VENV_NAME/COLCON_IGNORE"

##############################################
# INSTALL PYTHON DEPENDENCIES
##############################################

msg "Installing Python dependencies..."

find "src/$REPO_NAME" -name "requirements*txt" -print0 | \
while IFS= read -r -d '' req; do
    msg "Installing requirements from: $req"
    pip install -r "$req"
done

##############################################
# INSTALL ROS DEPENDENCIES
##############################################

msg "Installing ROS dependencies via rosdep..."

rosdep install --from-paths src -i -y --rosdistro "$ROS_DISTRO"

##############################################
# BUILD WORKSPACE
##############################################

msg "Building the workspace with colcon..."
cd "$ROS_WORKSPACE"
colcon build --symlink-install

##############################################
# FINAL INSTRUCTIONS
##############################################

msg "Installation completed successfully! 🎉"

echo -e "\nTo activate the environment, run:"
echo -e "  source $ROS_WORKSPACE/$VENV_NAME/bin/activate"
echo -e "  source $ROS_WORKSPACE/install/setup.bash"

echo -e "\nTo auto-load the workspace on every terminal:"
echo "Add this to your ~/.bashrc:"
echo ""
echo "  source $ROS_WORKSPACE/$VENV_NAME/bin/activate"
echo "  source $ROS_WORKSPACE/install/setup.bash"
echo ""
msg "You are ready to start using the Cybership Software Suite!"