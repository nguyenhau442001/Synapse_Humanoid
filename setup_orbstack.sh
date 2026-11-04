#!/usr/bin/env bash
# ==============================================================================
# setup_orbstack.sh — Automated OrbStack + Ubuntu 24.04 Environment Setup
#
# Sets up a lightweight, near-native Ubuntu 24.04 (Noble) Linux instance with
# ROS 2 Jazzy, ros2_control, and AI Perception tools on macOS (Apple Silicon).
# Replaces heavy VMs like UTM with instant startup, zero battery drain, and
# full bidirectional filesystem integration.
#
# Usage:
#   chmod +x setup_orbstack.sh
#   ./setup_orbstack.sh
# ==============================================================================

set -euo pipefail

# ── Colors & Logging ─────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${BLUE}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
step()    { echo -e "\n${BOLD}━━━ $* ━━━${RESET}"; }

ORB_MACHINE_NAME="synapse-humanoid"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 1. Check macOS Host ──────────────────────────────────────────────────────
step "1. Validating macOS Host System"
if [[ "$(uname -s)" != "Darwin" ]]; then
  error "This setup script is designed for macOS. Current OS: $(uname -s)"
fi

ARCH="$(uname -m)"
info "Detected architecture: $ARCH (macOS $(sw_vers -productVersion))"

# ── 2. Check / Install OrbStack ──────────────────────────────────────────────
step "2. Checking OrbStack Installation"
if ! command -v orb &> /dev/null; then
  warn "OrbStack CLI ('orb') not found in PATH."
  if command -v brew &> /dev/null; then
    info "Installing OrbStack via Homebrew..."
    brew install orbstack
  else
    error "Homebrew is required to install OrbStack. Please install brew (https://brew.sh) or OrbStack directly (https://orbstack.dev)."
  fi
fi

success "OrbStack is installed: $(orb version 2>/dev/null || echo 'ready')"

# ── 3. Ensure OrbStack Daemon is Running ──────────────────────────────────────
step "3. Verifying OrbStack Daemon Status"
if ! orb status &> /dev/null; then
  info "Starting OrbStack application..."
  open -a OrbStack || true
  info "Waiting for OrbStack daemon to become responsive..."
  for i in {1..30}; do
    if orb status &> /dev/null; then
      break
    fi
    sleep 1
  done
fi

if ! orb status &> /dev/null; then
  error "OrbStack failed to start. Please open OrbStack manually from Applications."
fi
success "OrbStack daemon is active."

# ── 4. Create Ubuntu 24.04 (Noble) Linux Machine ─────────────────────────────
step "4. Setting up Ubuntu 24.04 Linux Instance ('$ORB_MACHINE_NAME')"
EXISTING_MACHINES="$(orb list 2>/dev/null || true)"
if echo "$EXISTING_MACHINES" | grep -q "$ORB_MACHINE_NAME"; then
  info "Machine '$ORB_MACHINE_NAME' already exists."
else
  info "Creating fresh Ubuntu 24.04 LTS (Noble) instance: '$ORB_MACHINE_NAME'..."
  orb create ubuntu:noble "$ORB_MACHINE_NAME"
fi

success "Linux machine '$ORB_MACHINE_NAME' is ready."

# ── 5. Provision ROS 2 Jazzy Inside the Linux Machine ────────────────────────
step "5. Provisioning ROS 2 Jazzy & Build Dependencies Inside '$ORB_MACHINE_NAME'"
MAC_REPO_PATH="/mac$SCRIPT_DIR"

info "Executing setup_ubuntu.sh inside the container..."
orb -m "$ORB_MACHINE_NAME" bash -c "
  if [ -f '$MAC_REPO_PATH/setup_ubuntu.sh' ]; then
    bash '$MAC_REPO_PATH/setup_ubuntu.sh'
  else
    echo 'Error: Could not locate $MAC_REPO_PATH/setup_ubuntu.sh'
    exit 1
  fi
"

success "Provisioning completed inside '$ORB_MACHINE_NAME'."

# ── 6. Test Build Synapse Humanoid ───────────────────────────────────────────
step "6. Performing Verification Build via OrbStack"
info "Building Synapse Humanoid packages inside Linux..."
orb -m "$ORB_MACHINE_NAME" bash -c "
  source /opt/ros/jazzy/setup.bash
  cd '$MAC_REPO_PATH'
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
"

# ── 7. Summary & Quick Usage ─────────────────────────────────────────────────
step "🎉 Setup Complete!"
echo -e "${GREEN}OrbStack Linux development environment is fully operational.${RESET}"
echo ""
echo -e "${BOLD}Quick Access Commands:${RESET}"
echo -e "  1. ${BLUE}Enter Linux Terminal:${RESET}"
echo -e "     orb -m $ORB_MACHINE_NAME"
echo ""
echo -e "  2. ${BLUE}Build directly from your macOS terminal:${RESET}"
echo -e "     orb -m $ORB_MACHINE_NAME bash -c \"source /opt/ros/jazzy/setup.bash && cd /mac\$PWD && colcon build --symlink-install\""
echo ""
echo -e "  3. ${BLUE}Add convenient alias to ~/.zshrc (Optional):${RESET}"
echo -e "     alias orb-build='orb -m $ORB_MACHINE_NAME bash -c \"source /opt/ros/jazzy/setup.bash && cd /mac\\\$PWD && colcon build --symlink-install\"'"
echo ""
