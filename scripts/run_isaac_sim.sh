#!/usr/bin/env bash
# Launch Isaac Sim with ROS_DOMAIN_ID and Fast DDS profile set for LIMB_isaac_sim.
# Usage: ./scripts/run_isaac_sim.sh
# (Do not source system ROS in the same terminal.)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FASTRTPS_PROFILE="$REPO_ROOT/docker_setup/fastdds_profile.xml"

if [ ! -f "$FASTRTPS_PROFILE" ]; then
  echo "Error: Fast DDS profile not found at $FASTRTPS_PROFILE" >&2
  exit 1
fi

export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_PROFILE"

# Optional: set if Isaac Sim is not in ~/isaacsim
if [ -n "$ISAAC_SIM_PATH" ]; then
  export isaac_sim_package_path="$ISAAC_SIM_PATH"
fi

ISAAC_SCRIPT="${isaac_sim_package_path:-$HOME/isaacsim}/isaac-sim.sh"
if [ ! -f "$ISAAC_SCRIPT" ]; then
  echo "Error: Isaac Sim not found at $ISAAC_SCRIPT. Set ISAAC_SIM_PATH or install to ~/isaacsim." >&2
  exit 1
fi

exec "$ISAAC_SCRIPT" "$@"
