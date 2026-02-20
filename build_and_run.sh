#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

safe_source() {
  local file_path="$1"
  local had_nounset=0

  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  source "$file_path"

  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
}

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  safe_source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  safe_source "/opt/ros/jazzy/setup.bash"
else
  echo "Error: Could not find a ROS 2 setup file in /opt/ros." >&2
  echo "Set ROS_DISTRO (for example: export ROS_DISTRO=jazzy) and retry." >&2
  exit 1
fi

colcon build
safe_source "$SCRIPT_DIR/install/setup.bash"

ros2 launch robot_bringup robot.launch.py "$@"
