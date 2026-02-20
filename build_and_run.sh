#!/usr/bin/env sh

#
# variables setup
#
ROS_DISTRO=jazzy
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

#
# build
#
cd "$SCRIPT_DIR"
colcon build --symlink-install

#
# run
#
. "/opt/ros/${ROS_DISTRO}/setup.sh"
. "$SCRIPT_DIR/install/setup.sh"
ros2 launch robot_bringup robot.launch.py "$@"
