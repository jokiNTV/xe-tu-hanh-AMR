#!/usr/bin/env bash
# Do not use `set -u` before ROS setup.bash; Humble setup may reference
# AMENT_TRACE_SETUP_FILES before it is defined.
set -eo pipefail

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source "$HOME/amr_ws/install/setup.bash"

CFG="$HOME/Downloads/collision_monitor_FIX7.yaml"

echo "===== T4 COLLISION MONITOR FIX7 ====="
echo "config: $CFG"

exec ros2 launch nav2_collision_monitor collision_monitor_node.launch.py \
  use_sim_time:=False \
  params_file:="$CFG"
