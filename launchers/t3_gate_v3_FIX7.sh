#!/usr/bin/env bash
set -e
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source "$HOME/amr_ws/install/setup.bash"
exec python3 "$HOME/amr_ws/amr_new_nav/mission_v1_clean/test/turn_safety_gate_v3_FIX7.py" \
  --active \
  --input-topic /cmd_vel \
  --output-topic /cmd_vel_guarded \
  --guard-angle-deg 25
