#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/amr_ws/install/setup.bash"
python3 "$HOME/amr_ws/amr_new_route/scan_timing_v1/amr_scan_midstamp_V1.0.py"
