#!/usr/bin/env bash

export ROS_DOMAIN_ID=42

WS="$HOME/amr_ws"
MID="$WS/amr_new_route/scan_timing_v1/run_scan_midstamp_V1.sh"
MAP="$WS/maps/mapVinh1_FIX12.yaml"
PARAMS="$HOME/Downloads/mppi_035_FIX7.yaml"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

MID_PID=""
NAV2_PID=""

cleanup() {
    [ -n "$NAV2_PID" ] && kill "$NAV2_PID" 2>/dev/null || true
    [ -n "$MID_PID" ] && kill "$MID_PID" 2>/dev/null || true
    [ -n "$NAV2_PID" ] && wait "$NAV2_PID" 2>/dev/null || true
    [ -n "$MID_PID" ] && wait "$MID_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

MODE="${1:-}"

echo "===== T2 MPPI 0.35 DEMO FAST V3 FIX7: MID-STAMP + NAV2 MPPI | XY TOL=9CM ====="

# Prevent duplicate/rejected scan helpers.
if ros2 node list 2>/dev/null | grep -Eq '^/amr_scan_gate_v1_(0|1)$|^/amr_scan_midstamp_v1_0$'; then
    echo "ERROR: another scan helper is already running."
    echo "Stop old Terminal 2 / standalone scan helper first."
    exit 1
fi

"$MID" &
MID_PID=$!

sleep 1
if ! kill -0 "$MID_PID" 2>/dev/null; then
    echo "ERROR: MID-STAMP V1.0 failed to start."
    exit 1
fi
echo "MID-STAMP V1.0: STARTED"

ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP" \
  params_file:="$PARAMS" \
  use_sim_time:=False \
  autostart:=True \
  use_composition:=False \
  use_respawn:=False &
NAV2_PID=$!

echo "NAV2: STARTED"

# Wait for AMCL to become active. Do not manipulate lifecycle state here.
AMCL_ACTIVE=0
for i in $(seq 1 30); do
    if ros2 lifecycle get /amcl 2>/dev/null | grep -q "active"; then
        AMCL_ACTIVE=1
        echo "AMCL: ACTIVE"
        break
    fi
    sleep 0.5
done

if [ "$AMCL_ACTIVE" -ne 1 ]; then
    echo "WARNING: AMCL did not report ACTIVE within startup window."
fi

if [ "$MODE" = "home" ]; then
    echo "HOME MODE: waiting for /amr/reset_to_A_H ..."
    RESET_DONE=0
    for i in $(seq 1 20); do
        if ros2 service list 2>/dev/null | grep -qx "/amr/reset_to_A_H"; then
            echo "HOME MODE: resetting localization to physical A_H"
            ros2 service call /amr/reset_to_A_H std_srvs/srv/Trigger "{}"
            RESET_DONE=1
            break
        fi
        sleep 0.5
    done

    if [ "$RESET_DONE" -ne 1 ]; then
        echo "WARNING: /amr/reset_to_A_H was not available."
    fi

    echo "NOTE: 'home' is valid ONLY when robot is physically at A_H."
else
    echo "LOCALIZATION: set the actual initial pose after AMCL restart."
fi

wait "$NAV2_PID"
