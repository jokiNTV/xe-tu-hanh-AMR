#!/usr/bin/env bash
# Capture one forward V3.1 drive test as a rosbag2 file.
# Usage: ./drive_v31_capture.sh [duration_seconds] [linear_speed_m_s]

set -euo pipefail

DURATION_S="${1:-5}"
SPEED_M_S="${2:-0.15}"

if ! [[ "$DURATION_S" =~ ^[1-9][0-9]*([.][0-9]+)?$ ]]; then
    echo "Duration must be a positive number of seconds." >&2
    exit 2
fi

if ! [[ "$SPEED_M_S" =~ ^0[.][0-9]+$ ]]; then
    echo "Speed must be a decimal value, for example 0.15." >&2
    exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
BAG_ROOT="$WORKSPACE_DIR/bags"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$BAG_ROOT/drive_v31_${STAMP}"
RECORDER_LOG="${BAG_DIR}.recorder.log"
RECORDER_PID=""

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

mkdir -p "$BAG_ROOT"

cleanup() {
    if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
        kill -INT "$RECORDER_PID" 2>/dev/null || true
        wait "$RECORDER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Recording V3.1 diagnostic data to: $BAG_DIR"
ros2 bag record -o "$BAG_DIR" \
    /cmd_vel_safe \
    /amr/drive/motion_profile \
    /amr/drive/target_wheel_rad_s \
    /amr/drive/measured_wheel_rad_s \
    /amr/drive/pwm \
    >"$RECORDER_LOG" 2>&1 &
RECORDER_PID="$!"

sleep 1

echo "Driving forward at ${SPEED_M_S} m/s for ${DURATION_S} s."
PUBLISH_STATUS=0
timeout --signal=INT "${DURATION_S}s" \
    ros2 topic pub --rate 20 /cmd_vel_safe geometry_msgs/msg/Twist \
    "{linear: {x: ${SPEED_M_S}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    || PUBLISH_STATUS="$?"

# GNU timeout returns 124 when its planned time limit is reached.
if [[ "$PUBLISH_STATUS" -ne 0 && "$PUBLISH_STATUS" -ne 124 ]]; then
    echo "Command publisher failed with status ${PUBLISH_STATUS}." >&2
    exit "$PUBLISH_STATUS"
fi

ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Keep recording the controlled deceleration after the command ends.
sleep 1
cleanup
RECORDER_PID=""
trap - EXIT INT TERM

mv "$RECORDER_LOG" "$BAG_DIR/recorder.log"
ARCHIVE_PATH="${BAG_DIR}.tar.gz"
tar -C "$BAG_ROOT" -czf "$ARCHIVE_PATH" "$(basename "$BAG_DIR")"

echo "Saved: $ARCHIVE_PATH"
echo "Send the drive_v31_*.tar.gz file for analysis."
