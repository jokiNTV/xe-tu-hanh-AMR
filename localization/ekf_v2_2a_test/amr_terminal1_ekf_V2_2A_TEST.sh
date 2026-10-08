#!/usr/bin/env bash

WS="$HOME/amr_ws"
DIR="$WS/amr_new_route/ekf_v2_2a_test"

STM32_PORT="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_376B36513235-if00"
LIDAR_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
BRIDGE_PARAMS="$WS/src/amr_stm32_bridge/config/bridge.yaml"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
    echo "ERROR: robot_localization missing."
    exit 2
fi

PIDS=()

cleanup() {
    echo
    echo "Stopping Terminal 1 EKF V2.1..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

echo "================================================"
echo " TERMINAL 1 - AMR EKF V2.2A TEST"
echo "================================================"
echo "bridge  -> /odom_raw, TF OFF"
echo "adapter -> /odom_ekf_input, vy=0 constraint"
echo "IMU     -> /imu/data"
echo "EKF     -> /odom + odom->base_link TF"
echo

if ros2 node list 2>/dev/null | grep -qx "/amr_stm32_bridge"; then
    echo "ERROR: old bridge is already running. Stop old Terminal 1."
    exit 1
fi

ros2 run amr_stm32_bridge stm32_bridge --ros-args \
    --params-file "$BRIDGE_PARAMS" \
    -p serial_port:="$STM32_PORT" \
    -p cmd_vel_topic:=/cmd_vel_safe \
    -p clear_fault_on_connect:=true \
    -p include_drive_diagnostic:=true \
    -p odom_topic:=/odom_raw \
    -p publish_tf:=false &
PIDS+=($!)

ros2 run sllidar_ros2 sllidar_node --ros-args \
    -r __node:=rplidar_node \
    -p channel_type:=serial \
    -p serial_port:="$LIDAR_PORT" \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser \
    -p inverted:=false \
    -p angle_compensate:=true \
    -p scan_mode:=Standard \
    -p scan_frequency:=7.0 &
PIDS+=($!)

ros2 run tf2_ros static_transform_publisher \
    --x 0.045 --y 0.009 --z 0.550 \
    --yaw 3.141592654 --pitch 0.0 --roll 0.0 \
    --frame-id base_link --child-frame-id laser &
PIDS+=($!)

sleep 2

python3 "$DIR/amr_odom_adapter_V2.1.py" &
PIDS+=($!)

python3 "$DIR/amr_imu_adapter_V2.1.py" &
PIDS+=($!)

sleep 1

ros2 run robot_localization ekf_node --ros-args \
    --params-file "$DIR/ekf_amr_V2.2A_TEST.yaml" \
    -r /odometry/filtered:=/odom &
PIDS+=($!)

sleep 2
echo "EKF V2.2A TEST STARTED"
echo "Ctrl+C = stop all Terminal 1 processes."
wait "${PIDS[0]}"
