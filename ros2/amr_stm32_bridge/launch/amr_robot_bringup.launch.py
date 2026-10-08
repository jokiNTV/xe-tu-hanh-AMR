"""Bring up STM32 drive, RPLIDAR A1 and the calibrated laser TF."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


STM32_BY_ID = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_376F36433235-if00"
)
RPLIDAR_BY_ID = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def generate_launch_description() -> LaunchDescription:
    bridge_share = get_package_share_directory("amr_stm32_bridge")

    stm32_port = LaunchConfiguration("stm32_port")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")
    clear_fault_on_connect = LaunchConfiguration("clear_fault_on_connect")
    include_drive_diagnostic = LaunchConfiguration(
        "include_drive_diagnostic"
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share,
                "launch",
                "amr_stm32_bridge.launch.py",
            )
        ),
        launch_arguments={
            "serial_port": stm32_port,
            "clear_fault_on_connect": clear_fault_on_connect,
            "include_drive_diagnostic": include_drive_diagnostic,
        }.items(),
    )

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": ParameterValue(
                lidar_port,
                value_type=str,
            ),
            "serial_baudrate": 115200,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": ParameterValue(
                lidar_scan_mode,
                value_type=str,
            ),
            "scan_frequency": 7.0,
        }],
    )

    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        output="screen",
        arguments=[
            "--x", "-0.015",
            "--y", "-0.030",
            "--z", "0.532",
            "--yaw", "3.141592654",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "stm32_port",
            default_value=STM32_BY_ID,
            description="Stable STM32 USB CDC path",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value=RPLIDAR_BY_ID,
            description="Stable RPLIDAR A1 CP2102 path",
        ),
        DeclareLaunchArgument(
            "lidar_scan_mode",
            default_value="Standard",
            description="RPLIDAR A1 scan mode verified on the robot",
        ),
        DeclareLaunchArgument(
            "clear_fault_on_connect",
            default_value="false",
            description="Intentionally clear a latched drive fault at startup",
        ),
        DeclareLaunchArgument(
            "include_drive_diagnostic",
            default_value="false",
            description="Enable STM32 drive diagnostic stream",
        ),
        bridge_launch,
        lidar_node,
        laser_tf,
    ])
