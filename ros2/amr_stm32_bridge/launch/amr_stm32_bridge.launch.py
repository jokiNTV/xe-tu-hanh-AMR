"""Launch the Humble STM32 drive and odometry bridge."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("amr_stm32_bridge")
    config_file = os.path.join(package_share, "config", "bridge.yaml")

    serial_port = LaunchConfiguration("serial_port")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    clear_fault_on_connect = LaunchConfiguration("clear_fault_on_connect")
    stream_period_ms = LaunchConfiguration("stream_period_ms")
    include_drive_diagnostic = LaunchConfiguration(
        "include_drive_diagnostic"
    )
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="STM32 USB CDC device, preferably /dev/serial/by-id",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel_safe",
            description="Velocity command topic",
        ),
        DeclareLaunchArgument(
            "clear_fault_on_connect",
            default_value="false",
            description="Clear a latched STM32 fault during startup handshake",
        ),
        DeclareLaunchArgument(
            "stream_period_ms",
            default_value="50",
            description="STM32 stream period in milliseconds",
        ),
        DeclareLaunchArgument(
            "include_drive_diagnostic",
            default_value="false",
            description=(
                "Commissioning only: diagnostic replaces normal odometry"
            ),
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish odom to base_link TF",
        ),
        Node(
            package="amr_stm32_bridge",
            executable="stm32_bridge",
            name="amr_stm32_bridge",
            output="screen",
            parameters=[
                config_file,
                {
                    "serial_port": serial_port,
                    "cmd_vel_topic": cmd_vel_topic,
                    "clear_fault_on_connect": ParameterValue(
                        clear_fault_on_connect,
                        value_type=bool,
                    ),
                    "stream_period_ms": ParameterValue(
                        stream_period_ms,
                        value_type=int,
                    ),
                    "include_drive_diagnostic": ParameterValue(
                        include_drive_diagnostic,
                        value_type=bool,
                    ),
                    "publish_tf": ParameterValue(
                        publish_tf,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
