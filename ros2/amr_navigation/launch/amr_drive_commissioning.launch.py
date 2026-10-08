"""AMR drive commissioning without map, AMCL or planner."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    nav_share = get_package_share_directory("amr_navigation")
    bridge_share = get_package_share_directory("amr_stm32_bridge")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share,
                "launch",
                "amr_robot_bringup.launch.py",
            )
        )
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            os.path.join(nav_share, "config", "nav2_params.yaml")
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel"),
        ],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            os.path.join(
                nav_share,
                "config",
                "collision_monitor_params.yaml",
            )
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_drive_commissioning",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [
                "velocity_smoother",
                "collision_monitor",
            ],
        }],
    )

    return LaunchDescription([
        base,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ])
