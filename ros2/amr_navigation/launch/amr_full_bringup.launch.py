"""Unified AMR runtime: base, localization/Nav2, and collision monitor."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    navigation_share = get_package_share_directory("amr_navigation")
    bridge_share = get_package_share_directory("amr_stm32_bridge")
    nav2_share = get_package_share_directory("nav2_bringup")
    collision_share = get_package_share_directory("nav2_collision_monitor")

    map_file = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params")
    collision_params = LaunchConfiguration("collision_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bridge_share, "launch", "amr_robot_bringup.launch.py"
            )
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                collision_share, "launch",
                "collision_monitor_node.launch.py"
            )
        ),
        launch_arguments={
            "params_file": collision_params,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.expanduser(
                "~/amr_ws/maps/home_pickup_clean_v1.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(
                navigation_share, "config", "nav2_params.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "collision_params",
            default_value=os.path.join(
                navigation_share, "config",
                "collision_monitor_params.yaml"
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        base,
        nav2,
        collision_monitor,
    ])
