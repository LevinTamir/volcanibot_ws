import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Trimble RTK GPS + navsat transform.

    Brings up the trimble_driver_ros GSOF client (the external package pulled in
    via volcanibot.repos) and robot_localization's navsat_transform_node, which
    turns the receiver's NavSatFix into an odom-frame pose.

    Off by default; enable from a bringup with gps:=true. Needs the Trimble
    receiver reachable at the ip/port in config/gsof_client_params.yaml.
    """
    config_dir = os.path.join(
        get_package_share_directory("volcanibot_bringup"), "config")

    gps_arg = DeclareLaunchArgument(
        "gps",
        default_value="false",
        choices=["true", "false"],
        description="Launch the Trimble GSOF client + navsat_transform.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gps_enabled = IfCondition(LaunchConfiguration("gps"))

    gsof_client = Node(
        package="trimble_driver_ros",
        executable="gsof_client_node",
        name="gsof_client",
        output="screen",
        parameters=[
            os.path.join(config_dir, "gsof_client_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        condition=gps_enabled,
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            os.path.join(config_dir, "navsat_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("gps/fix", "/gsof_client/navsat"),
        ],
        condition=gps_enabled,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        gps_arg,
        gsof_client,
        navsat_transform,
    ])
