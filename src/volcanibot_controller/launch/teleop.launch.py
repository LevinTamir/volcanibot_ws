import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Command arbitration + (optional) joystick teleop.

    Always launches twist_mux, which muxes joystick (/joy_vel) and autonomy
    (/nav_vel) by priority and forwards the winner to the controller's
    cmd_vel topic. The joystick driver + joy_teleop are optional so that a
    headless sim or an autonomy-only run can disable them with joystick:=false.

    Included by both real_bringup and sim_bringup.
    """
    config_dir = os.path.join(
        get_package_share_directory("volcanibot_controller"), "config")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time.",
    )

    joystick_arg = DeclareLaunchArgument(
        "joystick",
        default_value="true",
        choices=["true", "false"],
        description="Launch the joystick driver + joy_teleop (publishes /joy_vel).",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # twist_mux output (cmd_vel_out) -> the topic DiffDriveController listens on.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[
            os.path.join(config_dir, "twist_mux.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/cmd_vel_out", "/volcanibot_controller/cmd_vel")],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(config_dir, "joy_config.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("joystick")),
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(config_dir, "joy_teleop.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("joystick")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        joystick_arg,
        twist_mux,
        joy_node,
        joy_teleop,
    ])
