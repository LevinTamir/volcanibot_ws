from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "toggle_button",
            default_value="4",
            description="Joystick button index to toggle tracking (4 = LB/L1 on Logitech Dual Action)",
        ),
        DeclareLaunchArgument(
            "lost_timeout",
            default_value="2.0",
            description="Seconds without detection before resuming search for the person",
        ),
        Node(
            package="volcanibot_follow_person",
            executable="person_target.py",
            name="person_target_node",
            output="screen",
            parameters=[{
                "toggle_button": LaunchConfiguration("toggle_button"),
                "lost_timeout": LaunchConfiguration("lost_timeout"),
            }],
        ),
    ])
