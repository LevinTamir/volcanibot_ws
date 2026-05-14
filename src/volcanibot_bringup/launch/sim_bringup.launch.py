import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="feild",
        description="Gazebo world name (without .sdf extension)",
    )

    lidar_arg = DeclareLaunchArgument(
        "lidar",
        default_value="false",
        choices=["true", "false"],
        description="Spawn the Ouster Rev5 OS2 lidar in sim and bridge /scan.",
    )

    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="false",
        choices=["true", "false"],
        description="Spawn the RGBD camera in sim and bridge /camera/* topics.",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Open RViz with the volcanibot_description display config.",
    )

    volcanibot_description_share = get_package_share_directory("volcanibot_description")
    volcanibot_controller_share = get_package_share_directory("volcanibot_controller")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcanibot_description_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments=[
            ("world_name", LaunchConfiguration("world_name")),
            ("lidar", LaunchConfiguration("lidar")),
            ("camera", LaunchConfiguration("camera")),
        ],
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcanibot_controller_share, "launch", "controller.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(volcanibot_description_share, "rviz", "display.rviz"),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        lidar_arg,
        camera_arg,
        rviz_arg,
        gazebo_launch,
        controller_launch,
        rviz_node,
    ])
