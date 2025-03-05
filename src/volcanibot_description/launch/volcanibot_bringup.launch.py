import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    volcanibot_description = get_package_share_directory("volcanibot_description")

    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(volcanibot_description).parent.resolve())],
    )

    # Robot description
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    volcanibot_description, "urdf", "volcanibot_description.xacro"
                ),
            ]
        ),
        value_type=str,
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Spawn robot in Gazebo Ignition
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "pendulum",
            "-x",
            "1",
            "-y",
            "1",
            "-z",
            "0",
            "-r",
            "0",
            "-p",
            "0",
            "-Y",
            "3.14",
            "-topic",
            "/robot_description",
        ],
        output="screen",
    )

    # Launch Gazebo Ignition
    ignition_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments=[("gz_args", ["-r -v 4 empty.sdf"])],
    )

    return LaunchDescription(
        [
            gazebo_resource_path,
            robot_state_publisher_node,
            spawn_node,
            ignition_gazebo_node,
        ]
    )
