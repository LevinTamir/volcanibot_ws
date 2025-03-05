import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share_directory = FindPackageShare("volcanibot_description").find(
        "volcanibot_description"
    )
    urdf_path = os.path.join(
        package_share_directory, "urdf", "volcanibot_description.xacro"
    )
    meshes_path = os.path.join(package_share_directory, "meshes")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path, " meshes:=", meshes_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Spawn
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

    ignition_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments=[("gz_args", [" -r -v 4 empty.sdf"])],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_node)
    ld.add_action(ignition_gazebo_node)

    return ld
