import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare paths
    volcanibot_description = get_package_share_directory("volcanibot_description")

    gazebo_models_path, ignore_last_dir = os.path.split(volcanibot_description)
    if "GZ_SIM_RESOURCE_PATH" not in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_config_path = os.path.join(
        FindPackageShare("volcanibot_description").find("volcanibot_description"),
        "rviz",
        "urdf_config.rviz",
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

    # Decalre path to .worlds files
    world_arg = DeclareLaunchArgument(
        "farm",
        default_value="farm.sdf",
        description="Name of the Gazebo world file to load",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="volcanibot_description.xacro",
        description="Name of the URDF description to load",
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution(
        [
            volcanibot_description,  # Replace with your package name
            "urdf",
            LaunchConfiguration("model"),  # Replace with your URDF or Xacro file
        ]
    )

    # Include world.launch.py file
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcanibot_description, "launch", "world.launch.py"),
        ),
        launch_arguments={
            "farm": LaunchConfiguration("farm"),
        }.items(),
    )

    # Launch rviz
    rviz2_node = Node(
        package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path]
    )

    # Spawn robot in Gazebo Ignition
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "volcanibot",
            "-x",
            "2.0",
            "-y",
            "2.0",
            "-z",
            "0",
            "-r",
            "0",
            "-p",
            "0",
            "-Y",
            "1.57",
            "-topic",
            "/robot_description",
        ],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        ],
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # trajectory_node = Node(
    #     package='mogi_trajectory_server',
    #     executable='mogi_trajectory_server',
    #     name='mogi_trajectory_server',
    # )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz2_node)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(spawn_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    # launchDescriptionObject.add_action(trajectory_node)

    return launchDescriptionObject
