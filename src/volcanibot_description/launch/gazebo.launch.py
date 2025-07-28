import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import launch_ros


def generate_launch_description():
    volcanibot_description = get_package_share_directory("volcanibot_description")

    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(volcanibot_description, "urdf", "volcanibot.xacro"),
        description="Absolute path to robot urdf file",
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution(
        [
            volcanibot_description,
            "worlds",
            PythonExpression(
                expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"]
            ),
        ]
    )

    model_path = str(Path(volcanibot_description).parent.resolve())
    model_path += pathsep + os.path.join(
        get_package_share_directory("volcanibot_description"), "models"
    )

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
    #             "/gz_sim.launch.py",
    #         ]
    #     ),
    #     launch_arguments={
    #         "gz_args": PythonExpression(["' ", world_path, " -v 4 -r'"])
    #     }.items(),
    # )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            unitree_go2_description,
            'worlds',
            'default.sdf'
        ])}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
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
            "3.14",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            # "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            
            # '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            
            '/realsense_d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/realsense_d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/realsense_d435/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/realsense_d435/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            world_name_arg,
            robot_state_publisher_node,
            gazebo_resource_path,
            # gazebo,
            gz_sim,
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
