import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        "farm",
        default_value="farm.sdf",
        description="Name of the Gazebo world file to load",
    )

    pkg_volcanibot_description = get_package_share_directory("volcanibot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Add your own gazebo library path here
    gazebo_models_path = (
        "/home/tamir/ros2_workspaces/volcanibot_ws/src/volcanibot_description/models"
    )
    if "GZ_SIM_RESOURCE_PATH" not in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [pkg_volcanibot_description, "worlds", LaunchConfiguration("farm")]
                ),
                # TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
                TextSubstitution(text=" -r -v -v1"),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject
