import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Optional perception stack (YOLO detection).

    Runs the vendored/external yolo_ros detector against the unified camera
    topics (the same names the sim RGBD plugin and the real RealSense driver
    publish), so detection is identical in sim and on hardware. Publishes 3D
    detections on /yolo/detections_3d, which follow-person consumes.

    Included by both bringups and gated by the `yolo` arg (off by default,
    since inference is heavy).
    """
    yolo_arg = DeclareLaunchArgument(
        "yolo",
        default_value="false",
        choices=["true", "false"],
        description="Run YOLO detection on the camera stream (/yolo/detections_3d).",
    )

    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model",
        default_value="yolov8n.pt",
        description="YOLO weights. A bare name is resolved/downloaded by "
                    "ultralytics; pass an absolute path for offline/field use. "
                    "On the Jetson, use a TensorRT .engine rebuilt on that GPU.",
    )

    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="cuda:0",
        description="Inference device: cuda:0 on the Jetson, cpu on a "
                    "CPU-only dev machine.",
    )

    yolo_target_frame_arg = DeclareLaunchArgument(
        "yolo_target_frame",
        default_value="base_link",
        description="TF frame the 3D detections are transformed into.",
    )

    follow_arg = DeclareLaunchArgument(
        "follow",
        default_value="false",
        choices=["true", "false"],
        description="Run the person-following node (consumes /yolo/detections_3d, "
                    "outputs /nav_vel). Requires yolo:=true.",
    )

    follow_toggle_button_arg = DeclareLaunchArgument(
        "follow_toggle_button",
        default_value="4",
        description="Joystick button index that toggles person-following.",
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yolo_bringup"), "launch", "yolo.launch.py")
        ),
        launch_arguments={
            "model": LaunchConfiguration("yolo_model"),
            "device": LaunchConfiguration("yolo_device"),
            "use_3d": "True",
            "input_image_topic": "/camera/image",
            "input_depth_topic": "/camera/depth_image",
            "input_depth_info_topic": "/camera/camera_info",
            "target_frame": LaunchConfiguration("yolo_target_frame"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("yolo")),
    )

    follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("volcanibot_follow_person"),
                "launch", "follow_person.launch.py")
        ),
        launch_arguments={
            "toggle_button": LaunchConfiguration("follow_toggle_button"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("follow")),
    )

    return LaunchDescription([
        yolo_arg,
        yolo_model_arg,
        yolo_device_arg,
        yolo_target_frame_arg,
        follow_arg,
        follow_toggle_button_arg,
        yolo_launch,
        follow_launch,
    ])
