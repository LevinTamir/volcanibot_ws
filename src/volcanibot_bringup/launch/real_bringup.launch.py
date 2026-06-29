import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (False for real hardware)",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/roboteq",
        description="Serial device for the Roboteq motor controller "
                    "(stable symlink created by the udev rule in "
                    "volcanibot_hardware_interface/udev/).",
    )

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Roboteq UART baud rate.",
    )

    lidar_arg = DeclareLaunchArgument(
        "lidar",
        default_value="false",
        choices=["true", "false"],
        description="Include the Ouster Rev5 OS2 lidar in the URDF and launch ouster_ros.",
    )

    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="false",
        choices=["true", "false"],
        description="Include the RGBD camera in the URDF and launch the "
                    "RealSense driver, remapped onto the sim camera topics "
                    "(/camera/image, /camera/depth_image, /camera/camera_info, "
                    "/camera/points).",
    )

    sensor_hostname_arg = DeclareLaunchArgument(
        "sensor_hostname",
        default_value="os-122000000000.local",
        description="Ouster lidar hostname or IP, forwarded to ouster_ros.",
    )

    ouster_params_file_arg = DeclareLaunchArgument(
        "ouster_params_file",
        default_value="",
        description="Optional path to a custom ouster_ros driver_params.yaml. "
                    "When empty, the driver's packaged default is used.",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Open RViz with the volcanibot_description display config.",
    )

    joystick_arg = DeclareLaunchArgument(
        "joystick",
        default_value="true",
        choices=["true", "false"],
        description="Launch the joystick driver + joy_teleop (publishes /joy_vel).",
    )

    yolo_arg = DeclareLaunchArgument(
        "yolo",
        default_value="false",
        choices=["true", "false"],
        description="Run YOLO detection on the camera stream (needs camera:=true).",
    )

    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="cuda:0",
        description="YOLO inference device (cuda:0 on the Jetson, cpu otherwise).",
    )

    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model",
        default_value="yolov8n.pt",
        description="YOLO weights; absolute path or a rebuilt .engine for field use.",
    )

    volcanibot_description_share = get_package_share_directory("volcanibot_description")
    volcanibot_controller_share = get_package_share_directory("volcanibot_controller")
    volcanibot_bringup_share = get_package_share_directory("volcanibot_bringup")

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(volcanibot_description_share, "urdf", "volcanibot.xacro"),
            " use_sim:=false",
            " lidar:=", LaunchConfiguration("lidar"),
            " camera:=", LaunchConfiguration("camera"),
            " serial_port:=", LaunchConfiguration("serial_port"),
            " baud_rate:=", LaunchConfiguration("baud_rate"),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
    )

    # On real hardware the Gazebo plugin loader doesn't run, so we launch
    # ros2_control_node ourselves. The controllers YAML is the same one
    # sim uses; use_sim_time is overridden here for real-hw operation.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            os.path.join(
                volcanibot_controller_share, "config", "volcanibot_controllers_real.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    volcanibot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcanibot_controller",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Intel RealSense depth camera. Topics are remapped onto the same names
    # the Gazebo RGBD plugin uses, so weed_3d_detector / YOLO are agnostic to
    # whether they run against sim or real hardware. Depth is aligned to the
    # color frame so 3D detections share the RGB optical frame.
    realsense_camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        output="screen",
        parameters=[{
            "enable_color": True,
            "enable_depth": True,
            "align_depth.enable": True,
            "pointcloud.enable": True,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=[
            ("color/image_raw", "/camera/image"),
            ("color/camera_info", "/camera/camera_info"),
            ("aligned_depth_to_color/image_raw", "/camera/depth_image"),
            ("depth/color/points", "/camera/points"),
        ],
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    # Command arbitration (twist_mux) + optional joystick. Muxes /joy_vel and
    # /nav_vel onto volcanibot_controller/cmd_vel by priority.
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcanibot_controller_share, "launch", "teleop.launch.py")
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("joystick", LaunchConfiguration("joystick")),
        ],
    )

    # Optional YOLO perception on the camera stream (/yolo/detections_3d).
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcanibot_bringup_share, "launch", "perception.launch.py")
        ),
        launch_arguments=[
            ("yolo", LaunchConfiguration("yolo")),
            ("yolo_device", LaunchConfiguration("yolo_device")),
            ("yolo_model", LaunchConfiguration("yolo_model")),
        ],
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

    ouster_share = get_package_share_directory("ouster_ros") if _has_pkg("ouster_ros") else ""
    ouster_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ouster_share, "launch", "driver.launch.py")
            if ouster_share else "/nonexistent"
        ),
        launch_arguments=[
            ("sensor_hostname", LaunchConfiguration("sensor_hostname")),
            ("params_file", LaunchConfiguration("ouster_params_file")),
        ],
        condition=IfCondition(LaunchConfiguration("lidar")),
    ) if ouster_share else None

    actions = [
        use_sim_time_arg,
        serial_port_arg,
        baud_rate_arg,
        lidar_arg,
        camera_arg,
        sensor_hostname_arg,
        ouster_params_file_arg,
        rviz_arg,
        joystick_arg,
        yolo_arg,
        yolo_device_arg,
        yolo_model_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        volcanibot_controller_spawner,
        realsense_camera,
        teleop_launch,
        perception_launch,
        rviz_node,
    ]
    if ouster_driver is not None:
        actions.append(ouster_driver)

    return LaunchDescription(actions)


def _has_pkg(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False
