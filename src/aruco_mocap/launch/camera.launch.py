"""D435i + ArUco 检测。

    ros2 launch aruco_mocap camera.launch.py
    ros2 launch aruco_mocap camera.launch.py aruco_ir:=true          # 红外也跑一份检测
    ros2 launch aruco_mocap camera.launch.py color_profile:=1280x720x30

realsense 的设置按 实验方案.md §3.5：彩色 + 红外 1 号 + IMU，关深度。
关红外投射器、电网频率这两项 rs_launch 不转发，写在 config/realsense.yaml 里。
标记编号和边长从 config/calibration.yaml 读，和 mocap_gt 用同一份。
彩色检测发布到 /aruco/...，红外检测发布到 /ir/aruco/...。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def aruco_nodes(context):
    share = Path(get_package_share_directory("aruco_mocap"))
    calibration_file = LaunchConfiguration("calibration_file").perform(context) or str(
        share / "config" / "calibration.yaml"
    )
    marker = yaml.safe_load(Path(calibration_file).read_text())["marker"]
    common = {
        "marker_id": int(marker["id"]),
        "marker_side_m": float(marker["side_m"]),
        "dictionary": marker["dictionary"],
        "corner_refinement": LaunchConfiguration("corner_refinement").perform(context),
    }
    return [
        Node(
            package="aruco_mocap",
            executable="aruco_pose",
            name="aruco_pose_color",
            parameters=[{**common, "image_topic": "/camera/camera/color/image_raw"}],
            output="screen",
        ),
        Node(
            package="aruco_mocap",
            executable="aruco_pose",
            name="aruco_pose_ir",
            namespace="ir",
            parameters=[{**common, "image_topic": "/camera/camera/infra1/image_rect_raw"}],
            condition=IfCondition(LaunchConfiguration("aruco_ir")),
            output="screen",
        ),
    ]


def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("realsense2_camera")) / "launch" / "rs_launch.py")
        ),
        launch_arguments={
            "enable_color": "true",
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
            "enable_infra1": LaunchConfiguration("enable_infra"),
            "enable_infra2": "false",
            "depth_module.infra_profile": LaunchConfiguration("infra_profile"),
            "enable_depth": "false",
            "enable_gyro": LaunchConfiguration("enable_imu"),
            "enable_accel": LaunchConfiguration("enable_imu"),
            "unite_imu_method": "2",
            "config_file": str(Path(get_package_share_directory("aruco_mocap")) / "config" / "realsense.yaml"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("color_profile", default_value="640x480x30"),
        DeclareLaunchArgument("infra_profile", default_value="848x480x30"),
        DeclareLaunchArgument("enable_infra", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("aruco_ir", default_value="false"),
        DeclareLaunchArgument("corner_refinement", default_value="subpix"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        realsense,
        OpaqueFunction(function=aruco_nodes),
    ])
