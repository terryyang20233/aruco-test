"""D435i + 屋脊板检测：每块标记一个 aruco_pose 节点，各自发布自己那一块的解。

    ros2 launch aruco_mocap camera.launch.py
    ros2 launch aruco_mocap camera.launch.py aruco_ir:=true          # 红外（全局快门）也跑一份检测
    ros2 launch aruco_mocap camera.launch.py color_profile:=1280x720x30
    ros2 launch aruco_mocap camera.launch.py markers_file:=$HOME/markers.yaml

realsense 只开彩色；aruco_ir:=true 时再开左红外相机（infra1）。深度和 IMU 都不开。
关红外投射器、曝光、电网频率这几项 rs_launch 不转发，写在 config/realsense.yaml 里。
标记编号和边长从 config/markers.yaml 读（或者 markers_file 指定的文件）。
彩色检测发布到 /marker_<编号>/aruco/...，红外检测发布到 /ir/marker_<编号>/aruco/...。
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
    markers_file = LaunchConfiguration("markers_file").perform(context) or str(
        share / "config" / "markers.yaml"
    )
    markers = yaml.safe_load(Path(markers_file).read_text())
    corner_refinement = LaunchConfiguration("corner_refinement").perform(context)
    nodes = []
    for marker_id in markers["ids"]:
        common = {
            "marker_id": int(marker_id),
            "marker_side_m": float(markers["side_m"]),
            "dictionary": markers["dictionary"],
            "corner_refinement": corner_refinement,
        }
        nodes.append(Node(
            package="aruco_mocap",
            executable="aruco_pose",
            name=f"aruco_pose_color_{marker_id}",
            namespace=f"marker_{marker_id}",
            parameters=[{**common, "image_topic": "/camera/camera/color/image_raw"}],
            output="screen",
        ))
        nodes.append(Node(
            package="aruco_mocap",
            executable="aruco_pose",
            name=f"aruco_pose_ir_{marker_id}",
            namespace=f"ir/marker_{marker_id}",
            parameters=[{**common, "image_topic": "/camera/camera/infra1/image_rect_raw"}],
            condition=IfCondition(LaunchConfiguration("aruco_ir")),
            output="screen",
        ))
    return nodes


def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("realsense2_camera")) / "launch" / "rs_launch.py")
        ),
        launch_arguments={
            "enable_color": "true",
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
            "enable_infra1": LaunchConfiguration("aruco_ir"),
            "enable_infra2": "false",
            "depth_module.infra_profile": LaunchConfiguration("infra_profile"),
            "enable_depth": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "config_file": str(Path(get_package_share_directory("aruco_mocap")) / "config" / "realsense.yaml"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("color_profile", default_value="640x480x30"),
        DeclareLaunchArgument("infra_profile", default_value="848x480x30"),
        DeclareLaunchArgument("aruco_ir", default_value="false"),
        DeclareLaunchArgument("corner_refinement", default_value="subpix"),
        DeclareLaunchArgument("markers_file", default_value=""),
        realsense,
        OpaqueFunction(function=aruco_nodes),
    ])
