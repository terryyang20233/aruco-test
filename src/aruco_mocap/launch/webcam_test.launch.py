"""没有 librealsense 时的流程测试：把一路普通 V4L2 摄像头当成 D435i 的红外 1 号发布出来，再跑 ArUco 检测。

    ros2 launch aruco_mocap webcam_test.launch.py video_device:=/dev/video2

用途：VMware 等虚拟机里 USB 直通不可用时，D435i 被当成普通摄像头共享进来（只有一路左红外，
投射器开着、关不掉）。话题名和 realsense 节点一样，所以 mocap.launch.py、record.sh 不用改。
内参来自 config/approx_ir_640x480.yaml，是典型值，不是出厂标定：距离只能当近似看。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("aruco_mocap"))
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="webcam_as_infra1",
        namespace="camera/camera/infra1",
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "image_size": [640, 480],
            "camera_frame_id": "camera_infra1_optical_frame",
            "camera_info_url": "file://" + str(share / "config" / "approx_ir_640x480.yaml"),
        }],
        remappings=[("image_raw", "image_rect_raw")],
        output="screen",
    )
    detector = Node(
        package="aruco_mocap",
        executable="aruco_pose",
        name="aruco_pose_ir",
        namespace="ir",
        parameters=[{
            "image_topic": "/camera/camera/infra1/image_rect_raw",
            "marker_side_m": LaunchConfiguration("marker_side_m"),
        }],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("video_device", default_value="/dev/video2"),
        DeclareLaunchArgument("marker_side_m", default_value="0.15"),
        camera,
        detector,
    ])
