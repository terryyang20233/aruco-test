"""mocap 这一半：VRPN 客户端（或假数据）+ 真值比较节点。

    ros2 launch aruco_mocap mocap.launch.py server:=192.168.1.100     # 实验室
    ros2 launch aruco_mocap mocap.launch.py fake:=true                # 家里，没有 mocap
    ros2 launch aruco_mocap mocap.launch.py server:=... compare_ir:=true

刚体名默认 cam_rig、marker_board，要和 Motive / Vicon 里建的名字一模一样。
vrpn_mocap 发布 /vrpn_mocap/<刚体名>/pose，时间戳是 Jetson 收到的时刻，和图像在同一个时钟上。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rig_topic = ["/vrpn_mocap/", LaunchConfiguration("rig_name"), "/pose"]
    board_topic = ["/vrpn_mocap/", LaunchConfiguration("board_name"), "/pose"]
    calibration_file = LaunchConfiguration("calibration_file")

    vrpn = Node(
        package="vrpn_mocap",
        executable="client_node",
        name="vrpn_mocap_client_node",
        namespace="vrpn_mocap",
        parameters=[{
            "server": LaunchConfiguration("server"),
            "port": PythonExpression(["int(", LaunchConfiguration("port"), ")"]),
            "frame_id": "world",
            "update_freq": 240.0,
            "refresh_freq": 1.0,
            "use_vrpn_timestamps": False,
        }],
        condition=UnlessCondition(LaunchConfiguration("fake")),
        output="screen",
    )
    fake = Node(
        package="aruco_mocap",
        executable="fake_mocap",
        parameters=[{"rig_topic": rig_topic, "board_topic": board_topic}],
        condition=IfCondition(LaunchConfiguration("fake")),
        output="screen",
    )
    compare_color = Node(
        package="aruco_mocap",
        executable="mocap_gt",
        name="mocap_gt_color",
        parameters=[{
            "rig_topic": rig_topic,
            "board_topic": board_topic,
            "aruco_pose_topic": "/aruco/pose",
            "camera": "color",
            "calibration_file": calibration_file,
        }],
        output="screen",
    )
    compare_ir = Node(
        package="aruco_mocap",
        executable="mocap_gt",
        name="mocap_gt_ir",
        namespace="ir",
        parameters=[{
            "rig_topic": rig_topic,
            "board_topic": board_topic,
            "aruco_pose_topic": "/ir/aruco/pose",
            "camera": "ir",
            "calibration_file": calibration_file,
            "broadcast_tf": False,
        }],
        condition=IfCondition(LaunchConfiguration("compare_ir")),
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("server", default_value="localhost"),
        DeclareLaunchArgument("port", default_value="3883"),
        DeclareLaunchArgument("fake", default_value="false"),
        DeclareLaunchArgument("rig_name", default_value="cam_rig"),
        DeclareLaunchArgument("board_name", default_value="marker_board"),
        DeclareLaunchArgument("compare_ir", default_value="false"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        vrpn,
        fake,
        compare_color,
        compare_ir,
    ])
