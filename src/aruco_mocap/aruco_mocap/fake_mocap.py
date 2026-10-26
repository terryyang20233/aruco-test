"""假的 mocap：没有动作捕捉时，在 vrpn_mocap 的同名话题上发两个刚体的位姿。

只用来在家里跑通 mocap_gt 节点、录包和分析脚本，数值没有意义。
默认把标记板放在相机支架正前方 1 m、正对相机，和 X、Y 为单位阵时的约定一致：
支架坐标就当作相机光心坐标（Z 向前），板绕 X 转 180°，让标记的 Z 轴朝向相机。
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FakeMocap(Node):
    def __init__(self):
        super().__init__("fake_mocap")
        self.declare_parameter("rig_topic", "/vrpn_mocap/cam_rig/pose")
        self.declare_parameter("board_topic", "/vrpn_mocap/marker_board/pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 120.0)
        self.declare_parameter("rig_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("rig_quat_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("board_xyz", [0.0, 0.0, 1.0])
        self.declare_parameter("board_quat_xyzw", [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter("wobble_m", 0.0)  # >0 时板左右慢慢晃，测插值

        self.frame_id = self.get_parameter("frame_id").value
        self.rig = (self.get_parameter("rig_xyz").value, self.get_parameter("rig_quat_xyzw").value)
        self.board = (self.get_parameter("board_xyz").value, self.get_parameter("board_quat_xyzw").value)
        self.wobble = float(self.get_parameter("wobble_m").value)
        self.rig_pub = self.create_publisher(PoseStamped, self.get_parameter("rig_topic").value,
                                             qos_profile_sensor_data)
        self.board_pub = self.create_publisher(PoseStamped, self.get_parameter("board_topic").value,
                                               qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.get_logger().warn("这是假的 mocap 数据，只用于测试流程")

    def make_pose(self, stamp, xyz, quat):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(v) for v in xyz]
        q = np.asarray(quat, dtype=float)
        q /= np.linalg.norm(q)
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = q.tolist()
        return msg

    def tick(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        board_xyz = list(self.board[0])
        if self.wobble > 0.0:
            board_xyz[0] += self.wobble * math.sin(2.0 * math.pi * 0.2 * now.nanoseconds * 1e-9)
        self.rig_pub.publish(self.make_pose(stamp, *self.rig))
        self.board_pub.publish(self.make_pose(stamp, board_xyz, self.board[1]))


def main():
    rclpy.init()
    node = FakeMocap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
