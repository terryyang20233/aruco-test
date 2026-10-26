"""ROS 2 节点：用 mocap 的两个刚体算出“标记在相机里”的真值，和 ArUco 的结果现场比较。

记号见 实验方案.md §4.1：
    T_C_M(真值) = X⁻¹ · T_W_R⁻¹ · T_W_B · Y
R 是相机支架刚体，B 是标记板刚体，X = T_R_C 和 Y = T_B_M 来自标定文件。
标定之前 X、Y 是单位阵，真值只在“距离”上大致可信（见 mocap/error 里的 body_distance_m）。

订阅：
- rig_topic、board_topic   vrpn_mocap 发布的 PoseStamped
- aruco_pose_topic         aruco_pose 节点的 PoseStamped
发布：
- mocap/gt_pose   PoseStamped，坐标系和时间戳都跟随那一帧 ArUco 位姿
- mocap/error     String（JSON）：位置误差（相机坐标、沿视线、横向）、姿态误差（度）
- TF：world → 两个刚体；刚体 → 标定出的相机光心、标记中心（RViz 里看用）

现场主要靠录包、回来离线分析。这个节点是给现场一个“数对不对”的快速判断。
"""

import bisect
import json
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from aruco_mocap.transforms import (
    from_translation_quat,
    invert,
    matrix_to_quat,
    rotation_angle_deg,
    slerp,
)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_arrays(pose):
    p, q = pose.position, pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def load_transform(entry):
    return from_translation_quat(entry["translation"], entry["quaternion_xyzw"])


class PoseBuffer:
    """一个刚体最近几秒的位姿，按时间插值。"""

    def __init__(self, horizon_s):
        self.horizon = horizon_s
        self.times = deque()
        self.positions = deque()
        self.quats = deque()
        self.received = 0
        self.stale = 0  # 和上一帧完全相同：mocap 可能丢了跟踪、在重复发旧值

    def add(self, t, position, quat):
        self.received += 1
        if self.times and np.array_equal(position, self.positions[-1]) and np.array_equal(
            quat, self.quats[-1]
        ):
            self.stale += 1
        if self.times and t <= self.times[-1]:
            return
        self.times.append(t)
        self.positions.append(position)
        self.quats.append(quat)
        while self.times and self.times[-1] - self.times[0] > self.horizon:
            self.times.popleft()
            self.positions.popleft()
            self.quats.popleft()

    def latest_time(self):
        return self.times[-1] if self.times else None

    def at(self, t, max_gap):
        """t 时刻的 4×4 位姿；t 不在缓冲区里、或两侧样本间隔太大时返回 None。"""
        times = list(self.times)
        index = bisect.bisect_left(times, t)
        if index < len(times) and times[index] == t:
            return from_translation_quat(self.positions[index], self.quats[index])
        if index == 0 or index == len(times):
            return None
        t0, t1 = times[index - 1], times[index]
        if t1 - t0 > max_gap:
            return None
        fraction = (t - t0) / (t1 - t0)
        position = (1 - fraction) * self.positions[index - 1] + fraction * self.positions[index]
        quat = slerp(self.quats[index - 1], self.quats[index], fraction)
        return from_translation_quat(position, quat)


class MocapGroundTruthNode(Node):
    def __init__(self):
        super().__init__("mocap_gt")
        self.declare_parameter("rig_topic", "/vrpn_mocap/cam_rig/pose")
        self.declare_parameter("board_topic", "/vrpn_mocap/marker_board/pose")
        self.declare_parameter("aruco_pose_topic", "/aruco/pose")
        self.declare_parameter("calibration_file", "")  # 空 = 包里的 config/calibration.yaml
        self.declare_parameter("camera", "color")  # color 或 ir，决定用哪一个 X
        self.declare_parameter("time_offset_s", float("nan"))  # nan = 用标定文件里的值
        self.declare_parameter("buffer_s", 3.0)
        self.declare_parameter("max_gap_s", 0.05)
        self.declare_parameter("max_wait_s", 0.5)
        self.declare_parameter("broadcast_tf", True)
        self.declare_parameter("stats_period_s", 5.0)

        path = self.get_parameter("calibration_file").value or str(
            Path(get_package_share_directory("aruco_mocap")) / "config" / "calibration.yaml"
        )
        calibration = yaml.safe_load(Path(path).read_text())
        camera = self.get_parameter("camera").value
        self.X = load_transform(calibration[f"X_rig_{camera}"])
        self.Y = load_transform(calibration["Y_board_marker"])
        self.X_inv = invert(self.X)
        offset = self.get_parameter("time_offset_s").value
        self.time_offset = float(calibration.get("time_offset_s", 0.0)) if np.isnan(offset) else offset
        self.max_gap = float(self.get_parameter("max_gap_s").value)
        self.max_wait = float(self.get_parameter("max_wait_s").value)
        if np.allclose(self.X, np.eye(4)) or np.allclose(self.Y, np.eye(4)):
            self.get_logger().warn("X 或 Y 还是单位阵（没做手眼标定）。真值只有距离大致可信。")

        horizon = float(self.get_parameter("buffer_s").value)
        self.rig = PoseBuffer(horizon)
        self.board = PoseBuffer(horizon)
        self.pending = deque()
        self.world_frame = "world"
        self.camera = camera

        rig_topic = self.get_parameter("rig_topic").value
        board_topic = self.get_parameter("board_topic").value
        self.create_subscription(PoseStamped, rig_topic, lambda m: self.on_mocap(m, self.rig, "cam_rig"),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, board_topic,
                                 lambda m: self.on_mocap(m, self.board, "marker_board"),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.get_parameter("aruco_pose_topic").value,
                                 self.pending.append, qos_profile_sensor_data)
        self.gt_pub = self.create_publisher(PoseStamped, "mocap/gt_pose", 10)
        self.error_pub = self.create_publisher(String, "mocap/error", 10)

        self.tf = None
        if self.get_parameter("broadcast_tf").value:
            self.tf = TransformBroadcaster(self)
            self.static_tf = StaticTransformBroadcaster(self)
            self.static_tf.sendTransform([
                self.transform_msg("cam_rig", f"mocap_{camera}_optical", self.X),
                self.transform_msg("marker_board", "mocap_marker", self.Y),
            ])

        self.matched = 0
        self.unmatched = 0
        self.errors = []
        self.create_timer(0.02, self.process_pending)
        self.create_timer(float(self.get_parameter("stats_period_s").value), self.report)
        self.get_logger().info(
            f"刚体 {rig_topic} / {board_topic}，标定 {path}，相机 {camera}，时间偏移 {self.time_offset} s"
        )

    def transform_msg(self, parent, child, transform, stamp=None):
        msg = TransformStamped()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        if stamp is not None:
            msg.header.stamp = stamp
        t = msg.transform.translation
        t.x, t.y, t.z = transform[:3, 3].tolist()
        r = msg.transform.rotation
        r.x, r.y, r.z, r.w = matrix_to_quat(transform[:3, :3]).tolist()
        return msg

    def on_mocap(self, msg, buffer, child_frame):
        position, quat = pose_arrays(msg.pose)
        buffer.add(stamp_seconds(msg.header.stamp), position, quat)
        if msg.header.frame_id:
            self.world_frame = msg.header.frame_id
        if self.tf is not None:
            transform = from_translation_quat(position, quat)
            message = self.transform_msg(self.world_frame, child_frame, transform, msg.header.stamp)
            self.tf.sendTransform(message)

    def process_pending(self):
        newest = [b.latest_time() for b in (self.rig, self.board)]
        while self.pending:
            detection = self.pending[0]
            t = stamp_seconds(detection.header.stamp) + self.time_offset
            if None not in newest and t > min(newest):
                # mocap 还没追上这一帧。等一会儿，等太久就放弃。
                if min(newest) - t < -self.max_wait:
                    self.pending.popleft()
                    self.unmatched += 1
                    continue
                break
            self.pending.popleft()
            self.evaluate(detection, t)

    def evaluate(self, detection, t):
        T_W_R = self.rig.at(t, self.max_gap)
        T_W_B = self.board.at(t, self.max_gap)
        if T_W_R is None or T_W_B is None:
            self.unmatched += 1
            return
        self.matched += 1
        T_C_M_gt = self.X_inv @ invert(T_W_R) @ T_W_B @ self.Y
        position, quat = pose_arrays(detection.pose)
        T_C_M_est = from_translation_quat(position, quat)

        gt_pose = PoseStamped()
        gt_pose.header = detection.header
        p = gt_pose.pose.position
        p.x, p.y, p.z = T_C_M_gt[:3, 3].tolist()
        o = gt_pose.pose.orientation
        o.x, o.y, o.z, o.w = matrix_to_quat(T_C_M_gt[:3, :3]).tolist()
        self.gt_pub.publish(gt_pose)

        error = T_C_M_est[:3, 3] - T_C_M_gt[:3, 3]
        line_of_sight = T_C_M_gt[:3, 3] / max(np.linalg.norm(T_C_M_gt[:3, 3]), 1e-9)
        along = float(error @ line_of_sight)
        lateral = float(np.sqrt(max(error @ error - along * along, 0.0)))
        rotation_error = rotation_angle_deg(T_C_M_gt[:3, :3].T @ T_C_M_est[:3, :3])
        # 标定之前用来粗查：两个刚体原点的距离 vs ArUco 解出的距离
        body_distance = float(np.linalg.norm(T_W_B[:3, 3] - T_W_R[:3, 3]))
        record = {
            "stamp": stamp_seconds(detection.header.stamp),
            "camera": self.camera,
            "gt_tvec": T_C_M_gt[:3, 3].tolist(),
            "position_error": error.tolist(),
            "position_error_norm": float(np.linalg.norm(error)),
            "along_sight_error": along,
            "lateral_error": lateral,
            "rotation_error_deg": rotation_error,
            "aruco_distance_m": float(np.linalg.norm(position)),
            "body_distance_m": body_distance,
        }
        self.errors.append((record["position_error_norm"], rotation_error,
                            record["aruco_distance_m"], body_distance))
        self.error_pub.publish(String(data=json.dumps(record)))

    def report(self):
        line = (
            f"mocap 收到 cam_rig {self.rig.received}（重复 {self.rig.stale}），"
            f"marker_board {self.board.received}（重复 {self.board.stale}）；"
            f"ArUco 配上 {self.matched}，没配上 {self.unmatched}"
        )
        if self.errors:
            e = np.array(self.errors)
            line += (
                f"；位置误差 RMS {1000 * np.sqrt(np.mean(e[:, 0] ** 2)):.1f} mm，"
                f"姿态误差中位 {np.median(e[:, 1]):.2f}°，"
                f"距离 ArUco {np.median(e[:, 2]):.3f} m / 刚体间 {np.median(e[:, 3]):.3f} m"
            )
        self.get_logger().info(line)
        for buffer in (self.rig, self.board):
            buffer.received = 0
            buffer.stale = 0
        self.matched = 0
        self.unmatched = 0
        self.errors = []


def main():
    rclpy.init()
    node = MocapGroundTruthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
