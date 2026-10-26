"""ROS 2 节点：订阅一路图像和它的 camera_info，发布 ArUco 位姿。

发布（相对话题名，launch 里加命名空间就能同时跑彩色和红外两份）：
- aruco/pose        PoseStamped，坐标系 = 图像的 frame_id（相机 optical frame），stamp = 图像 stamp
- aruco/detection   String（JSON），每一帧一条，包括没看见的帧：角点、两个 IPPE 解、重投影误差、处理耗时
- aruco/debug_image Image（bgr8），画了框和坐标轴，降频发布，给笔记本看

不用 cv_bridge：它的 C++ 部分链接系统 OpenCV 4.5，和 pip 装的新版 OpenCV 混在一个进程里容易出问题。
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from aruco_mocap.marker_pose import MarkerDetector, dictionary_id
from aruco_mocap.transforms import matrix_to_quat, rvec_to_matrix


def image_to_array(msg):
    """sensor_msgs/Image → numpy。返回 (数组, 是否是 RGB 顺序)。"""
    channels = {"mono8": 1, "8UC1": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if msg.encoding not in channels:
        raise ValueError(f"不支持的图像编码 {msg.encoding}")
    count = channels[msg.encoding]
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, : msg.width * count].reshape(msg.height, msg.width, count)
    if count == 1:
        return image[:, :, 0], False
    if count == 4:
        image = image[:, :, :3]
    return image, msg.encoding.startswith("rgb")


class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_pose")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "")  # 空 = 和图像同一目录下的 camera_info
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_side_m", 0.15)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("corner_refinement", "subpix")
        self.declare_parameter("debug_image_hz", 5.0)
        self.declare_parameter("stats_period_s", 5.0)
        # realsense 和 v4l2_camera 默认都按 reliable 发图。best-effort 订阅大图时，DDS 丢一个分片就丢整帧，
        # 虚拟机里实测只收到一半。发布端改成 best-effort 时这里也要跟着改，否则两边连不上。
        self.declare_parameter("image_reliability", "reliable")

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value or (
            image_topic.rsplit("/", 1)[0] + "/camera_info"
        )
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.side = float(self.get_parameter("marker_side_m").value)
        self.detector = MarkerDetector(
            dictionary_id(self.get_parameter("dictionary").value),
            self.get_parameter("corner_refinement").value,
        )
        debug_hz = float(self.get_parameter("debug_image_hz").value)
        self.debug_period = 1.0 / debug_hz if debug_hz > 0 else None
        self.last_debug = 0.0

        self.camera_matrix = None
        self.dist = None
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, qos_profile_sensor_data)
        reliability = {
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        }[self.get_parameter("image_reliability").value]
        image_qos = QoSProfile(depth=2, reliability=reliability)
        self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "aruco/pose", 10)
        self.detection_pub = self.create_publisher(String, "aruco/detection", 10)
        self.debug_pub = self.create_publisher(Image, "aruco/debug_image", 1)

        self.frames = 0
        self.hits = 0
        self.processing_ms = []
        self.create_timer(float(self.get_parameter("stats_period_s").value), self.report)
        self.get_logger().info(
            f"图像 {image_topic}，内参 {info_topic}，标记 {self.marker_id} 号，边长 {self.side} m，"
            f"OpenCV {cv2.__version__}"
        )

    def on_camera_info(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info(
                f"收到内参：fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}，"
                f"畸变模型 {msg.distortion_model}，系数 {[round(float(v), 4) for v in msg.d]}"
            )
            if msg.distortion_model not in ("plumb_bob", "rational_polynomial", ""):
                self.get_logger().warn(f"畸变模型 {msg.distortion_model} 不是 OpenCV 的模型，结果会偏")
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("还没收到 camera_info，先不检测", throttle_duration_sec=2.0)
            return
        self.frames += 1
        image, is_rgb = image_to_array(msg)
        start = time.perf_counter()
        gray = image if image.ndim == 2 else cv2.cvtColor(
            image, cv2.COLOR_RGB2GRAY if is_rgb else cv2.COLOR_BGR2GRAY
        )
        result = self.detector.detect(gray, self.camera_matrix, self.dist, self.side, self.marker_id)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.processing_ms.append(elapsed_ms)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        record = {"stamp": stamp, "frame_id": msg.header.frame_id, "detected": result is not None,
                  "processing_ms": round(elapsed_ms, 3)}
        if result is not None:
            self.hits += 1
            record.update({
                "id": self.marker_id,
                "corners": result["corners"].round(3).tolist(),
                "tvec": result["tvec"].tolist(),
                "rvec": result["rvec"].tolist(),
                "reprojection_error": result["reprojection_error"],
                "alt_tvec": None if result["alt_tvec"] is None else result["alt_tvec"].tolist(),
                "alt_rvec": None if result["alt_rvec"] is None else result["alt_rvec"].tolist(),
                "alt_reprojection_error": result["alt_reprojection_error"],
            })
            self.pose_pub.publish(self.to_pose(msg.header, result))
        self.detection_pub.publish(String(data=json.dumps(record)))

        now = time.monotonic()
        if self.debug_period is not None and now - self.last_debug >= self.debug_period:
            self.last_debug = now
            self.publish_debug(msg.header, image, is_rgb, result)

    @staticmethod
    def to_pose(header, result):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = result["tvec"].tolist()
        x, y, z, w = matrix_to_quat(rvec_to_matrix(result["rvec"]))
        pose.pose.orientation.x, pose.pose.orientation.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = z, w
        return pose

    def publish_debug(self, header, image, is_rgb, result):
        if image.ndim == 2:
            canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            canvas = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if is_rgb else image.copy()
        if result is not None:
            corners = result["corners"].astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [corners], True, (0, 255, 0), 2)
            cv2.drawFrameAxes(canvas, self.camera_matrix, self.dist, result["rvec"], result["tvec"],
                              self.side * 0.5)
            x, y, z = result["tvec"]
            cv2.putText(canvas, f"x={x:.3f} y={y:.3f} z={z:.3f} m", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        out = Image()
        out.header = header
        out.height, out.width = canvas.shape[:2]
        out.encoding = "bgr8"
        out.step = out.width * 3
        out.data = np.ascontiguousarray(canvas).tobytes()
        self.debug_pub.publish(out)

    def report(self):
        if self.frames == 0:
            self.get_logger().info("这段时间没有收到图像")
            return
        times = np.array(self.processing_ms)
        self.get_logger().info(
            f"{self.frames} 帧，检出 {self.hits}（{100.0 * self.hits / self.frames:.0f}%），"
            f"处理耗时 中位 {np.median(times):.1f} ms / 95% {np.percentile(times, 95):.1f} ms"
        )
        self.frames = 0
        self.hits = 0
        self.processing_ms = []


def main():
    rclpy.init()
    node = ArucoPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
