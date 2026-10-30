"""从正在跑的 aruco_pose 节点收集单块 ArUco 的角点，用来检查相机内参。

只订阅 aruco/detection（JSON，含四个角点）和 camera_info，不碰相机、不碰检测节点。
姿态和已经收下的视角差得够多（转角 > min_angle_deg 或平移 > min_shift_m）才收，避免一堆几乎一样的帧。
还要求标记静止：四个角相对上一帧移动不超过 max_motion_px。彩色是卷帘快门，手持移动时方块会被拉歪，
这种帧混进来会把内参带偏（Orin 上实测：不筛时 fx 偏 +2.9%、cx 偏 −20 px，只用干净视角就回到出厂值）。

    python3 collect_views.py --seconds 120 --out /tmp/views.json
    python3 collect_views.py --streams color ir --marker 1   # 彩色和红外都收，用 1 号

拿着标记在相机前换一个姿态、停一下、再换下一个：上下左右各倾斜 20–45°，放到画面四角和中间，远近都有。
正对相机的视角对焦距几乎没有约束，倾斜的视角才有用。
"""

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

STREAMS = {  # 检测话题里的 {marker} 换成 --marker 给的编号
    "color": ("/marker_{marker}/aruco/detection", "/camera/camera/color/camera_info"),
    "ir": ("/ir/marker_{marker}/aruco/detection", "/camera/camera/infra1/camera_info"),
}


def rotation_between_deg(rvec_a, rvec_b):
    import cv2

    ra, _ = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64))
    rb, _ = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64))
    cosine = np.clip((np.trace(ra.T @ rb) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


class Collector(Node):
    def __init__(self, streams, marker, min_angle, min_shift, max_reprojection, max_motion):
        super().__init__("collect_views")
        self.max_motion = max_motion
        self.min_angle = min_angle
        self.min_shift = min_shift
        self.max_reprojection = max_reprojection
        self.data = {s: {"camera_info": None, "views": [], "seen": 0} for s in streams}
        self.previous_corners = {s: None for s in streams}
        for stream in streams:
            detection_topic, info_topic = STREAMS[stream]
            self.create_subscription(String, detection_topic.format(marker=marker),
                                     lambda m, s=stream: self.on_detection(s, m), 50)
            self.create_subscription(CameraInfo, info_topic,
                                     lambda m, s=stream: self.on_info(s, m), qos_profile_sensor_data)

    def on_info(self, stream, msg):
        if self.data[stream]["camera_info"] is None:
            self.data[stream]["camera_info"] = {
                "width": msg.width, "height": msg.height, "k": list(msg.k), "d": list(msg.d),
                "distortion_model": msg.distortion_model,
            }

    def on_detection(self, stream, msg):
        record = json.loads(msg.data)
        if not record.get("detected"):
            self.previous_corners[stream] = None
            return
        self.data[stream]["seen"] += 1
        corners = np.array(record["corners"])
        previous = self.previous_corners[stream]
        self.previous_corners[stream] = corners
        if previous is None or np.abs(corners - previous).max() > self.max_motion:
            return
        if record["reprojection_error"] > self.max_reprojection:
            return
        for view in self.data[stream]["views"]:
            shift = np.linalg.norm(np.subtract(view["tvec"], record["tvec"]))
            if shift < self.min_shift and rotation_between_deg(view["rvec"], record["rvec"]) < self.min_angle:
                return
        self.data[stream]["views"].append(
            {k: record[k] for k in ("stamp", "corners", "rvec", "tvec", "reprojection_error")}
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--out", default="/tmp/views.json")
    parser.add_argument("--streams", nargs="+", default=["color"], choices=list(STREAMS))
    parser.add_argument("--marker", type=int, default=0, help="用屋脊板上哪一块标记的角点")
    parser.add_argument("--min-angle-deg", type=float, default=8.0)
    parser.add_argument("--min-shift-m", type=float, default=0.05)
    parser.add_argument("--max-reprojection-px", type=float, default=0.5)
    parser.add_argument("--max-motion-px", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = Collector(args.streams, args.marker, args.min_angle_deg, args.min_shift_m, args.max_reprojection_px,
                     args.max_motion_px)
    start = time.time()
    last_print = start
    while time.time() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - last_print >= 10.0:
            last_print = time.time()
            counts = "，".join(f"{s} 收下 {d['views'].__len__()} 个视角（检出 {d['seen']} 帧）"
                              for s, d in node.data.items())
            print(f"{time.time() - start:5.0f} s：{counts}", flush=True)
    with open(args.out, "w") as f:
        json.dump(node.data, f)
    print(f"写入 {args.out}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
