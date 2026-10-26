"""单块 ArUco 的位姿。不依赖 ROS：Jetson 上的节点和 Mac 上的离线分析调用同一个函数。

约定和 isaac-sim/detect_aruco.py 一样：标记坐标原点在中心，X 向右，Y 向上，Z 穿出纸面朝相机；
输出在 OpenCV 相机坐标系里（X 右、Y 下、Z 向前），单位米。

比那一份多做的事（见 实验方案.md §1）：
- 按编号挑角点，画面里有别的标记时不会拿错；
- 亚像素角点细化；
- 用 IPPE_SQUARE 同时拿到两个解和各自的重投影误差，用来分析姿态翻转；
- 单通道（红外）图直接用，不再转灰度；
- 兼容 OpenCV 4.5（Ubuntu 22.04 自带）到 5.x 的接口差别。
"""

import cv2
import numpy as np

DICTIONARY_ID = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
SIDE = 0.15  # 黑方块外沿，米。打印后用尺子量，改成实测值。

REFINEMENT = {
    "none": cv2.aruco.CORNER_REFINE_NONE,
    "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
    "contour": cv2.aruco.CORNER_REFINE_CONTOUR,
    "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
}


def dictionary_id(name):
    """'DICT_4X4_50' 这样的名字换成 OpenCV 的常量。"""
    return getattr(cv2.aruco, name)


def marker_object_points(side=SIDE):
    """标记坐标系里的四个角，顺序和 OpenCV 返回的角点一致：左上、右上、右下、左下。"""
    half = side / 2.0
    return np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )


def _detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


class MarkerDetector:
    def __init__(self, dictionary=DICTIONARY_ID, refinement="subpix"):
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        parameters = _detector_parameters()
        parameters.cornerRefinementMethod = REFINEMENT[refinement]
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detect = cv2.aruco.ArucoDetector(aruco_dictionary, parameters).detectMarkers
        else:  # OpenCV < 4.7
            self._detect = lambda gray: cv2.aruco.detectMarkers(
                gray, aruco_dictionary, parameters=parameters
            )

    def detect(self, image, camera_matrix, dist=None, side=SIDE, marker_id=MARKER_ID):
        """返回一个 dict；没看见、或同一编号出现不止一次时返回 None。

        rvec、tvec 是重投影误差最小的那个解；alt_rvec、alt_tvec 是 IPPE 的另一个解。
        reprojection_error 是四个角的均方根，单位像素。
        """
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return None
        hits = [c for c, i in zip(corners, ids.ravel()) if i == marker_id]
        if len(hits) != 1:
            return None

        image_points = hits[0].reshape(4, 2).astype(np.float64)
        if dist is None:
            dist = np.zeros(5)
        count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
            marker_object_points(side),
            image_points,
            np.asarray(camera_matrix, dtype=np.float64),
            np.asarray(dist, dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if count == 0:
            return None
        errors = np.asarray(errors, dtype=np.float64).ravel()
        order = np.argsort(errors)
        best = order[0]
        result = {
            "corners": image_points,
            "rvec": np.asarray(rvecs[best]).reshape(3),
            "tvec": np.asarray(tvecs[best]).reshape(3),
            "reprojection_error": float(errors[best]),
            "alt_rvec": None,
            "alt_tvec": None,
            "alt_reprojection_error": None,
        }
        if count > 1:
            other = order[1]
            result["alt_rvec"] = np.asarray(rvecs[other]).reshape(3)
            result["alt_tvec"] = np.asarray(tvecs[other]).reshape(3)
            result["alt_reprojection_error"] = float(errors[other])
        return result


_default_detector = None


def detect_pose(bgr, camera_matrix, dist=None, side=SIDE):
    """和 isaac-sim/detect_aruco.detect_pose 一样的接口：返回 (tvec, rvec) 或 None。"""
    global _default_detector
    if _default_detector is None:
        _default_detector = MarkerDetector()
    result = _default_detector.detect(bgr, camera_matrix, dist, side)
    if result is None:
        return None
    return result["tvec"], result["rvec"].reshape(3, 1)
