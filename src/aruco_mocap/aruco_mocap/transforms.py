"""4×4 位姿矩阵和四元数的小工具。只用 numpy。

T_A_B 表示 B 在 A 里的位姿，把 B 坐标里的点变到 A 坐标里。四元数一律 (x, y, z, w)，和 ROS 一致。
"""

import numpy as np


def quat_to_matrix(q):
    x, y, z, w = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(rotation):
    r = np.asarray(rotation, dtype=np.float64)
    trace = np.trace(r)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def rvec_to_matrix(rvec):
    """Rodrigues 旋转向量 → 旋转矩阵，不依赖 OpenCV。"""
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
    angle = np.linalg.norm(rvec)
    if angle < 1e-12:
        return np.eye(3)
    k = rvec / angle
    skew = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def make_transform(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def from_translation_quat(translation, quat_xyzw):
    return make_transform(quat_to_matrix(quat_xyzw), translation)


def invert(transform):
    rotation = transform[:3, :3]
    return make_transform(rotation.T, -rotation.T @ transform[:3, 3])


def rotation_angle_deg(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def slerp(q0, q1, fraction):
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:  # 走短的那一半圆
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + fraction * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(dot)
    return (np.sin((1 - fraction) * theta) * q0 + np.sin(fraction * theta) * q1) / np.sin(theta)
