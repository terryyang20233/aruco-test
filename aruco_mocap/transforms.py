"""旋转向量、旋转矩阵和四元数的小工具。只用 numpy。

T_A_B 表示 B 在 A 里的位姿，把 B 坐标里的点变到 A 坐标里。四元数一律 (x, y, z, w)，和 ROS 一致。
"""

import numpy as np


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
