"""用 collect_views.py 收到的单块 ArUco 角点，检查出厂内参是否可信。

    python3 check_intrinsics.py /tmp/views.json

每个视角只有 4 个角点，约束比 ChArUco 弱很多，所以结论看两样：
1. 自己解出来的 fx、fy、cx、cy 和出厂值差多少，差值是标准差的几倍；
2. 固定出厂内参时的重投影误差，和放开内参时差多少。差得很少，说明出厂值已经够用。
fx 差 1%，PnP 解出的距离就差大约 1%。
标记边长只影响位姿，不影响内参，这里随便给一个。
"""

import argparse
import json

import cv2
import numpy as np

SIDE = 0.105


def object_points():
    half = SIDE / 2.0
    return np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )


def tilt_deg(rvec):
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    # 标记 Z 轴朝相机，所以它和相机 -Z 轴的夹角就是倾斜角
    return float(np.degrees(np.arccos(np.clip(-rotation[2, 2], -1.0, 1.0))))


def reprojection_rms(views, camera_matrix, dist):
    errors = []
    for corners in views:
        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points(), corners, camera_matrix, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        projected, _ = cv2.projectPoints(object_points(), rvecs[0], tvecs[0], camera_matrix, dist)
        errors.append(projected.reshape(4, 2) - corners)
    errors = np.concatenate(errors)
    return float(np.sqrt(np.mean(np.sum(errors ** 2, axis=1))))


def calibrate(views, size, factory_k, free_distortion):
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    if not free_distortion:
        flags |= cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2
    rms, k, d, _, _, std_intrinsics, _, _ = cv2.calibrateCameraExtended(
        [object_points()] * len(views),
        [v.astype(np.float32).reshape(-1, 1, 2) for v in views],
        size,
        factory_k.copy(),
        np.zeros(5),
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9),
    )
    return rms, k, d.ravel(), std_intrinsics.ravel()


def report(stream, data):
    info = data["camera_info"]
    raw_views = data["views"]
    if info is None:
        print(f"\n==== {stream}：没收到 camera_info，跳过")
        return
    print(f"\n==== {stream}：{info['width']}×{info['height']}，收下 {len(raw_views)} 个视角（共检出 {data['seen']} 帧）")
    if len(raw_views) < 6:
        print("视角太少（< 6），解不出可信的内参。多换几个倾斜姿态再收一次。")
        return
    views = [np.array(v["corners"], dtype=np.float64) for v in raw_views]
    tilts = np.array([tilt_deg(v["rvec"]) for v in raw_views])
    centers = np.array([v.mean(axis=0) for v in views])
    print(f"倾斜角：中位 {np.median(tilts):.0f}°，最大 {tilts.max():.0f}°，> 15° 的有 {(tilts > 15).sum()} 个")
    print(f"标记中心覆盖画面：x {centers[:, 0].min():.0f}–{centers[:, 0].max():.0f} px，"
          f"y {centers[:, 1].min():.0f}–{centers[:, 1].max():.0f} px")
    if (tilts > 15).sum() < 8:
        print("警告：倾斜 > 15° 的视角少于 8 个，焦距约束很弱，下面的数只能参考。")

    factory_k = np.array(info["k"], dtype=np.float64).reshape(3, 3)
    factory_d = np.array(info["d"] or [0.0] * 5, dtype=np.float64)
    size = (info["width"], info["height"])
    factory_rms = reprojection_rms(views, factory_k, factory_d)

    rms, k, d, std = calibrate(views, size, factory_k, free_distortion=False)
    names = ["fx", "fy", "cx", "cy"]
    factory_values = [factory_k[0, 0], factory_k[1, 1], factory_k[0, 2], factory_k[1, 2]]
    fitted_values = [k[0, 0], k[1, 1], k[0, 2], k[1, 2]]
    print(f"{'':4s}{'出厂':>10s}{'本次解出':>10s}{'差':>9s}{'标准差':>8s}{'差/标准差':>9s}")
    for name, f_value, k_value, s in zip(names, factory_values, fitted_values, std[:4]):
        diff = k_value - f_value
        print(f"{name:4s}{f_value:10.1f}{k_value:10.1f}{diff:+9.1f}{s:8.1f}{abs(diff) / max(s, 1e-9):9.1f}")
    print(f"重投影 RMS：固定出厂内参 {factory_rms:.3f} px，放开内参（无畸变）{rms:.3f} px")

    rms_d, k_d, d_d, _ = calibrate(views, size, factory_k, free_distortion=True)
    print(f"再放开 k1、k2：k1 {d_d[0]:+.4f}，k2 {d_d[1]:+.4f}，RMS {rms_d:.3f} px")

    focal_diff = 100.0 * (np.mean(fitted_values[:2]) / np.mean(factory_values[:2]) - 1.0)
    sigmas = [abs(a - b) / max(s, 1e-9) for a, b, s in zip(fitted_values, factory_values, std[:4])]
    print(f"焦距相对出厂 {focal_diff:+.2f}%（距离会按同样比例偏）")
    if max(sigmas) < 3.0 and factory_rms - rms < 0.1:
        print("结论：和出厂一致（差值都在 3 倍标准差以内，固定出厂内参的误差也几乎不变）。用出厂内参。")
    else:
        print("结论：和出厂有可测的差别。先确认视角是否够多、够斜；仍然如此再考虑用本次的值，或用 ChArUco 正式标定。")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("views_json")
    args = parser.parse_args()
    with open(args.views_json) as f:
        data = json.load(f)
    for stream, stream_data in data.items():
        report(stream, stream_data)


if __name__ == "__main__":
    main()
