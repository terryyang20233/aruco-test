#!/usr/bin/env bash
# 录一段实验数据。先起 camera.launch.py 和 mocap.launch.py，确认 ros2 topic hz 都正常，再跑这个。
#   ./record.sh 20261027_E1_d100_r01            # 默认存到 $BAG_DIR（没设就是 ~/bags）
#   ./record.sh 20261027_E8_r01 --no-images     # 只录位姿和 IMU（E8 长时间跑）
# Ctrl-C 结束后自动打印 ros2 bag info，看各话题条数对不对。
set -euo pipefail

run_name="${1:?用法: record.sh <run_name> [--no-images]}"
bag_dir="${BAG_DIR:-$HOME/bags}"
mkdir -p "$bag_dir"
output="$bag_dir/$run_name"
if [[ -e "$output" ]]; then
  echo "已经有 $output 了，换个名字" >&2
  exit 1
fi

topics=(
  /camera/camera/color/camera_info
  /camera/camera/infra1/camera_info
  /camera/camera/imu
  /tf_static
  /vrpn_mocap/cam_rig/pose
  /vrpn_mocap/marker_board/pose
  /aruco/pose /aruco/detection
  /ir/aruco/pose /ir/aruco/detection
  /mocap/error /ir/mocap/error
)
if [[ "${2:-}" != "--no-images" ]]; then
  topics+=(/camera/camera/color/image_raw /camera/camera/infra1/image_rect_raw)
fi

for topic in /camera/camera/color/image_raw /vrpn_mocap/cam_rig/pose /vrpn_mocap/marker_board/pose; do
  if ! ros2 topic list | grep -qx "$topic"; then
    echo "警告：$topic 现在没有发布者。先起节点再录，否则 best-effort 话题可能一条都录不到。" >&2
  fi
done

trap 'echo; ros2 bag info "$output"' EXIT
ros2 bag record -s mcap -o "$output" "${topics[@]}"
