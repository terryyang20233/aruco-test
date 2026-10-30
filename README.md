# aruco_mocap

机载用：D435i 看屋脊板上的两块 ArUco（0 号、1 号，7 cm），**每块标记一个节点，各自发布自己那一块的解**。只管相机和检测，不碰 mocap、不碰飞控和 IMU。

两块怎么合（融合、只取一块、还是别的方案）现在不定：先把两块各自的结果录下来，回来离线分析再决定。检测 JSON 里有每块的四个角点，所以联合 PnP 之类的方案不用图像也能离线算。

这个文件夹就是推到 GitHub、拉到 Orin 上的全部内容。第一轮 mocap 测试实际用的那一版（带真值节点、录包脚本）冻结在 `low-range/mocap-test/aruco_mocap/`，不再改。

## 在 Orin 上装

工作空间（`build/`、`install/`、`log/`）只在 Orin 上，仓库里没有。第一次：

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone <这个仓库的地址> aruco_mocap
```

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select aruco_mocap && source install/setup.bash
```

以后改了代码，只要在 `~/ros2_ws/src/aruco_mocap` 里 `git pull`。`--symlink-install` 下 Python 和 launch 文件不用重新编译；改了 `setup.py` 或新加、删了文件，再跑一次 `colcon build`。

OpenCV 要用 pip 装的 ≥ 4.7，见 `mocap-test/实验方案.md` §3.4。

## 跑起来

先量屋脊板，把 `config/markers.yaml` 里的边长改成实测值，夹角也记进去。也可以不改包里那份，另存一份传进去：`markers_file:=$HOME/markers.yaml`。

```bash
ros2 launch aruco_mocap camera.launch.py
```

确认两路都在出数（看见标记时约 30 Hz）：

```bash
ros2 topic hz /marker_0/aruco/detection
```

## 录包

和第一轮一样用 `ros2 bag record`，不录图像。先起节点，再开始录：

```bash
ros2 bag record -o ridge_$(date +%Y%m%d_%H%M%S) /marker_0/aruco/pose /marker_0/aruco/detection /marker_1/aruco/pose /marker_1/aruco/detection /camera/camera/color/camera_info
```

在 mocap 室里，再把实验室的 mocap 话题加在后面（第一轮是 `/uav_0/mocap /obj_0/mocap`），回来才有真值可比。`aruco_ir:=true` 时把 `/ir/marker_0/aruco/...`、`/ir/marker_1/aruco/...` 也加上。

## 话题

每块标记一组，`<编号>` 是 0 或 1：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/marker_<编号>/aruco/pose` | PoseStamped | 这块标记在彩色相机 optical frame 里的位姿，stamp = 图像 stamp。没看见就不发 |
| `/marker_<编号>/aruco/detection` | String（JSON） | 每帧一条，没看见时 `detected` 为 false。看见时有四个角点、IPPE 的两个解（`tvec`/`rvec` 和 `alt_tvec`/`alt_rvec`）和各自的重投影误差、处理耗时 |
| `/marker_<编号>/aruco/debug_image` | Image | 5 Hz，画了框和坐标轴 |

`aruco_ir:=true` 时左红外相机（infra1）也跑一份，话题在 `/ir/marker_<编号>/aruco/...`。

单块标记正对相机时姿态可能翻转，`tvec`/`rvec` 是重投影误差小的那个解，不一定是对的那个，另一个解在 `alt_*` 里。

两个节点各自订阅同一路图像、各自跑一遍检测，Orin 上每帧大约多花 7–12 ms。

## 里面有什么

| 位置 | 做什么 |
| --- | --- |
| `aruco_mocap/marker_pose.py` | 单块标记的检测 + IPPE 两个解 + 重投影误差。不依赖 ROS，离线分析也调它 |
| `aruco_mocap/aruco_node.py` | ROS 节点：图像 → 一块标记的位姿 |
| `aruco_mocap/transforms.py` | 旋转向量、四元数小工具 |
| `launch/camera.launch.py` | D435i（只开彩色，`aruco_ir:=true` 再开红外）+ 每块标记一个检测节点 |
| `config/markers.yaml` | 标记编号、边长、屋脊夹角 |
| `config/realsense.yaml` | 相机参数：关投射器、曝光、电网频率 |
| `scripts/collect_views.py`、`check_intrinsics.py` | 用一块标记的角点检查出厂内参（`--marker 0` 或 `1`） |

## 坐标系

- 相机：OpenCV optical frame，X 右、Y 下、Z 向前。
- 标记：原点在黑方块中心，X 右、Y 上、Z 穿出纸面朝相机。
