# software/ros_ws — 上位机 ROS2 工作区（Astra Pro 感知）

烟草智能打顶机器人上位机 ROS2 工作区。当前包含：

| 包 | 来源 | 说明 |
| --- | --- | --- |
| `astra_camera` | 奥比中光 OpenNI_ROS2_SDK v1.1.0（vendored，不改动） | Astra Pro 驱动：OpenNI2 深度/IR + UVC 彩色 |
| `astra_camera_msgs` | 同上 | 驱动自定义消息/服务 |
| `topping_perception` | 自研 | 启动封装 + `base_link→camera_link` 外参 TF；后续感知节点也进这里 |

依赖：`third_party/libuvc` 为 **git submodule（v0.0.9）**，构建时安装到工作区私有前缀
`third_party/install/`，**不写系统 `/usr/local`**。

## 1. 环境前提

- Ubuntu 22.04（x86_64 开发机；后续移植树莓派需 Ubuntu 22.04 arm64，见 §7）
- ROS2 Humble（`/opt/ros/humble`）
- apt 依赖：

```bash
sudo apt install libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev \
  libeigen3-dev nlohmann-json3-dev \
  ros-humble-image-geometry ros-humble-camera-info-manager \
  ros-humble-image-transport ros-humble-image-publisher
```

- udev 规则（每台机器一次，装完生成 `/dev/astra_pro`、`/dev/astrauvc`）：

```bash
sudo bash src/ros2_astra_camera/astra_camera/scripts/install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

- 克隆仓库后初始化 submodule：

```bash
git submodule update --init --recursive
```

## 2. 构建

```bash
software/ros_ws/scripts/build.sh
```

脚本自动完成两件事：

1. 若 `third_party/install/lib/pkgconfig/libuvc.pc` 不存在，先编 libuvc 并装到私有前缀；
2. `colcon build --base-paths src` 编译全部 ROS 包。

增量重编直接再跑一次即可（libuvc 会跳过）。

> 已知行为：SDK 的 `astra_camera/CMakeLists.txt` 硬编码 `CMAKE_BUILD_TYPE=Debug`，
> 命令行传的 Release 会被覆盖，属厂商源码现状，不改。

## 3. 运行

```bash
source /opt/ros/humble/setup.bash
source software/ros_ws/scripts/setup_env.sh   # 提供 libuvc 运行时路径
source software/ros_ws/install/setup.bash

ros2 launch topping_perception camera.launch.py
```

生产配置默认只开 **彩色 + 深度**（640×480@30）。调试时覆盖参数：

```bash
ros2 launch topping_perception camera.launch.py enable_ir:=true enable_point_cloud:=true
```

主要话题：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/camera/color/image_raw` + `camera_info` | rgb8 | YOLO11n-seg 输入 |
| `/camera/depth/image_raw` + `camera_info` | **16UC1，单位 mm** | 花顶 Z、按掩码反投影 |
| `/camera/ir/image_raw` | mono8 | 默认关闭 |
| `/camera/depth/points` | PointCloud2 | 默认关闭（全分辨率带宽高，感知用 image_geometry 按需反投影） |

> 深度值是毫米，进解算链前除以 1000 转米（README §8 口径）。

服务：`/camera/get_device_info`、`/camera/get_camera_info`、`/camera/get_sdk_version` 等
（`astra_camera_msgs/srv/*`）。

## 4. 坐标系与 TF

驱动自发布：`camera_link → camera_{color,depth,ir}_frame → *_optical_frame`。

本工作区只额外加一根静态外参 **`base_link → camera_link`**（README §8 的 `T_CV`），
数值在 `src/topping_perception/config/camera_mount.yaml`：

- 当前为**占位初值**（俯角 45°、高度 0.75 m 假设，带 TODO(D15/K1)）；
- D15 安装高度冻结后改设计值；K1 外参标定完成后用
  `calibration/astra_pro/extrinsics/` 档案覆盖。

深度/点云解算挂 `camera_depth_optical_frame`（OpenCV 惯例：Z 前、X 右、Y 下），即基线 C 系。
不要手建 `*_optical_frame` 的静变换（会与驱动重复父节点冲突）。

与仿真联看：本工作区与 `simulation/ros2_ws` 用同一 `ROS_DOMAIN_ID` 启动即可，
两边都以 `base_link` 为根，RViz Fixed Frame 选 `base_link` 同屏显示 Delta + 相机。

## 5. 参数说明（camera.launch.py）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `serial_number` | `""` | 空 = 接第一台设备；多机时填序列号（本机设备 `17122313163`）。**不要直接透传数字**——官方 XML 会把纯数字解析成 integer 参数，驱动声明的是 string，会崩 |
| `enable_ir` | `false` | 红外流 |
| `enable_point_cloud` | `false` | 深度点云 |
| `enable_colored_point_cloud` | `false` | 彩色点云 |
| `depth_registration` | `false` | 深度↔彩色逐像素对齐（开后在树莓派上需复测帧率） |
| `color_qos` / `depth_qos` | `sensor_data` | 图像 QoS |
| `color/depth_width/height/fps` | 640×480@30 | 分辨率帧率 |

## 6. 排障

| 现象 | 原因与处理 |
| --- | --- |
| cmake 报 `libuvc is not found` | 没 source `scripts/setup_env.sh`（`PKG_CONFIG_PATH` 缺私有前缀），或 libuvc 未构建（先跑 `build.sh`） |
| 链接报 `cannot find -luvc` | `LIBRARY_PATH` 未导出。SDK 只给 `-luvc` 不给 `-L`，必须靠它 |
| 运行报 `libuvc.so.0: cannot open shared object` | 运行时缺 `LD_LIBRARY_PATH`，source `setup_env.sh` |
| `Waiting for device connection...` 循环 | 相机没接 / USB 口供电不足 / udev 规则未装。`lsusb \| grep 2bc5` 应看到 `0403`+`0501` |
| 权限 denied | udev 规则未生效，重跑 §1 的 install.sh 并 `udevadm trigger`，重新插拔 |
| 启动时 class_loader SEVERE WARNING | SDK 把组件库直接链进可执行文件导致，厂商已知行为，**不影响功能** |
| 关闭时报 `cannot undeclare an statically typed parameter` | Humble 与 SDK 静态参数登记的兼容警告，进程正常退出，可忽略 |
| 高负载掉帧 | USB 事件线程无法提优先级（普通用户），USB2 下三路同开不稳定；生产只开 color+depth |

## 7. 树莓派部署（后续）

- 系统前提：**Ubuntu 22.04 arm64**（Humble deb 要求；Raspberry Pi OS 不可用 apt 源直装）。
- libuvc 必须**在板子上本地重编**（同一 `build.sh` 即可，产物自动为 arm64）。
- SDK 自带 `openni2_redist/arm64`，CMake 会按 `aarch64` 自动选用。
- 用蓝色 USB3 口；只开 color+depth。x86+USB2 实测 color 26–30 / depth 20–28 fps，
  树莓派需复测。

## 8. 目录速览

```text
software/ros_ws/
├─ src/
│  ├─ ros2_astra_camera/        # 厂商 SDK（vendored，勿改）
│  └─ topping_perception/       # 自研：launch 封装 + camera_mount.yaml 外参
├─ third_party/
│  ├─ libuvc/                   # submodule @ v0.0.9
│  └─ install/                  # 私有前缀（git 忽略，本地生成）
├─ scripts/
│  ├─ setup_env.sh              # libuvc 私有前缀环境（PKG_CONFIG/CMAKE/LIBRARY/LD_LIBRARY_PATH）
│  └─ build.sh                  # 一键构建
└─ build/ install/ log/         # colcon 产物（git 忽略）
```

验证记录与实测数据见 `doc/vision_solution/AstraPro_ROS2驱动集成记录.md`。
