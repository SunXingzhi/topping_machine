# Astra Pro ROS2 驱动集成记录

> 日期：2026-11-10。状态：已在开发机（Ubuntu 22.04 x86_64 + ROS2 Humble）完成构建与节点级验证；
> 相机数据流验证待插机复测。树莓派部署为后续任务。

## 1. 集成落点

| 内容 | 位置 |
| --- | --- |
| 驱动包（vendored） | `software/ros_ws/src/ros2_astra_camera/`（`astra_camera` + `astra_camera_msgs`，OpenNI_ROS2_SDK v1.1.0，源码零改动） |
| 自研封装 | `software/ros_ws/src/topping_perception/`（`camera.launch.py` + `config/camera_mount.yaml`） |
| libuvc | `software/ros_ws/third_party/libuvc`（git submodule @ v0.0.9），构建产物装到私有前缀 `third_party/install/` |
| 构建/环境脚本 | `software/ros_ws/scripts/{build.sh, setup_env.sh}` |
| 使用文档 | `software/ros_ws/README.md` |
| 标定档案 | `calibration/astra_pro/{intrinsics, extrinsics}/`（待 K1 标定后落档） |

## 2. 设备信息（来自隔离验证环境实测）

- 深度/IR：`2bc5:0403`（OpenNI2 协议）；彩色：`2bc5:0501`（标准 UVC，经 libuvc）
- 序列号：`17122313163`；USB 2.0 协商（480 Mbps）
- udev 规则生效后生成 `/dev/astra_pro`、`/dev/astrauvc`

## 3. 实测数据（隔离验证环境，同版本 SDK）

- 彩色 `640x480 rgb8`、深度 `640x480 16UC1`（**单位 mm**）、IR `640x480 mono8`
- 稳定帧率（USB2，三路同开）：Color 26–30 Hz / Depth 20–28 Hz / IR 29–30 Hz / 点云 21–25 Hz
- 深度帧有效非零像素 69,952，距离范围 711–3991 mm
- UVC 原始格式支持 YUY2 / MJPEG，最高 1280x720@30
- RViz / rqt 订阅验证通过

## 4. 集成时确认的坑（已规避）

1. **`-luvc` 链接失败**：SDK CMake 用 `pkg_search_module` 找头文件，但不把库目录传给链接器。
   由 `scripts/setup_env.sh` 导出 `LIBRARY_PATH`（链接）与 `LD_LIBRARY_PATH`（运行）解决。
2. **`serial_number` 类型崩溃**：官方 launch 的 `serial_number` 默认值是整数 `0`，
   XML `$(var ...)` 会把纯数字解析成 integer 参数，而驱动声明为 string →
   `InvalidParameterTypeException`。`topping_perception` 默认传空串（接第一台设备），不透传数字。
3. **CMAKE_BUILD_TYPE 被硬编码 Debug**：厂商源码第 10 行写死，命令行 Release 无效，接受现状。
4. **udev 规则**：仓库内只保留 SDK 自带脚本，安装动作写入使用文档（每台机器一次），不进构建流程。
5. **TF 父节点冲突**：驱动自发布 `camera_link → *_frame → *_optical_frame`，
   外部只加 `base_link → camera_link` 一根静变换，不为 optical frame 另建变换。
6. **class_loader 命名空间警告 / 退出时静态参数警告**：厂商已知行为，不影响功能，记录在排障表。

## 5. 本次验证结果（开发机）

- [x] `scripts/build.sh` 从零构建通过（libuvc 0.0.9 私有前缀 + 3 个 ROS 包）
- [x] `ros2 pkg list` 可见 `astra_camera` / `astra_camera_msgs` / `topping_perception`
- [x] `ros2 launch topping_perception camera.launch.py --show-args` 参数解析正常
- [x] `ldd astra_camera_node` 链接到本工作区私有前缀的 `libuvc.so.0` 与随包 `libOpenNI2.so`
- [x] 节点 `/camera/camera` 启动、TF `base_link → camera_link` 正常发布
- [ ] 插机后的数据流验证（topic hz / get_camera_info / 取图）——**待相机接入后执行 §6 清单**

## 6. 插机复测清单（相机接入后）

```bash
source /opt/ros/humble/setup.bash
source software/ros_ws/scripts/setup_env.sh
source software/ros_ws/install/setup.bash
ros2 launch topping_perception camera.launch.py
```

另开终端（同样 source 后）：

- [ ] `ros2 topic hz /camera/depth/image_raw` ≥ 20 Hz
- [ ] `ros2 topic hz /camera/color/image_raw` ≥ 20 Hz
- [ ] `ros2 service call /camera/get_camera_info astra_camera_msgs/srv/GetCameraInfo '{}'`
- [ ] `ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame`
- [ ] `rqt -s rqt_image_view/ImageView --args /camera/color/image_raw` 图像正常
- [ ] 与 `simulation/ros2_ws` 同 domain 启动，RViz 同屏显示 Delta + 相机 TF

## 7. 与工程基线的差异/待办

- 相机选型：基线原为 RealSense D435i，实际落地 Astra Pro（README §5 已回写）。
- 结构光原理：强日照下失效早于双目，降级阈值需田间实测（并入数据集采集任务）。
- 最小测距 0.6 m + 深度 FOV 58°×45°：D15 安装高度与横向覆盖需按安装几何复核（并入 D15）。
- 树莓派部署：系统需 Ubuntu 22.04 arm64；libuvc 板端重编；只开 color+depth（README 使用文档 §7）。
