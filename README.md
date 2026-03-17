# Odin ROS2 驱动包

Manifold Tech Ltd. Odin 传感器 ROS2 驱动

Odin1 官方文档：https://manifoldtechltd.github.io/wiki/Odin1/Cover.html

## 重要说明

本驱动包面向**专业技术人员**进行二次开发，提供点云 SLAM 应用的核心功能。实际部署时需根据具体场景进行优化和定制开发。

---

## 1. 版本信息

| 项目 | 版本 |
|------|------|
| 驱动版本 | v0.9.0 |
| 所需设备固件版本 | v0.10.0 |
| 支持操作系统 | Ubuntu 22.04.5 LTS |
| 支持 ROS 版本 | ROS2 Humble |
| 支持架构 | x86_64 (AMD/Intel) / ARM / AArch64 |

---

## 2. 环境准备

### 2.1 系统要求

- **操作系统**：Ubuntu 22.04.5 LTS
- **ROS 版本**：ROS2 Humble
- **硬件架构**：x86_64 或 ARM/AArch64

### 2.2 安装 ROS2 Humble

参考官方文档：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

```bash
# 设置语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 添加 ROS2 apt 源
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble 桌面版
sudo apt update
sudo apt install ros-humble-desktop
```

### 2.3 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  libopencv-dev \
  libyaml-cpp-dev \
  libssl-dev \
  libeigen3-dev \
  libpcl-dev \
  libusb-1.0-0-dev \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-pcl-conversions \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-visualization-msgs
```

### 2.4 配置 USB 设备权限（udev 规则）

```bash
# 创建 udev 规则文件（设备 VID=2207, PID=0019）
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-odin.rules

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 将当前用户添加到 plugdev 组
sudo usermod -aG plugdev $USER
# 注意：需要重新登录后生效
```

---

## 3. 编译与构建

### 3.1 创建工作空间并克隆驱动

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <仓库地址> odin_ros_driver
```

### 3.2 编译

```bash
cd ~/ros2_ws

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译功能包
colcon build --packages-select odin_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载工作空间环境
source install/setup.bash
```

> **架构说明**：CMakeLists.txt 会自动检测当前架构（x86_64 或 ARM），
> 并选择对应的预编译库（`lib/liblydHostApi_amd.a` 或 `lib/liblydHostApi_arm.a`）。

### 3.3 验证编译结果

```bash
ros2 pkg list | grep odin_ros_driver
```

---

## 4. 启动驱动

### 4.1 快速启动（推荐）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch odin_ros_driver odin1.launch.py
```

### 4.2 自定义配置文件路径

```bash
ros2 launch odin_ros_driver odin1.launch.py \
  config_file:=/path/to/control_command.yaml
```

### 4.3 单独启动主驱动节点

```bash
ros2 run odin_ros_driver host_sdk_sample \
  --ros-args -p config_file:=/path/to/control_command.yaml
```

---

## 5. 配置说明

配置文件位于 `config/control_command.yaml`，所有参数均有详细中文注释。

### 5.1 运行模式（custom_map_mode）

| 模式值 | 名称 | 说明 |
|--------|------|------|
| `0` | 里程计模式 | map 坐标系 = odom 坐标系，轻量级定位，适合无需建图的场景 |
| `1` | SLAM 建图模式 | 带回环检测，实时建立三维地图，支持保存地图文件 |
| `2` | 重定位模式 | 加载已有地图进行定位，需设置 `relocalization_map_abs_path` |

> **注意**：SLAM 模式（mode 1 和 2）需要 USB 3.0 连接！

### 5.2 主要参数说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `strict_usb3.0_check` | `0` | USB 3.0 强制检查（推荐 SLAM 模式设为 1） |
| `use_host_ros_time` | `2` | 时间戳模式（0=设备时间, 1=ROS时间, 2=NTP对齐） |
| `sendrgbcompressed` | `1` | 发布 JPEG 压缩 RGB 图像 |
| `sendrgb` | `1` | 发布解压 BGR8 RGB 图像 |
| `sendimu` | `1` | 发布 IMU 数据 |
| `sendodom` | `1` | 发布里程计数据 |
| `senddtof` | `1` | 发布原始稀疏点云 |
| `sendcloudslam` | `1` | 发布 SLAM 彩色点云 |
| `sendcloudrender` | `1` | 发布彩色渲染点云 |
| `dtof_fps` | `100` | 传感器帧率（100=10fps, 145=14.5fps） |
| `cloud_raw_confidence_threshold` | `35` | 点云置信度过滤阈值（0~1300，推荐 30~35） |
| `senddepth` | `0` | 深度图生成（计算量大，默认关闭） |
| `sendreprojection` | `0` | 点云重投影演示（默认关闭） |
| `recorddata` | `0` | 录制 .olx 格式数据（默认关闭） |
| `custom_map_mode` | `0` | 运行模式（0=里程计, 1=SLAM, 2=重定位） |

---

## 6. ROS2 话题列表

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/odin1/imu` | `sensor_msgs/Imu` | IMU 数据（加速度 + 角速度） |
| `/odin1/image` | `sensor_msgs/Image` | RGB 图像（BGR8） |
| `/odin1/image/compressed` | `sensor_msgs/CompressedImage` | JPEG 压缩 RGB 图像 |
| `/odin1/image/undistorted` | `sensor_msgs/Image` | 去畸变 RGB 图像 |
| `/odin1/image/intensity_gray` | `sensor_msgs/Image` | dToF 强度灰度图（调试用） |
| `/odin1/cloud_raw` | `sensor_msgs/PointCloud2` | 原始稀疏点云（含置信度字段） |
| `/odin1/cloud_slam` | `sensor_msgs/PointCloud2` | SLAM 彩色点云（PointXYZRGB） |
| `/odin1/cloud_render` | `sensor_msgs/PointCloud2` | 彩色渲染点云 |
| `/odin1/odometry` | `nav_msgs/Odometry` | 里程计（标准频率） |
| `/odin1/odometry_highfreq` | `nav_msgs/Odometry` | 高频里程计 |
| `/odin1/depth_image` | `sensor_msgs/Image` | 密集深度图（32FC1，需 senddepth: 1） |
| `/odin1/depth_cloud` | `sensor_msgs/PointCloud2` | 深度彩色点云（需 senddepth: 1） |
| `/odin1/reprojected_image` | `sensor_msgs/Image` | 点云重投影图像（需 sendreprojection: 1） |

---

## 7. 点云字段格式（cloud_raw）

原始点云（`/odin1/cloud_raw`）每个点包含以下字段：

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `x`, `y`, `z` | float32 | 三维坐标（米） |
| `intensity` | float32 | 反射强度值 |
| `confidence` | float32 | 测量置信度（0~1300，过滤阈值参考 `cloud_raw_confidence_threshold`） |
| `offset_time` | float32 | 相对帧时间偏移（毫秒） |

---

## 8. SLAM 地图操作

### 8.1 保存 SLAM 地图

在 SLAM 建图模式运行时，使用以下命令触发地图保存：

```bash
# 方法一：使用 set_param.sh 脚本
./set_param.sh save_map 1

# 方法二：通过 ROS2 参数服务（效果等同）
echo "save_map=1" > /tmp/odin_command.txt
```

地图默认保存到：`{工作空间}/src/odin_ros_driver/map/{驱动启动时间}/map_{保存时间}.bin`

自定义保存路径在 `control_command.yaml` 中配置：
```yaml
mapping_result_dest_dir: "/home/user/my_maps"
mapping_result_file_name: "office_map"
```

### 8.2 使用已有地图重定位

在 `control_command.yaml` 中配置：

```yaml
custom_map_mode: 2
relocalization_map_abs_path: "/home/user/my_maps/office_map.bin"
```

---

## 9. RViz2 可视化

launch 文件会自动启动 RViz2，使用 `rviz/odin_ros2.rviz` 配置文件。

默认显示内容：
- 原始稀疏点云（`/odin1/cloud_raw`）
- SLAM 彩色点云（`/odin1/cloud_slam`）
- 彩色渲染点云（`/odin1/cloud_render`）
- 里程计轨迹
- RGB 图像

---

## 10. 目录结构

```
odin_ros_driver/
├── CMakeLists.txt          # CMake 构建配置（ROS2 Humble）
├── package.xml             # ROS2 功能包描述
├── config/
│   └── control_command.yaml  # 驱动控制参数配置文件
├── include/                # 头文件目录
│   ├── host_sdk_sample.h   # 主驱动头文件
│   ├── lidar_api.h         # LiDAR 设备 API
│   ├── lidar_api_type.h    # LiDAR 数据类型定义
│   ├── yaml_parser.h       # YAML 解析器
│   ├── rawCloudRender.h    # 点云彩色渲染
│   ├── data_logger.h       # 二进制数据记录器
│   ├── polynomial_camera.hpp        # 多项式畸变相机模型
│   ├── pointcloud_depth_converter.hpp # 点云转深度图
│   ├── depth_image_ros2_node.hpp    # 深度图 ROS2 节点
│   ├── cloud_reprojector.hpp        # 点云重投影核心
│   ├── cloud_reprojection_ros_node.hpp # 重投影 ROS2 节点
│   └── camera_pose_visualization.h  # 相机位姿可视化
├── src/                    # 源文件目录
│   ├── host_sdk_sample.cpp # 主驱动实现
│   ├── yaml_parser.cpp     # YAML 解析实现
│   ├── rawCloudRender.cpp  # 点云渲染实现
│   ├── camera_pose_visualization.cpp # 位姿可视化实现
│   ├── pointcloud_depth_converter.cpp # 深度转换实现
│   ├── depth_image_ros2_node.cpp    # 深度图节点实现
│   ├── pcd2depth_ros2.cpp           # 深度图节点入口
│   ├── cloud_reprojector.cpp        # 重投影核心实现
│   └── cloud_reprojection_ros.cpp   # 重投影节点实现
├── lib/                    # 预编译静态库
│   ├── liblydHostApi_amd.a # x86_64 架构库
│   └── liblydHostApi_arm.a # ARM/AArch64 架构库
├── launch/
│   └── odin1.launch.py     # ROS2 启动文件
├── rviz/
│   └── odin_ros2.rviz      # RViz2 可视化配置
└── set_param.sh            # 参数热更新脚本
```

---

## 11. 常见问题

### Q: 设备无法被识别？
**A**：检查 udev 规则是否正确安装，确保已将用户加入 `plugdev` 组并重新登录。
```bash
lsusb | grep 2207  # 应显示 Odin 设备
```

### Q: SLAM 模式下地图传输失败？
**A**：SLAM 模式需要 USB 3.0 连接。检查 USB 接口和线缆，建议将 `strict_usb3.0_check` 设为 `1` 以在连接时进行检测。

### Q: 点云显示不完整？
**A**：调整 `cloud_raw_confidence_threshold`。值越小，保留的点越多（但噪声也越多）；值越大，点云越稀疏但质量更高。推荐范围：30~35。

### Q: 时间戳不同步？
**A**：将 `use_host_ros_time` 设为 `2`（NTP 对齐模式），驱动会自动用 PTP 平滑算法将设备时间与主机时间对齐。

### Q: 编译报错找不到 OpenCV？
**A**：确保系统中只安装了一个版本的 OpenCV（推荐 4.5.5 或 4.8.0）。多版本共存可能导致符号冲突。
```bash
dpkg -l | grep libopencv  # 检查已安装的 OpenCV 版本
```

### Q: ARM 平台编译报链接错误？
**A**：CMakeLists.txt 已自动为 ARM 添加了 `-Wl,--no-as-needed` 和 RPATH。确保编译时系统正确识别为 ARM 架构：
```bash
uname -m  # 应显示 aarch64 或 armv7l
```

### Q: 如何录制数据用于离线处理？
**A**：在 `control_command.yaml` 中设置 `recorddata: 1`，数据将以 `.olx` 格式保存。录制完成后可使用 MindCloud 软件进行后处理。

---

## 12. 许可证

Copyright 2025 Manifold Tech Ltd. (www.manifoldtech.com.co)

Licensed under the Apache License, Version 2.0.
详见 [LICENSE](LICENSE) 文件。
