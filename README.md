# Odin_ROS_Driver Readme

ROS driver suite for Odin sensor modules (Manifold Tech Ltd.) 

Odin1 wiki: https://manifoldtechltd.github.io/wiki/Odin1/Cover.html

## Odin_ROS_Driver

Compatibility:

● ROS 1(LTS Release: Noetic recommended)

● ROS 2(LTS Release: Humble recommended)

## Important Notice:

This driver package provides core functionality for point cloud SLAM applications and targets specific use cases. It is intended exclusively for technical professionals conducting secondary development. End users must perform scenario-specific optimization and custom development to align with operational requirements in practical deployment environments.

## 1. Version

Current version: v0.13.1

Required device firmware version: v0.13.0

## 2. Preparation

### 2.1 OS Requirement

● Ubuntu 20.04 for ROS Noetic and ROS2 Foxy;

● Ubuntu 22.04 for ROS2 Humble;

● Ubuntu 18.04 is currently not supported;

● Ubuntu 24.04 is not officially supported but may work with some modifications.

### 2.2 Dependencies

● Opencv >= 4.2.0(recommand 4.5.5/4.8.0. Make sure only one version of opencv is installed)

● yaml-cpp

● thread

● OpenSSL

● Eigen3

### 2.3 Dependencies Install

#### 2.3.1 System
```shell
sudo apt update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

#### 2.3.2 yaml-cpp
```shell
sudo apt update
sudo apt install -y libyaml-cpp-dev
```

#### 2.3.3 libusb
```shell
sudo apt update
sudo apt install -y libusb-1.0-0-dev
```

#### 2.3.4 opencv
```shell
sudo apt update
sudo apt-get install libopencv-dev
```

#### 2.3.4 ROS install

For ROS Noetic installation, please refer to:
[ROS Noetic installation instructions](https://wiki.ros.org/noetic/Installation)

For ROS2 Foxy installation, please refer to:
[ROS Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

For ROS2 Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## 3. Preparation

### 3.1 Create Udev rules 
```shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```
Add the following content to the 99-odin-usb.rules file
```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```
Reload rules and reinsert devices
```shell
sudo udevadm control --reload
sudo udevadm trigger
```
### 3.2 OS Requirement
```shell
git clone https://github.com/manifoldsdk/odin_ros_driver.git catkin_ws/src/odin_ros_driver
```
Note:
Please clone the source code into the "[ros_workspace]/src/" folder, otherwise compilation errors will occur.

### 3.3 make

#### 3.3.1 ROS1 (Noetic for example):

```shell
source /opt/ros/noetic/setup.bash
./script/build_ros.sh
```

#### 3.3.2 ROS2 (Foxy for example):

```shell
source /opt/ros/foxy/setup.bash
./script/build_ros2.sh
```

### 3.4 run:

#### 3.4.1 ROS1 (Noetic for example):

```shell
source [ros_workspace]/devel/setup.bash
roslaunch odin_ros_driver [launch file]
```
● odin_ros_driver: package name;

● launch file: launch file;

● ros_workspace: User's ROS environment workspace;
```shell
roslaunch odin_ros_driver odin1_ros1.launch
```
#### 3.4.2 ROS2 (Foxy for example):

```shell
source [ros2_workspace]/install/setup.bash
ros2 launch odin_ros_driver [launch file]
```
● odin_ros_driver: package name;

● launch file: launch file;

● ros2_workspace: User's ROS2 environment workspace;

ROS2 Demo Launch Instructions:
```shell
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 3.5 Operation Mode:

The operation mode can be configured via the `custom_map_mode` parameter in config/control_command.yaml.

#### Odometry mode

Set `custom_map_mode = 0` to enable odometry mode. In this mode, the map frame and odom frame share the same pose.

If the odom data is found to drift, the script command "./set_param.sh algo_reset 1" can be used to dynamically reset the algorithm.

#### SLAM mode

Set `custom_map_mode = 1` to enable slam mode. This mode provides a complete SLAM system that builds upon the Odometry Mode by adding **loop closure detection** and **map saving** capabilities.

After launching the driver, odin1 will automatically perform mapping and cache map data. When the scene capture is complete, users need to execute `./set_param.sh save_map 1` in the driver's source directory to save all map data collected since the program started. The map will be saved to the location specified by the `mapping_result_dest_dir` and `mapping_result_file_name` parameters in config/control_command.yaml. If these parameters are not specified, default values will be used.

After the initial save, you can execute the command again to save a new map. Each save operation will generate a new map file. (Please allow at least 5 seconds between consecutive save operations)

The map origin corresponds to the odom coordinate system's origin at the program's startup.

##### Relocalization mode

To enable relocalization, set `custom_map_mode = 2` and specify the absolute path to the pre-built map using the `relocalization_map_abs_path` parameter in config/control_command.yaml.

Once launched, odin1 will initiate the relocalization process based on the current viewpoint and the specified map. To ensure a high success rate, it is recommended to starting within 1 meter ±10 degrees of the original position and orientation from the SLAM trajectory.

Note that relocalization performance is highly environment-dependent. In highly distinctive scenes, successful matching may occur even beyond the 1m/10° range, while other environments may require more stringent conditions. We advise testing in your target environment to determine practical tolerances.

If relocalization fails initially, the system will temporarily operate in a fallback SLAM mode (map saving is disabled in this state). During this time, you can freely move odin1. It will continue relocalization attempts in the background. Once successful, the TF between map and odom frames will be published. (Tip: Gently shaking or moving the device after initialization can help improve relocalization accuracy.)

The following topics are published in the odom frame: `/odin1/cloud_slam, /odin1/odom, /odin1/highodom and /odin1/path`. To obtain these in the map frame, apply the TF from odom frame to map frame.

## 4. File structure and data format
### 4.1 File structure
```shell
Odin_ROS_Driver/                // ROS1/ROS2 driver package
    3rdparty/                   // Third-party libraries
    src/
        host_sdk_sample.cpp     // Example source code
        yaml_parser.cpp         // Source code for reading yaml parameters
        rawCloudRender.cpp      // Source code for RenderCloud
        depth_image_ros_node.cpp //depth_image_ros_node
        depth_image_ros2_node.cpp //depth_image_ros2_node
        pcd2depth_ros.cpp       //Source code for pcd2depth_ros
        pcd2depth_ros2.cpp      //Source code for pcd2depth_ros2
        pointcloud_depth_converter.cpp //Source code for pointcloud_depth_converter
        cloud_reprojection_ros.cpp //Source code for cloud reprojection node (ROS1/ROS2)
        cloud_reprojector.cpp   //Core logic for cloud reprojection
    lib/
        liblydHostApi_amd.a     // Static library for AMD platform
        liblydHostApi_arm.a     // Static library for ARM platform
    include/
        host_sdk_sample.h       // Example header file
        lidar_api_type.h        // API data structure header file
        lidar_api.h             // API function declarations
        yaml_parser.h           // Parameter file reading header file
        rawCloudRender.h        // API about RenderCloud
        data_logger.h           // LOG about save_data
        depth_image_ros_node.hpp // depth_image_ros_node
        depth_image_ros2_node.hpp // depth_image_ros2_node
        pointcloud_depth_converter.hpp // pointcloud_depth_convert
        cloud_reprojection_ros_node.hpp // cloud_reprojection_ros_node (ROS1/ROS2)
        cloud_reprojector.hpp   // Core class for cloud reprojection
    config/
        control_command.yaml    // Control parameter file for driver
        calib.yaml              // Machine calibration yaml，differ for each individual device. Retrieved from the device everytime it connects to ROS driver
    launch_ROS1/
        odin1_ros1.launch       // ROS1 launch file
    launch_ROS2/
        odin1_ros2.launch.py    // ROS2 launch file
    script/
        build_ros1.sh           // Installation script for ROS1
        build_ros2.sh           // Installation script for ROS2
    recorddata/                 // holds recorded data that can import into MindCloud
    log/                        // holds log files
        Driver_{timestamp}/     // holds all log folders for each time driver started
            Conn_{timestamp}/   // holds all log files for each odin1 device connection
                dev_status.csv  // device status log file
    README.md                   // Usage instructions
    CMakeLists.txt              // CMake build file
    License                     // License file
```
### 4.2 File structure
| Launch File Name         | Description |
|--------------------------|-------------|
| odin1_ros1.launch        | Launch file for ROS1 - Odin1 Basic Operations Demo |
| odin1_ros2.launch.py     | Launch file for ROS2 - Odin1 Basic Operations Demo |


### 4.3 ROS topics
Internal parameters of the Odin ROS driver are defined in config/control_command.yaml. Below are descriptions of the commonly used parameters:

| Topic               |control_command.yaml  | Detailed Description |
|---------------------|----------------------|----------------------|
| odin1/imu                     | sendimu           | Imu Topic |
| odin1/image                   | sendrgb           | RGB Camera Topic, decoded from original jpeg data from device, bgr8 format |
| odin1/image_undistort         | sendrgbundistort  | undistorted RGB Camera Topic, processed with calib.yaml from device |
| odin1/image/compressed        | sendrgbcompressed | RGB Camera compressed Topic, original jpeg data from device |
| odin1/cloud_raw               | senddtof          | Raw_Cloud Topic |
| odin1/cloud_render            | sendcloudrender   | Render_Cloud Topic, processed with raw point cloud, rgb image, and calib.yaml from device |
| odin1/cloud_slam              | sendcloudslam     | Slam_PointCloud Topic |
| odin1/odometry                | sendodom          | Odom Topic |
| odin1/odometry_high           | sendodom          | high frequency Odom Topic |
| odin1/path                    | showpath          | Odom Path Topic |
| tf                            | sendodom          | tf tree Topic |
| odin1/depth_img_competetion   | senddepth         | Dense depth image Topic. Demo, high computing power required. One-to-one with odin1/image_undistort. To utilize the data please directly subscribe to this topic instead of echoing it. Original value is already depth data, no need for further convert. |
| odin1/depth_img_competetion_cloud  | senddepth         | Dense Depth_Cloud Topic. Demo, high computing power required |
| odin1/reprojected_image       | sendreprojection  | Reprojected cloud to image Topic. Projects cloud_slam to camera image using odometry. Processed on host device. |

### 4.4 Data format

1. The raw point cloud (cloud_raw) has the following fields:
```
float32 x             // X axis, in meters
float32 y             // Y axis, in meters
float32 z             // Z axis, in meters
uint8  intensity      // Reflectivity, range 0–255
uint16 confidence     // Point confidence, actual value range from 0 to around 1300 in typical scene, higher value means more reliable. Recommanded filtering threshold is 30-35, should be adjusted accordingly.
float32 offset_time   // Time offset relative to the base timestamp unit: s 
```

To work with this custom format in PCL, first define the point type:
```cpp
/*** LS ***/
namespace ls_ros {
    struct EIGEN_ALIGN16 Point {
        float x;
        float y;
        float z;
        uint8_t intensity;
        uint16_t confidence;
        float offset_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ls_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ls_ros::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (uint8_t, intensity, intensity)
      (uint16_t, confidence, confidence)
      (float offset_time , offset_time)
)
```
Then, you can easily convert a ROS sensor_msgs::PointCloud2 message into a PCL point cloud:
```
pcl::PointCloud<ls_ros::Point> ls_cloud;
pcl::fromROSMsg(*msg, ls_cloud);
```

2. The slam point cloud (cloud_slam) and directly rendered point cloud (cloud_render) has the following fields:
```
float32 x             // X axis, in meters
float32 y             // Y axis, in meters
float32 z             // Z axis, in meters
float32 rgb           // RGB value
```

### 4.5 Other functionalities

|control_command.yaml   | Detailed Description |
|-----------------------|----------------------|
| use_host_ros_time     | Time synchronization mode: 0 - use odin internal system time as data timestamp (typical and recommended); 1 - use host ROS time upon receive (not recommended for most users); 2 - align odin1 time to host time via NTP-like synchronization, timestamp is the sensor data reception time on host time axis. |
| strict_usb3.0_check   | Strict USB3.0 check, if off, allow connection even if usb connection is below usb 3.0 |
| recorddata            | Record data in specific format that can be imported into MindCloud(TM) for post-processing. Please be aware that this will consume a lot of storage space. Testing shows 9.5G for 10mins of data. The per-frame timestamps written into the recorded files (IMU / image / point cloud / pose / rotate) follow the same alignment policy as `use_host_ros_time`, so under NTP mode (`use_host_ros_time=1` or `2`) the recorded timestamps are NTP-aligned host time instead of odin1 boot time. <br>录制文件 (IMU / 图像 / 点云 / Pose / Rotate) 中每帧的时间戳与 `use_host_ros_time` 采用相同对齐策略：在 NTP 模式 (`use_host_ros_time=1` 或 `2`) 下，录制时间戳为 NTP 对齐后的主机时间，而非 odin1 开机时间。 |
| devstatuslog          | Device status logging, currently save device status (soc temperature, cpu usage, ram usage, dtof sensor temp .etc) and data tx & rx rate to devstatus.csv under log folder. A new file will be created every time the driver is started. |
| showcamerapose        | Display Camera Pose and Field of View. |
| custom_map_mode        | Operation Modes: Mode 0 - Odometry mode: The map frame and odom frame share the same pose. Mode 1 - Mapping (with loop closure) mode: This mode supports map saving. Mode 2 - Relocalization mode: Requires specifying the absolute path to the map file. After successful relocalization, it will output the TF relationship between the map and odom frames.|
| custom_init_pos        | Initialization Position (currently unused). |
| relocalization_map_abs_path        | Absolute Path to Map File: Used for relocalization mode. |
| mapping_result_dest_dir and mapping_result_file_name| Path and Name for Saving Maps in Mapping Mode: If not specified, default values will be used. |

### 4.6 Runtime AE/AWB Tuning via ROS Service / 通过 ROS Service 在线调节 AE/AWB

The driver hosts four ROS services that let a side terminal tune the
camera's auto exposure (AE) and auto white balance (AWB) at runtime,
while the main data streams keep flowing. The same SDK call is shared
with the driver's main control path and serialised by an internal
mutex, so it is safe to invoke these services concurrently with normal
operation.

驱动启动后会注册 4 个 ROS Service，允许在不重启 driver 的前提下，从另一个终端动态调节
相机的自动曝光（AE）和自动白平衡（AWB）。底层 SDK 调用与驱动主控制路径共享同一把
互斥锁，因此可以与正常数据流并发调用。

**Service list / Service 一览**

| Service name | Type / 类型 | Purpose / 用途 |
|---|---|---|
| `/odin1/get_ae`  | `odin_ros_driver/srv/GetAe`  | Query current AE status / 查询当前 AE 状态 |
| `/odin1/get_awb` | `odin_ros_driver/srv/GetAwb` | Query current AWB status / 查询当前 AWB 状态 |
| `/odin1/set_ae`  | `odin_ros_driver/srv/SetAe`  | Set AE mode and (manual) exposure / gain / 设置 AE 模式和手动曝光/增益 |
| `/odin1/set_awb` | `odin_ros_driver/srv/SetAwb` | Set AWB mode and (manual) R/B gain / 设置 AWB 模式和手动 R/B 增益 |

#### 4.6.1 Request fields, ranges, physical meaning / 请求字段、范围与物理含义

**`SetAe.Request`**

| Field | Range / 范围 | Meaning / 含义 |
|---|---|---|
| `mode` | `0` (AUTO) or / 或 `1` (MANUAL) | `0` = device runs its own AE loop, the two floats below are ignored / 设备自动调 AE，下方参数被忽略<br>`1` = device locks AE and applies the provided values / 设备锁 AE 并应用提供的值 |
| `exposure_time` | `0.0001` ~ `0.033` s (manual only / 仅手动模式) | Sensor exposure time per frame. Longer = brighter but more motion blur / 每帧传感器曝光时间。越长越亮但运动模糊增大 |
| `gain` | `1.0` ~ `64.0` (manual only / 仅手动模式) | Analog gain. Higher = brighter output but worse SNR / 模拟增益。越大越亮但信噪比越差 |

**`SetAwb.Request`**

| Field | Range / 范围 | Meaning / 含义 |
|---|---|---|
| `mode`  | `0` (AUTO) or / 或 `1` (MANUAL) | `0` = device runs its own AWB loop / 设备自动 AWB<br>`1` = device locks AWB and applies provided gains / 设备锁定 AWB 并应用所给增益 |
| `rgain` | `0.1` ~ `4.0` (manual only / 仅手动模式) | R channel gain. Higher `rgain` vs `bgain` shifts the image warm (yellow/red) / R 通道增益，相对 bgain 越大，画面越偏暖 |
| `bgain` | `0.1` ~ `4.0` (manual only / 仅手动模式) | B channel gain. Higher `bgain` vs `rgain` shifts the image cool (blue) / B 通道增益，相对 rgain 越大，画面越偏冷 |

> Gr / Gb channels are fixed to 1.0 by the device and are not adjustable.
> Gr / Gb 通道被设备固定为 1.0，不可调节。

#### 4.6.2 Response fields / 响应字段

All four services return a `success` (bool) and `rc` (int32). Get
services additionally return the queried state.
4 个 Service 都返回 `success` (bool) 与 `rc` (int32)。Get 类还会返回查询到的状态字段。

**`GetAe.Response`**

| Field | Typical range / 典型范围 | Meaning / 含义 |
|---|---|---|
| `exposure_time` | `0.0001`~`0.033` s | Current exposure / 当前曝光时间 |
| `gain` | `1.0`~`64.0` | Current analog gain / 当前模拟增益 |
| `iso` | `100`~`6400` | Equivalent ISO / 等效 ISO |
| `brightness` | `0`~`255` | Average frame brightness / 平均帧亮度 |
| `is_converged` | `0` or `1` | `1` = AE settled / AE 已收敛 |
| `env_lv` | `0`~`15` | Ambient luminance index, higher = brighter / 环境光强度指数，越大越亮 |
| `fps` | `~10` / `~14.5` / `~29` | Current frame rate / 当前帧率 |

**`GetAwb.Response`**

| Field | Typical range / 典型范围 | Meaning / 含义 |
|---|---|---|
| `rgain` / `bgain` | `0.1`~`4.0` | R / B channel gain / R / B 通道增益 |
| `grgain` / `gbgain` | `1.0` (fixed / 固定) | Gr / Gb gain, device-fixed / Gr / Gb 增益，设备固定 |
| `cct` | `2500`~`8000` K | Correlated color temperature / 相关色温 |
| `ccri` | `-50`~`50` | Color temp deviation index, 0 = on Planckian locus / 色温偏离指数，0 表示在普朗克轨迹上 |
| `is_converged` | `0` or `1` | `1` = AWB settled / AWB 已收敛 |

#### 4.6.3 `rc` return code / `rc` 返回码

| `rc` | Meaning / 含义 |
|---|---|
| `0` | Success / 成功 |
| `400` | Device payload too short / 设备载荷过短 |
| `401` | Device opcode not supported / 设备不支持该 opcode |
| `402` | Device parameter length wrong / 参数长度错误 |
| `403` | **Parameter out of range** / 参数越界 — most common when manual values exceed the table above / 手动值超出上表范围时最常见 |
| `404` | Device-side socket error / 设备端 socket 错误 |
| `405` | Device-side `ae_control` did not respond / 设备端 `ae_control` 无应答（确认 lydapp 已运行） |
| `255` (`0xFF`) | Unknown opcode reported by ae_control / ae_control 报未知 opcode |
| `-1` | SDK not initialised / SDK 未初始化 |
| `-2` ~ `-5` | USB transfer / timeout / malformed reply / USB 传输异常、超时、应答畸形 |
| `-100` | **Driver has not opened the device yet** / driver 还未打开设备，请等设备连接成功 |

#### 4.6.4 Usage examples / 调用示例

ROS2 (Humble) — start the driver in one terminal, then in a side terminal:
ROS2（Humble）—— 在一个终端启动 driver，在另一个终端：

```bash
source install/setup.bash

# Query current state / 查询当前状态
ros2 service call /odin1/get_ae  odin_ros_driver/srv/GetAe
ros2 service call /odin1/get_awb odin_ros_driver/srv/GetAwb

# Set AE to AUTO / 设置 AE 为自动
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe "{mode: 0}"

# Set AE to MANUAL with 10 ms exposure and gain 4.0
# 设置 AE 为手动，10 毫秒曝光，增益 4.0
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe \
  "{mode: 1, exposure_time: 0.010, gain: 4.0}"

# Set AWB to MANUAL with rgain=1.5, bgain=2.0
# 设置 AWB 为手动，rgain=1.5、bgain=2.0
ros2 service call /odin1/set_awb odin_ros_driver/srv/SetAwb \
  "{mode: 1, rgain: 1.5, bgain: 2.0}"

# Restore AUTO / 一键回自动
ros2 service call /odin1/set_ae  odin_ros_driver/srv/SetAe  "{mode: 0}"
ros2 service call /odin1/set_awb odin_ros_driver/srv/SetAwb "{mode: 0}"

# Inspect srv definition / 查看 srv 完整定义
ros2 interface show odin_ros_driver/srv/SetAe
```

ROS1 (Noetic) — start the driver, then in a side terminal:
ROS1（Noetic）—— 启动 driver 后，新开终端：

```bash
source devel/setup.bash

# Query / 查询
rosservice call /odin1/get_ae
rosservice call /odin1/get_awb

# Set AE manual / 设置 AE 手动
rosservice call /odin1/set_ae  "{mode: 1, exposure_time: 0.010, gain: 4.0}"

# Set AWB manual / 设置 AWB 手动
rosservice call /odin1/set_awb "{mode: 1, rgain: 1.5, bgain: 2.0}"

# Restore AUTO (ROS1 requires all fields to be present)
# 一键回自动（ROS1 要求填齐全部字段）
rosservice call /odin1/set_ae  "{mode: 0, exposure_time: 0.0, gain: 0.0}"
rosservice call /odin1/set_awb "{mode: 0, rgain: 0.0, bgain: 0.0}"

# Inspect srv definition / 查看 srv 完整定义
rossrv show odin_ros_driver/SetAe
```

#### 4.6.5 Recommended starting points by scene / 不同场景推荐起步参数

**AE (`exposure_time`, `gain`)**

| Scene / 场景 | `exposure_time` | `gain` |
|---|---|---|
| Bright outdoor / 明亮室外 | `0.001` ~ `0.005` s | `1.0` ~ `2.0` |
| Normal indoor / 普通室内 | `0.008` ~ `0.015` s | `2.0` ~ `8.0` |
| Dim light / 暗光环境 | `0.020` ~ `0.030` s | `8.0` ~ `32.0` |
| Very dark / 极暗 | `0.033` s | `32.0` ~ `64.0` |

**AWB (`rgain`, `bgain`)**

| Target tone / 目标色调 | `rgain` | `bgain` |
|---|---|---|
| Warm (tungsten, sunset) / 暖（钨丝灯、夕阳） | `2.0` ~ `2.5` | `1.0` ~ `1.2` |
| Neutral (D65 daylight) / 中性（D65 日光） | `1.5` ~ `1.7` | `1.8` ~ `2.0` |
| Cool (cloudy, fluorescent) / 冷（阴天、荧光） | `1.2` ~ `1.4` | `2.2` ~ `2.6` |
| Very cool / 极冷 | `1.0` | `3.0` ~ `4.0` |

#### 4.6.6 Caveats / 注意事项

- The service blocks for up to ~10 s waiting for the device to reply;
  typical latency is tens of milliseconds.
  Service 最长阻塞约 10 秒等设备应答；正常几十毫秒返回。
- Manual mode is **not** persisted across driver / device restart;
  it falls back to AUTO on each new connection.
  手动模式**不会**跨重启保留；每次重连默认回到 AUTO。
- `rc = -100` means the driver has not yet opened the device.
  Wait until the driver logs `device connected` before calling.
  返回 `rc = -100` 表示 driver 还没打开设备，等到 driver 日志显示 `device connected` 再调用。
- The effective maximum `exposure_time` is bounded by the frame
  period `1 / fps`. With `dtof_fps = 290` (29 Hz, period ~34 ms)
  the upper limit 0.033 s is already at the frame boundary.
  最大可用 `exposure_time` 受帧周期 `1/fps` 限制。在 `dtof_fps = 290`（29 Hz、周期 ~34 ms）下，上限 0.033 s 已经贴到帧边界。

## 5. FAQ
### 5.1 Segmentation fault upon re-launching host SDK
**Error Message**  
No device connected after 60 seconds 

**Solution**  
1. Please power on Odin module again # Disconnect and reconnect odin power

2. Reinitialize Odin SDK # Execute SDK after device reboot


### 5.2 Library binding failure during compilation

**Error Message**  
ld: cannot find -llydHostApi or symbol lookup errors

**Resolution** 

1. Clean previous build artifacts

ROS1 
```shell
rm -rf devel/ build/  
```
ROS2
```shell
rm -rf devel/ install/ log/ 
```
2. Re-run script installation

### 5.3 Docker GUI passthrough failure

**Error Message**  
Unable to open X display or No protocol specified

**Resolution** 
```shell
xhost + #This command enables graphical passthrough to Docker containers
```

### 5.4 ROS driver exit with get version failed error

**Error Message**  
```shell
<ERROR><api.cpp:lidar_get_version:672>: get device version fail.
get version failed.
```

**Resolution** 

Device firmware version is too low, please update to latest version.


### 5.5 RVIZ has not responded for a long time

**Error Message**  
Rviz does not respond, and after a while the terminal prints Device disconnected, waiting for reconnection...

**Resolution** 

Please power on Odin module again

### 5.6 Device not responding

**Error Message**  
Missed ok response from device,probably wrong interaction procedure.

**Resolution** 

Please adopt the solution mentioned in 5.1

### 5.7 Device has no external calibration file 

**Error Message**  
ERROR：Missing camera node 'cam_0'

**Resolution** 

Please plug and unplug the USB again

### 5.8 ROS Driver report device disconnected immediately after stream started

**Error Message**  

```shell
Device ready and streams activated
Device detaching...
Wating for device reconnection...
Device disconnected, waiting for reconnection...
```

**Reason**

Mostly common on ros2 environment and connected to complex network environment, such as office wifi & ethernet. ROS2 default to broadcast, and complex network environment will cause ros2 publish to block, leading to device disconnection.

**Resolution** 

If cross-device communication is not required, please restrict ros2 to localhost only with:
```shell
export ROS_LOCALHOST_ONLY=1
```

If cross-device communication is required, please simplify the network environment as much as possible. Mini local network with only required devices is recommended.

### 5.9 ROS Driver died immediately after stream started

**Error Message**  

```shell
Device ready and streams activated
[host_sdk_sample-2] process has died ......
```

**Test**

Disable odin1/image	with sendrgb = 0 in control_command.yaml and try again. If the driver now works, it is likely that the issue is related to multiple version of opencv is installed on the system.

**Resolution** 

Purge the unused version of opencv and maintain a single complete version, then rebuild the driver and try again.

### 5.10 ROS Driver printing "TF_OLD_DATA ignoring data" warning

**Error Message**  

```shell
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame odin1_base_link at time 20.547632 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
```

**Reason**

This is a ros & rviz feature to warn user that some tf data is being ignored due to timestamp conflicts. It happens when user keeps ros driver running and power-cycles odin device, which cause odin's internal system time being reset and now data timestamps conflicts with old data recieved by rviz during last run.

**Resolution** 

There's a reset button on bottom of rviz gui. Click on this button will reset rviz's internal state and stop the warning.

### 5.11 ROS Driver printing "unknown cmd code: xx" error

**Error Message**  

```shell
<ERROR><api.cpp:cmd_data_deal:418>: unknow command code 21.
```

**Reason**

This is due to ros driver version mismatch with device firmware version, resulting in ros driver unable to decode new data added in newer firmware.

**Resolution** 

Please make sure you are using most up-to-date ros driver and device firmware.

### 5.12 USB device access error (LIBUSB_ERROR_BUSY or LIBUSB_ERROR_ACCESS)

**Error Message**  

```shell
libusb: error [udev_hotplug_event] ignoring udev action bind
LIBUSB_ERROR_BUSY
```

or

```shell
libusb: error [_get_usbfs_fd] libusb couldn't open USB device /dev/bus/usb/xxx/xxx, errno=13
LIBUSB_ERROR_ACCESS
```

**Reason**

- **LIBUSB_ERROR_BUSY**: Another process is already using the USB device. This commonly happens when multiple instances of the ROS driver are running, or another application (such as a previous crashed instance) still holds the device handle.

- **LIBUSB_ERROR_ACCESS**: The current user does not have permission to access the USB device. This is typically caused by missing udev rules or insufficient user privileges.

**Resolution** 

For **LIBUSB_ERROR_BUSY**:

1. Check if another instance of the driver is running:
```shell
ps aux | grep host_sdk_sample
```

2. Kill any existing instances:
```shell
killall host_sdk_sample
```

3. If the issue persists, unplug and replug the USB device to reset the device state.

For **LIBUSB_ERROR_ACCESS**:

1. Add udev rules for the device. Create a file `/etc/udev/rules.d/99-odin.rules` with the following content:
```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

2. Reload udev rules:
```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. Alternatively, run the driver with sudo (not recommended for production):
```shell
sudo -E ros2 launch odin_ros_driver odin_ros_driver.launch.py
```

4. Make sure your user is in the `plugdev` group:
```shell
sudo usermod -aG plugdev $USER
```
Then log out and log back in for the group change to take effect.

### 5.13 ros2 bag drops high-frequency topics (IMU / odometry_highfreq) / ros2 bag 录制丢失高频话题（IMU / odometry_highfreq）

**Symptom / 现象**

When recording with `ros2 bag record`, low-frequency topics (cloud, image, odometry, wiwc) are intact, but `/odin1/imu` (400 Hz) and `/odin1/odometry_highfreq` (400 Hz) show missing samples — analysis scripts report inter-message intervals that are 2× or more of the expected period, while no drop is reported on the SDK side or by an online subscriber such as `ros2 topic hz`.

使用 `ros2 bag record` 录制时，低频话题（cloud、image、odometry、wiwc）完整无丢，但 `/odin1/imu`（400 Hz）和 `/odin1/odometry_highfreq`（400 Hz）会出现丢帧——分析脚本上看到消息间隔达到正常周期的 2 倍以上，而 SDK 侧不报丢，独立的 `ros2 topic hz` 订阅者也看不到丢。

**Reason / 原因**

The driver publishes `/odin1/imu` and `/odin1/odometry_highfreq` with `RELIABLE` QoS. By default `ros2 bag record` subscribes with `history = keep_last`, `depth = 10`, which only buffers ~25 ms of samples at 400 Hz. Whenever the recorder is briefly delayed (disk flush, mcap/sqlite chunk write, scheduler jitter), its subscription queue overflows and DDS silently drops the oldest samples on the **subscriber side**. The SDK and publisher are unaffected, which is why no drop appears in the driver logs or in `ros2 topic hz`.

驱动以 `RELIABLE` QoS 发布 `/odin1/imu` 与 `/odin1/odometry_highfreq`。`ros2 bag record` 默认订阅使用 `history = keep_last`、`depth = 10`，在 400 Hz 下只能缓冲约 25 ms。一旦录制端有短暂阻塞（落盘 flush、mcap/sqlite chunk 写入、调度抖动），订阅队列就会溢出，DDS 在**订阅端**静默丢掉最旧的样本。SDK 与 publisher 不受影响，因此驱动日志和 `ros2 topic hz` 都看不到丢。

**Resolution / 解决方案**

Use the provided QoS override file `script/rosbag2_qos.yaml` to raise the subscriber-side queue depth on the recorder for the two high-rate topics:

使用本仓库提供的 QoS 配置 `script/rosbag2_qos.yaml`，把高频话题的录制订阅 depth 拉大：

```yaml
# script/rosbag2_qos.yaml
/odin1/imu:
  reliability: reliable
  history: keep_last
  depth: 4000

/odin1/odometry_highfreq:
  reliability: reliable
  history: keep_last
  depth: 4000
```

Apply it when recording / 录制时通过 `--qos-profile-overrides-path` 应用：

```shell
ros2 bag record -a \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag
```

Or only the high-rate topics / 也可以只录制高频话题：

```shell
ros2 bag record \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag \
    /odin1/imu /odin1/odometry_highfreq /odin1/odometry /odin1/wiwc /odin1/cloud_raw
```

**Optional further tuning / 可选的进一步优化**

If drops still occur after applying the override (typically on slower disks), try the following in addition / 套用上述 override 后仍有丢包时（通常发生在慢盘上），可叠加以下措施：

```shell
# Use mcap backend with a larger internal cache (faster than sqlite3).
# 使用 mcap 后端 + 更大的内部缓存（比 sqlite3 快）。
ros2 bag record -s mcap --max-cache-size 1073741824 \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag \
    /odin1/imu /odin1/odometry_highfreq ...

# Enlarge kernel UDP socket buffers (the most common hidden bottleneck for
# 400 Hz RELIABLE traffic, default is only 208 KB).
# 放大内核 UDP socket buffer（400 Hz RELIABLE 流量最常见的隐藏瓶颈，默认仅 208 KB）。
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432
```

**Does ROS1 have the same problem? / ROS1 是否存在同样的问题？**

No. ROS1 uses TCP-based publish/subscribe with a single `queue_size` parameter on each side, and has no QoS profile mismatch between publisher and subscriber. The ROS1 publisher path in this driver already sizes the IMU and `odometry_highfreq` publishers to `queue_size = 4000` (`include/host_sdk_sample.h`, see `initialize_publishers` ROS1 branch), and `rosbag record` uses TCP transport which is reliable by construction. As a result this specific drop pattern does not occur under ROS1; no additional configuration is required.

不存在。ROS1 使用基于 TCP 的发布/订阅，发布端与订阅端各自只有一个 `queue_size` 参数，不存在 ROS2 那种 QoS profile 不匹配的问题。本驱动 ROS1 路径已经把 IMU 与 `odometry_highfreq` 的发布队列设置为 `queue_size = 4000`（见 `include/host_sdk_sample.h` 中 `initialize_publishers` 的 ROS1 分支），并且 `rosbag record` 使用 TCP 传输本身即可靠传递。因此在 ROS1 下不会出现该丢帧现象，也不需要额外配置。

### 5.14 On Ubuntu 24.04 (ROS2 Jazzy) point clouds / TF fail to transform in RViz / Ubuntu 24.04（ROS2 Jazzy）下 RViz 点云 / TF 无法转换

**Symptom / 现象**

On Ubuntu 24.04 (ROS2 Jazzy), when the RViz **Fixed Frame** is set to `odom`, `cloud_raw` (in the `lidar` frame) fails to display and RViz reports a transform / extrapolation error for the `odom -> imu -> lidar` chain. On Ubuntu 22.04 (Humble) the same setup works.

在 Ubuntu 24.04（ROS2 Jazzy）下，当 RViz 的 **Fixed Frame** 设为 `odom` 时，`cloud_raw`（`lidar` 系）无法显示，RViz 对 `odom -> imu -> lidar` 链条报 transform / 外推（extrapolation）错误。相同配置在 Ubuntu 22.04（Humble）下却正常。

**Reason / 原因**

The device point-cloud timestamp is typically ~100 ms ahead of the odometry timestamp. ROS2 Jazzy's tf2 performs a strict timestamp lookup and will not extrapolate, so when the TF is published only at the odometry rate (and only at the odometry timestamp), tf2 cannot find a transform bracketing the cloud timestamp and the lookup fails.

设备点云时间戳通常比里程计时间戳超前约 100 ms。ROS2 Jazzy 的 tf2 采用严格的时间戳查找且不做外推（extrapolation），因此当 TF 只在里程计频率、且只按里程计时间戳发布时，tf2 找不到能覆盖点云时间戳的变换，查找失败。

**Resolution / 解决方案**

Set `tf_extra_publish_rate` to a value greater than `0` (e.g. `100`) in `config/control_command.yaml`. This starts an extra timer that keeps re-publishing the `odom -> imu` and `imu -> lidar` transforms at the configured rate with advancing timestamps, so tf2 always has a transform covering the cloud timestamp.

在 `config/control_command.yaml` 中把 `tf_extra_publish_rate` 设为大于 `0` 的值（例如 `100`）。它会启动一个额外的定时器，按设定频率、以推进的时间戳持续补发 `odom -> imu` 与 `imu -> lidar` 变换，使 tf2 始终有覆盖点云时间戳的变换。

```yaml
# config/control_command.yaml
tf_extra_publish_rate: 100  # 0: off; >0: rate in Hz (recommended on ROS2 Jazzy / Ubuntu 24.04)
```

On Ubuntu 22.04 (Humble) or ROS1 this is usually not required and the parameter can be left at `0`.

在 Ubuntu 22.04（Humble）或 ROS1 下通常不需要该配置，参数可保持为 `0`。

### 5.15 USB "transfer no mem" / "transfer error" on low-performance hosts / 低性能主机上出现 "transfer no mem" / "transfer error"

**Symptom / 现象**

```shell
<ERROR>: submit x transfer fail LIBUSB_ERROR_NO_MEM.
<ERROR>: transfer error: LIBUSB_TRANSFER_ERROR.
```

The driver runs fine on a powerful PC but, when deployed on a lower-performance board (e.g. an embedded ARM host), after a while the SDK starts reporting `LIBUSB_ERROR_NO_MEM` on send, followed by a cascade of `transfer error`. The module's sending rate is constant; the only thing that changed is the host the SDK runs on.

驱动在性能强的 PC 上运行正常，但部署到性能较弱的板子（例如嵌入式 ARM 主机）上跑一段时间后，SDK 发送时开始报 `LIBUSB_ERROR_NO_MEM`，随后接连出现 `transfer error`。模组的发送速率是恒定的，唯一改变的条件是 SDK 运行在哪台机器上。

**Reason / 原因**

libusb async transfers queued to the kernel consume a limited per-device usbfs memory pool (`/sys/module/usbcore/parameters/usbfs_memory_mb`, default **16 MB**). On a slow host the single libusb event thread reaps completed transfers more slowly than new ones are submitted, so outstanding OUT (send) transfers pile up in the usbfs pool until submission fails with `LIBUSB_ERROR_NO_MEM`. Once submission starts failing, subsequent transfers report `LIBUSB_TRANSFER_ERROR`.

libusb 异步传输提交到内核后，会占用每个设备有限的 usbfs 内存池（`/sys/module/usbcore/parameters/usbfs_memory_mb`，默认 **16 MB**）。在慢主机上，单一的 libusb 事件线程回收已完成传输的速度跟不上新传输的提交速度，于是在途的 OUT（发送）传输在 usbfs 池中不断堆积，直到提交失败并返回 `LIBUSB_ERROR_NO_MEM`。一旦提交开始失败，后续传输就会报 `LIBUSB_TRANSFER_ERROR`。

**Resolution / 解决方案**

1. Raise the usbfs memory limit at runtime (takes effect immediately, resets on reboot) / 在运行时调大 usbfs 内存上限（立即生效，重启后失效）：

```shell
# Check current value / 查看当前值 (default 16)
cat /sys/module/usbcore/parameters/usbfs_memory_mb

# Raise to 128 MB / 调大到 128 MB
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

2. Make it persistent across reboots / 让配置在重启后仍生效（二选一）：

```shell
# Option A: kernel boot parameter (edit GRUB / board bootargs)
# 方式 A：内核启动参数（编辑 GRUB / 板子的 bootargs）
usbcore.usbfs_memory_mb=128

# Option B: modprobe config
# 方式 B：modprobe 配置
echo "options usbcore usbfs_memory_mb=128" | sudo tee /etc/modprobe.d/usbcore.conf
```

**Note / 说明**

Raising `usbfs_memory_mb` is a mitigation that increases headroom. The SDK USB layer already bounds the number of in-flight OUT transfers (flow control) and releases transfer buffers on every terminal state to prevent the memory leaks that previously turned a transient error into a runaway. Keeping a larger usbfs pool (e.g. 128 MB) is still recommended on slow hosts or when transferring large files (e.g. relocalization maps).

调大 `usbfs_memory_mb` 属于提高余量的缓解措施。SDK 的 USB 层已经对在途 OUT 传输数量做了上限限制（流控），并在每个终止状态释放传输缓冲，避免了此前"瞬时错误演变为不可恢复的内存泄漏雪崩"的问题。在慢主机上或传输大文件（如重定位地图）时，仍建议保留较大的 usbfs 池（例如 128 MB）。

### 5.16 Driver crashes when publishing RGB due to cv_bridge / OpenCV version mismatch (without touching the system) / 出图时因 cv_bridge 与 OpenCV 版本不匹配崩溃（不动系统的解法）

**Symptom / 现象**

The driver connects and activates streams, but crashes (segfault / heap corruption) as soon as the first RGB frame is published. This is the same root cause as 5.9, seen when the host has **two OpenCV versions** installed: the ROS `cv_bridge` (from `/opt/ros/<distro>`) was compiled against one OpenCV (e.g. 4.2.0), while the driver links a different one (e.g. 4.5.4).

驱动能连上、数据流也激活，但一发布第一帧 RGB 就崩溃（段错误 / 堆损坏）。这与 5.9 是同一根因，出现在主机装了**两个 OpenCV 版本**时：ROS 的 `cv_bridge`（来自 `/opt/ros/<distro>`）是针对某个 OpenCV（如 4.2.0）编译的，而驱动链接了另一个（如 4.5.4）。

**Reason / 原因**

With `sendrgb: 1`, the host decodes RGB with `cv::imdecode(...)` and publishes via `cv_bridge::toImageMsg()`. If the driver's OpenCV differs from `cv_bridge`'s, the same process loads two `libopencv_core` copies; a `cv::Mat` created by one and released by the other crosses an ABI boundary and corrupts the heap → crash on the first RGB frame.

开启 `sendrgb: 1` 时，主机用 `cv::imdecode(...)` 解码 RGB，再经 `cv_bridge::toImageMsg()` 发布。若驱动的 OpenCV 与 `cv_bridge` 的不一致，同一进程会加载两份 `libopencv_core`；一个 `cv::Mat` 由其中一方创建、另一方释放，跨越 ABI 边界导致堆损坏 → 第一帧 RGB 到来时崩溃。

**Which ROS distros are affected? / 哪些 ROS 版本会受影响？**

This is **not** specific to any one ROS version. The crash happens whenever `cv_bridge`'s OpenCV ≠ the driver's OpenCV, regardless of ROS1 or ROS2. In practice, the most common trigger is on **Jetson / embedded platforms** where NVIDIA preinstalls a different OpenCV than the one ROS apt packages were compiled against.

这**不是**某个 ROS 版本独有的问题。只要 `cv_bridge` 的 OpenCV ≠ 驱动的 OpenCV，无论 ROS1 还是 ROS2 都会崩。实际中最常见的触发场景是 **Jetson / 嵌入式平台**，NVIDIA 预装的 OpenCV 与 ROS apt 仓库编译 `cv_bridge` 时用的版本不一致。

| ROS Distro | Type | vision_opencv branch | Typical system OpenCV |
|---|---|---|---|
| Kinetic (Ubuntu 16.04) | ROS1 | `kinetic` | 3.3.1 |
| Melodic (Ubuntu 18.04) | ROS1 | `melodic` | 3.2 / 3.4 |
| Noetic (Ubuntu 20.04) | ROS1 | `noetic` | 4.2.0 (apt) / 4.5.4 (Jetson) |
| Foxy (Ubuntu 20.04) | ROS2 | `foxy` | 4.2.0 |
| Galactic (Ubuntu 20.04) | ROS2 | `galactic` | 4.2.0 |
| Humble (Ubuntu 22.04) | ROS2 | `humble` | 4.5.x |
| Iron (Ubuntu 22.04) | ROS2 | `iron` | 4.5.x |
| Jazzy (Ubuntu 24.04) | ROS2 | `rolling` | 4.5.x / 4.6.x |

> If your system has only one OpenCV and it matches `cv_bridge`'s, you will **not** hit this bug — the driver builds and runs out of the box.
> 如果系统只有一个 OpenCV 且与 `cv_bridge` 一致，你**不会**遇到此问题——驱动开箱即编即跑。

**Resolution / 解决方案**

5.9 suggests purging the extra OpenCV. If you must keep multiple OpenCV versions on the system, the alternative below **rebuilds `cv_bridge` inside the workspace** against the OpenCV the driver uses, so the driver and `cv_bridge` share one OpenCV. All artifacts go into the workspace only — **the system `/opt/ros` and system OpenCV are never modified**.

5.9 的做法是卸载多余的 OpenCV。如果你必须在系统上保留多个 OpenCV 版本，可用下面的替代方案：**在工作区内基于驱动所用的 OpenCV 重新编译 `cv_bridge`**，让驱动与 `cv_bridge` 使用同一份 OpenCV。所有产物只落在工作区内，**系统 `/opt/ros` 与系统 OpenCV 完全不会被改动**。

The driver's `CMakeLists.txt` reads the OpenCV version recorded in `cv_bridge-extras.cmake` and pins the driver to that exact version. It prefers a `cv_bridge` from the current workspace / a sourced overlay over the one in `/opt/ros`. If no `cv_bridge` is provided by the workspace and the exact version is unavailable, it falls back to the system default OpenCV with a warning (so a self-consistent single-OpenCV machine still builds out of the box).

驱动的 `CMakeLists.txt` 会读取 `cv_bridge-extras.cmake` 里记录的 OpenCV 版本，并把驱动锁定到该版本；查找时优先使用当前工作区 / 已 source 的 overlay 里的 `cv_bridge`，其次才是 `/opt/ros`。若工作区未提供 `cv_bridge` 且系统缺该精确版本，则回退到系统默认 OpenCV 并给出警告（因此单一、一致 OpenCV 的机器仍可开箱即编）。

**Step 0 — Identify your ROS distro / 步骤零 —— 确认你的 ROS 发行版**

```shell
echo $ROS_DISTRO
# Example output: noetic  (ROS1)
# Example output: foxy    (ROS2)
```

Use the matching branch from the table above for all commands below. The examples use `$ROS_DISTRO` as a placeholder — replace it (or export it) with your actual distro name.

以下命令中用 `$ROS_DISTRO` 作为占位符，请替换为你的实际发行版名称（或直接 `export ROS_DISTRO=<你的发行版>`）。

**Step 1 — Download vision_opencv (matching your ROS distro branch) / 步骤一 —— 下载 vision_opencv（分支要与 ROS 发行版对应）**

Clone it into the workspace `src/` next to `odin_ros_driver`. Use the branch matching your ROS distro from the table above.

克隆到工作区 `src/` 下、与 `odin_ros_driver` 并列。分支要与你的 ROS 发行版对应（见上方表格）。

```shell
cd <your_workspace>/src
git clone -b $ROS_DISTRO https://github.com/ros-perception/vision_opencv.git
```

> If `git clone -b $ROS_DISTRO` fails, your distro name may differ from the branch name (e.g. Jazzy → `rolling`). Check the table above and substitute the correct branch.
> 如果 `git clone -b $ROS_DISTRO` 失败，可能是发行版名与分支名不一致（如 Jazzy → `rolling`）。请对照上方表格替换为正确的分支名。

**Step 2 — Build cv_bridge FIRST, then build the driver / 步骤二 —— 先编 cv_bridge，再编驱动**

**Order matters: `cv_bridge` must be built into the workspace before the driver.** The driver's `CMakeLists.txt` reads the workspace `cv_bridge-extras.cmake` at configure time to pin its OpenCV; if `cv_bridge` is not in the workspace yet, the driver only sees the system `cv_bridge` and links a mismatched OpenCV.

**顺序很重要：必须先把 `cv_bridge` 编进工作区，再编驱动。** 驱动的 `CMakeLists.txt` 在配置阶段读取工作区的 `cv_bridge-extras.cmake` 来锁定 OpenCV；若此时工作区里还没有 `cv_bridge`，驱动只会看到系统 `cv_bridge`，从而链接到不匹配的 OpenCV。

**ROS1 (catkin_make):**

```shell
# (1) Build cv_bridge into the workspace FIRST.
#     先把 cv_bridge 编进工作区。
source /opt/ros/$ROS_DISTRO/setup.bash
cd <your_catkin_ws>
catkin_make -DBUILD_SYSTEM=ROS1 -DCATKIN_WHITELIST_PACKAGES="cv_bridge" -j$(nproc)

# (2) Then build the driver. It now finds the workspace cv_bridge and pins the
#     driver's OpenCV to the exact same version (do NOT clean build/devel here).
#     再编驱动。它会找到工作区的 cv_bridge 并把驱动 OpenCV 锁到同一版本（此步不要清 build/devel）。
cd <your_catkin_ws>/src/odin_ros_driver/script
./build_ros.sh
```

> **Simplest option / 最简做法**: a single full build compiles everything in the correct dependency order automatically — `catkin_make` builds `cv_bridge` (a dependency) before the driver, so you don't have to think about ordering:
> 一条全量编译命令会**按依赖顺序自动**先编 `cv_bridge`、再编驱动，无需关心顺序：
> ```shell
> cd <your_catkin_ws> && catkin_make -DBUILD_SYSTEM=ROS1 -j$(nproc)
> ```

**ROS2 (colcon):**

```shell
# (1) Build cv_bridge into the workspace FIRST.
#     先把 cv_bridge 编进工作区。
source /opt/ros/$ROS_DISTRO/setup.bash
cd <your_ros2_ws>
colcon build --packages-select cv_bridge

# (2) Then build the driver. Source the overlay so the driver finds the workspace cv_bridge.
#     再编驱动。先 source overlay，让驱动找到工作区的 cv_bridge。
source <your_ros2_ws>/install/setup.bash
colcon build --packages-select odin_ros_driver
```

**Verification / 验证**

```shell
# ROS1
source /opt/ros/$ROS_DISTRO/setup.bash
source <your_catkin_ws>/devel/setup.bash
ldd <your_catkin_ws>/devel/lib/odin_ros_driver/host_sdk_sample | grep -iE 'cv_bridge|opencv_core'

# ROS2
source /opt/ros/$ROS_DISTRO/setup.bash
source <your_ros2_ws>/install/setup.bash
ldd <your_ros2_ws>/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample | grep -iE 'cv_bridge|opencv_core'
```

- Before Step 2 (crash) / 步骤二之前（会崩）：`libcv_bridge.so => /opt/ros/$ROS_DISTRO/lib/...`, and a mismatched `libopencv_core.so.<old>` is mixed in / 且混入不匹配的 `libopencv_core.so.<旧版本>`。
- After Step 2 (fixed) / 步骤二之后（已修复）：`libcv_bridge.so => <your_workspace>/...`, and all `libopencv_*` are a single consistent version / 且所有 `libopencv_*` 为同一个版本。

Then run the driver and confirm `/odin1/image` publishes steadily without crashing.

随后运行驱动，确认 `/odin1/image` 稳定发布且不再崩溃。

> Important / 重要: run in a clean ROS environment; do not mix another ROS distro into `LD_LIBRARY_PATH` (e.g. don't source both noetic and foxy), or `cv_bridge` may resolve back to a mismatched OpenCV.
> 请在干净的 ROS 环境中运行；不要把另一个 ROS 发行版混入 `LD_LIBRARY_PATH`（如同时 source noetic 和 foxy），否则 `cv_bridge` 可能又解析回不匹配的 OpenCV。

## 6.  Contact Information​​

You can contact our support through support@manifoldtech.cn

To help diagnose the issue, please provide the following details to our FAE engineer:

1. Current firmware version​​ 
```shell
[device_version_capture]: ros_driver_version: [Version Number]
```
2. Photos of power adapter and converter cable​​ in use.

3. Does the issue happen occasionally or consistently?

4. Provide images of the problem scenario.

5. Did the troubleshooting methods in ​​Section V​​ resolve the issue?

6. Expected timeline for issue resolution.
