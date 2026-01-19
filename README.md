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

Current version: v0.8.0

Required device firmware version: v0.9.0

## 2. Preparation

### 2.1 OS Requirement

● Ubuntu 20.04 for ROS Noetic and ROS2 Foxy;

● Ubuntu 22.04 for ROS2 Humble;

● Ubuntu 18.04 is currently not supported;

● Ubuntu 24.04 is not officially supported but may work with some modifications.

### 2.2 Dependencies

● Opencv >= 4.5.0(recommand 4.5.5/4.8.0. Make sure only one version of opencv is installed)

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
| strict_usb3.0_check   | Strict USB3.0 check, if off, allow connection even if usb connection is below usb 3.0 |
| recorddata            | Record data in specific format that can be imported into MindCloud(TM) for post-processing. Please be aware that this will consume a lot of storage space. Testing shows 9.5G for 10mins of data. |
| devstatuslog          | Device status logging, currently save device status (soc temperature, cpu usage, ram usage, dtof sensor temp .etc) and data tx & rx rate to devstatus.csv under log folder. A new file will be created every time the driver is started. |
| showcamerapose        | Display Camera Pose and Field of View. |
| custom_map_mode        | Operation Modes: Mode 0 - Odometry mode: The map frame and odom frame share the same pose. Mode 1 - Mapping (with loop closure) mode: This mode supports map saving. Mode 2 - Relocalization mode: Requires specifying the absolute path to the map file. After successful relocalization, it will output the TF relationship between the map and odom frames.|
| custom_init_pos        | Initialization Position (currently unused). |
| relocalization_map_abs_path        | Absolute Path to Map File: Used for relocalization mode. |
| mapping_result_dest_dir and mapping_result_file_name| Path and Name for Saving Maps in Mapping Mode: If not specified, default values will be used. |

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

## 6.  Contact Information​​

You can contact our support through support@manifoldtech.cn

To help diagnose the issue, please provide the following details to our FAE engineer:

1. ​Current firmware version​​ 
```shell
[device_version_capture]: ros_driver_version: [Version Number]
``` 
2. ​Photos of power adapter and converter cable​​ in use.

3. Does the issue happen occasionally or consistently?

4. Provide images of the problem scenario.

5. Did the troubleshooting methods in ​​Section V​​ resolve the issue?

6. Expected timeline for issue resolution.
