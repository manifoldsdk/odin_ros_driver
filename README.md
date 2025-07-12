# Odin_ROS_Driver Readme

ROS driver suite for Odin sensor modules (Manifold Tech Ltd.) 

## Odin_ROS_Driver

Compatibility:

● ROS 1(LTS Release: Noetic recommended)

● ROS 2(LTS Release: Humble recommended)

## Important Notice:

This driver package provides core functionality for point cloud SLAM applications and targets specific use cases. It is intended exclusively for technical professionals conducting secondary development. End users must perform scenario-specific optimization and custom development to align with operational requirements in practical deployment environments.

## 1. Version

Current Version: v0.1

## 2. Preparation

### 2.1 OS Requirement

● Ubuntu 18.04 for ROS Melodic;

● Ubuntu 20.04 for ROS Noetic and ROS2 Foxy;

● Ubuntu 22.04 for ROS2 Humble;

### 2.2 Dependencies

● Opencv

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
For ROS Melodic installation, please refer to:
[ROS Melodic installation instructions](https://wiki.ros.org/melodic/Installation)

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
Please clone the source code into the "[workspace]/src/" folder, otherwise compilation errors will occur.

### 3.3 make

#### 3.3.1 ROS1 (Noetic for example):

```shell
source /opt/ros/noetic/setup.sh
./scripty/build_ros.sh
```

#### 3.3.2 ROS2 (Foxy for example):

```shell
source /opt/ros/foxy/setup.sh
./scripty/build_ros2.sh
```

### 3.4 run:

#### 3.4.1 ROS1 (Noetic for example):

```shell
source ../../devel/setup.sh
roslaunch odin_ros_driver [launch file]
```
● odin_ros_driver: package name;

● launch file: launch file;

ROS1 Demo Launch Instructions:
```shell
roslaunch odin_ros_driver odin1_ros1.launch
```
#### 3.4.2 ROS2 (Foxy for example):

```shell
source ../../install/setup.sh
ros2 launch odin_ros_driver [launch file]
```
● odin_ros_driver: package name;

● launch file: launch file;

ROS2 Demo Launch Instructions:
```shell
ros2 launch odin_ros_driver odin1_ros2.launch.py
```
## 4. File structure and data format
### 4.1 File structure
```shell
Odin_ROS_Driver/                // ROS1/ROS2 driver package
    3rdparty/                   // Third-party libraries
    src/
        host_sdk_sample.cpp     // Example source code
        yaml_parser.cpp         // Source code for reading yaml parameters
    lib/
        liblydHostApi_amd.a     // Static library for AMD platform
        liblydHostApi_arm.a     // Static library for ARM platform
    include/
        host_sdk_sample.h       // Example header file
        lidar_api_type.h        // API data structure header file
        lidar_api.h             // API function declarations
        yaml_parser.h           // Parameter file reading header file
    config/
        control_command.yaml    // control parameter file for driver
    launch_ROS1/
        odin1_ros1.launch       // ROS1 launch file
    launch_ROS2/
        odin1_ros2.launch.py    // ROS2 launch file
    script/
        build_ros1.sh           // Installation script for ROS1
        build_ros2.sh           // Installation script for ROS2
    README.md                   // Usage instructions
    CMakeLists.txt              // CMake build file
    License                     // License file
```
### 4.2 File structure
| Launch File Name         | Description |
|--------------------------|-------------|
| odin1_ros1.launch        | Launch file for ROS1 - Odin1 Basic Operations Demo |
| odin1_ros2.launch.py     | Launch file for ROS2 - Odin1 Basic Operations Demo |

### 4.3 Config file
Internal parameters of the Odin ROS driver are defined in config/control_command.yaml. Below are descriptions of the commonly used parameters:
| Parameter     | Detailed Description | Default |
|---------------|----------------------|---------|
| streamctrl: 1 | Master data stream control (0: OFF, 1: ON) | 1 |
| sendrgb:1     | RGB image stream control (0: OFF, 1: ON) | 1 |
| sendimu:1     | IMU (Inertial Measurement Unit) stream control (0: OFF, 1: ON) | 1 |
| sendodom:1    | Odometry data stream control (0: OFF, 1: ON) | 1 |
| senddtof:0    | Raw PointCloud sensor stream control (0: OFF, 1: ON) | 0 |
| sendcloudslam:1 | Slam PointCloud stream control (0: OFF, 1: ON) | 1 |

### 4.4 ROS topics
Internal parameters of the Odin ROS driver are defined in config/control_command.yaml. Below are descriptions of the commonly used parameters:

| Topic               | Detailed Description |
|---------------------|----------------------|
| odin1/imu           | Imu Topic |
| odin1/image         | RGB Camera Topic |
| odin1/cloud_raw     | Raw_Cloud Topic |
| odin1/cloud_slam    | Slam_PointCloud Topic |
| odin1/odometry_map  | Odom Topic |

## 5. FAQ
### 5.1 Segmentation fault upon re-launching host SDK
**Error Message**  
Core dump occurs when restarting host SDK after initial successful run

**Solution**  
```shell
power cycle Odin1 # Disconnect and reconnect LiDAR power
reinitialize host SDK # Execute SDK after device reboot
```

### 5.2 Library binding failure during compilation

**Error Message**  
ld: cannot find -llydHostApi or symbol lookup errors

**Resolution** 
```shell
Clean previous build artifacts
ROS1 rm -rf devel/ build/
ROS2 rm -rf devel/ install/ log/
Re-run script installation (refer to section 2.3)
```

### 5.3 Docker GUI passthrough failure

**Error Message**  
Unable to open X display or No protocol specified

**Resolution** 
```shell
xhost + #This command enables graphical passthrough to Docker containers
``` 