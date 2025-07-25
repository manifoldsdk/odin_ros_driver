# Odin_ROS_Driver Readme

ROS driver suite for Odin sensor modules (Manifold Tech Ltd.) 

## Odin_ROS_Driver

Compatibility:

● ROS 1(LTS Release: Noetic recommended)

● ROS 2(LTS Release: Humble recommended)

## Important Notice:

This driver package provides core functionality for point cloud SLAM applications and targets specific use cases. It is intended exclusively for technical professionals conducting secondary development. End users must perform scenario-specific optimization and custom development to align with operational requirements in practical deployment environments.

## 1. Version

Current Version: v0.2

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
source [ros_workspace]/install/setup.bash
ros2 launch odin_ros_driver [launch file]
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
## 4. File structure and data format
### 4.1 File structure
```shell
Odin_ROS_Driver/                // ROS1/ROS2 driver package
    3rdparty/                   // Third-party libraries
    src/
        host_sdk_sample.cpp     // Example source code
        yaml_parser.cpp         // Source code for reading yaml parameters
        rawCloudRender.cpp      // Source code for RenderCloud
    lib/
        liblydHostApi_amd.a     // Static library for AMD platform
        liblydHostApi_arm.a     // Static library for ARM platform
    include/
        host_sdk_sample.h       // Example header file
        lidar_api_type.h        // API data structure header file
        lidar_api.h             // API function declarations
        yaml_parser.h           // Parameter file reading header file
        rawCloudRender.h        // API about RenderCloud
    config/
        control_command.yaml    // Control parameter file for driver
        calib.yaml              //Machine calibration yaml
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


### 4.3 ROS topics
Internal parameters of the Odin ROS driver are defined in config/control_command.yaml. Below are descriptions of the commonly used parameters:

| Topic               | Detailed Description |
|---------------------|----------------------|
| odin1/imu           | Imu Topic |
| odin1/image         | RGB Camera Topic |
| odin1/image/compressed         | RGB Camera compressed Topic |
| odin1/cloud_raw     | Raw_Cloud Topic |
| odin1/cloud_render     | Render_Cloud Topic |
| odin1/cloud_slam    | Slam_PointCloud Topic |
| odin1/odometry_map  | Odom Topic |

## 5. FAQ
### 5.1 Segmentation fault upon re-launching host SDK
**Error Message**  
No device connected after 60 seconds 

**Solution**  
1.Please power on Odin module again # Disconnect and reconnect odin power

2.Reinitialize Odin SDK # Execute SDK after device reboot


### 5.2 Library binding failure during compilation

**Error Message**  
ld: cannot find -llydHostApi or symbol lookup errors

**Resolution** 

1.Clean previous build artifacts

ROS1 
```shell
rm -rf devel/ build/  
``` 
ROS2
```shell
rm -rf devel/ install/ log/ 
``` 
2.Re-run script installation

### 5.3 Docker GUI passthrough failure

**Error Message**  
Unable to open X display or No protocol specified

**Resolution** 
```shell
xhost + #This command enables graphical passthrough to Docker containers
``` 

### 5.4 RVIZ has not responded for a long time

**Error Message**  
Rviz does not respond, and after a while the terminal prints Device disconnected, waiting for reconnection...

**Resolution** 

Please power on Odin module again

### 5.5 Device not responding

**Error Message**  
Missed ok response from device,probably wrong interaction procedure.

**Resolution** 

Please adopt the solution mentioned in 5.1

### 5.6 Device has no external calibration file 

**Error Message**  
ERROR：Missing camera node 'cam_0'

**Resolution** 

Please plug and unplug the USB again