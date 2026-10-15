# Relocalization Guide / 重定位使用指南

This guide explains how to use the relocalization feature in Odin ROS Driver, including automatic relocalization and init position relocalization modes.

本指南介绍如何使用 Odin ROS Driver 的重定位功能，包括自动重定位和指定初始位置重定位两种模式。

---

## Table of Contents / 目录

1. [Overview / 概述](#overview--概述)
2. [Prerequisites / 前提条件](#prerequisites--前提条件)
3. [Mode 1: Auto Relocalization / 自动重定位](#mode-1-auto-relocalization--自动重定位)
4. [Mode 2: Init Position Relocalization / 指定初始位置重定位](#mode-2-init-position-relocalization--指定初始位置重定位)
5. [init_pos Format / init_pos 格式说明](#init_pos-format--init_pos-格式说明)
6. [Configuration Examples / 配置示例](#configuration-examples--配置示例)
7. [Programmatic API / 编程接口](#programmatic-api--编程接口)
8. [Troubleshooting / 故障排除](#troubleshooting--故障排除)

---

## Overview / 概述

### English

Relocalization mode (`custom_map_mode: 2`) allows Odin to localize itself within a pre-built map. There are two approaches:

| Mode | Description | Use Case |
|------|-------------|----------|
| **Auto Relocalization** | Algorithm automatically searches for position in the map | Starting position is unknown or within recommended range |
| **Init Position Relocalization** | User provides an initial pose estimate via `init_pos` | Starting position is known, faster convergence needed |

### 中文

重定位模式（`custom_map_mode: 2`）允许 Odin 在预先构建的地图中进行自我定位。有两种方式：

| 模式 | 描述 | 适用场景 |
|------|------|----------|
| **自动重定位** | 算法自动在地图中搜索位置 | 起始位置未知，或在推荐范围内 |
| **指定初始位置重定位** | 用户通过 `init_pos` 提供初始位姿估计 | 起始位置已知，需要更快收敛 |

---

## Prerequisites / 前提条件

### English

1. **Pre-built map file**: A `.bin` map file created in SLAM mode (`custom_map_mode: 1`)
2. **Map file path**: Know the absolute path to your map file
3. **Starting position**: For init position mode, know the approximate starting pose in map coordinates

### 中文

1. **预构建的地图文件**：在 SLAM 模式（`custom_map_mode: 1`）下创建的 `.bin` 地图文件
2. **地图文件路径**：知道地图文件的绝对路径
3. **起始位置**：对于指定初始位置模式，需要知道在地图坐标系中的大致起始位姿

---

## Mode 1: Auto Relocalization / 自动重定位

### English

In auto relocalization mode, the algorithm automatically searches for the device's position within the map based on current sensor observations.

**Configuration** (`config/control_command.yaml`):

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/path/to/your/map.bin"
  # custom_init_pos is NOT set or uses default [0,0,0,0,0,0,1]
```

**Recommended Starting Conditions**:
- Within **1 meter** of a position on the original SLAM trajectory
- Within **±10 degrees** of the original orientation
- In a visually distinctive area of the map

**Behavior**:
1. On startup, Odin attempts to match current observations with the map
2. If successful, TF between `map` and `odom` frames is published
3. If unsuccessful, system operates in fallback SLAM mode (map saving disabled)
4. Relocalization attempts continue in background until successful

**Tips**:
- Gently shaking or moving the device after startup can improve relocalization accuracy
- Highly distinctive scenes may allow successful matching beyond the 1m/10° range

### 中文

在自动重定位模式下，算法根据当前传感器观测自动在地图中搜索设备位置。

**配置** (`config/control_command.yaml`):

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/path/to/your/map.bin"
  # custom_init_pos 不设置或使用默认值 [0,0,0,0,0,0,1]
```

**推荐起始条件**：
- 距离原始 SLAM 轨迹上某点 **1 米**以内
- 朝向与原始方向偏差在 **±10 度**以内
- 位于地图中视觉特征明显的区域

**行为**：
1. 启动时，Odin 尝试将当前观测与地图匹配
2. 如果成功，发布 `map` 和 `odom` 坐标系之间的 TF
3. 如果失败，系统进入后备 SLAM 模式（地图保存功能禁用）
4. 后台持续尝试重定位直到成功

**提示**：
- 启动后轻轻晃动或移动设备可以提高重定位精度
- 在特征明显的场景中，可能在超出 1m/10° 范围时也能成功匹配

---

## Mode 2: Init Position Relocalization / 指定初始位置重定位

### English

In init position relocalization mode, you provide an initial pose estimate to help the algorithm converge faster.

**Configuration** (`config/control_command.yaml`):

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/path/to/your/map.bin"
  custom_init_pos: [x, y, z, qx, qy, qz, qw]
```

**When to Use**:
- You know the approximate starting position (e.g., from external localization system)
- Starting position is far from the recommended 1m/10° range
- You need faster relocalization convergence
- Deploying in a fixed docking station with known pose

**Behavior**:
1. Algorithm uses provided `init_pos` as initial pose estimate
2. Searches for matches in the vicinity of the provided position
3. Faster convergence compared to auto mode when estimate is accurate

### 中文

在指定初始位置重定位模式下，您提供初始位姿估计以帮助算法更快收敛。

**配置** (`config/control_command.yaml`):

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/path/to/your/map.bin"
  custom_init_pos: [x, y, z, qx, qy, qz, qw]
```

**适用场景**：
- 您知道大致的起始位置（例如，来自外部定位系统）
- 起始位置远离推荐的 1m/10° 范围
- 需要更快的重定位收敛速度
- 部署在已知位姿的固定充电桩

**行为**：
1. 算法使用提供的 `init_pos` 作为初始位姿估计
2. 在提供位置的附近搜索匹配
3. 当估计准确时，比自动模式收敛更快

---

## init_pos Format / init_pos 格式说明

### English

`init_pos` is an array of **7 float values** representing position and orientation:

```yaml
custom_init_pos: [x, y, z, qx, qy, qz, qw]
```

| Index | Parameter | Description | Unit |
|-------|-----------|-------------|------|
| 0 | x | X position in map frame | meters |
| 1 | y | Y position in map frame | meters |
| 2 | z | Z position in map frame | meters |
| 3 | qx | Quaternion X component | - |
| 4 | qy | Quaternion Y component | - |
| 5 | qz | Quaternion Z component | - |
| 6 | qw | Quaternion W component | - |

**Important Notes**:
- The quaternion must be normalized: `sqrt(qx² + qy² + qz² + qw²) ≈ 1.0`
- Coordinates are relative to the **map frame** (world frame at SLAM start)
- Default value `[0, 0, 0, 0, 0, 0, 1]` represents origin with no rotation

**Common Quaternion Values**:

| Orientation | qx | qy | qz | qw |
|-------------|----|----|----|----|
| No rotation (identity) | 0 | 0 | 0 | 1 |
| 90° around Z-axis | 0 | 0 | 0.707 | 0.707 |
| 180° around Z-axis | 0 | 0 | 1 | 0 |
| -90° around Z-axis | 0 | 0 | -0.707 | 0.707 |

### 中文

`init_pos` 是一个包含 **7 个 float 值**的数组，表示位置和朝向：

```yaml
custom_init_pos: [x, y, z, qx, qy, qz, qw]
```

| 索引 | 参数 | 描述 | 单位 |
|------|------|------|------|
| 0 | x | 地图坐标系中的 X 位置 | 米 |
| 1 | y | 地图坐标系中的 Y 位置 | 米 |
| 2 | z | 地图坐标系中的 Z 位置 | 米 |
| 3 | qx | 四元数 X 分量 | - |
| 4 | qy | 四元数 Y 分量 | - |
| 5 | qz | 四元数 Z 分量 | - |
| 6 | qw | 四元数 W 分量 | - |

**重要说明**：
- 四元数必须归一化：`sqrt(qx² + qy² + qz² + qw²) ≈ 1.0`
- 坐标相对于**地图坐标系**（SLAM 启动时的世界坐标系）
- 默认值 `[0, 0, 0, 0, 0, 0, 1]` 表示原点且无旋转

**常用四元数值**：

| 朝向 | qx | qy | qz | qw |
|------|----|----|----|----|
| 无旋转（单位四元数） | 0 | 0 | 0 | 1 |
| 绕 Z 轴旋转 90° | 0 | 0 | 0.707 | 0.707 |
| 绕 Z 轴旋转 180° | 0 | 0 | 1 | 0 |
| 绕 Z 轴旋转 -90° | 0 | 0 | -0.707 | 0.707 |

---

## Configuration Examples / 配置示例

### Example 1: Auto Relocalization / 自动重定位示例

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/home/user/maps/office_map.bin"
```

### Example 2: Init Position at Origin / 在原点指定初始位置

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/home/user/maps/office_map.bin"
  custom_init_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
```

### Example 3: Init Position with Offset / 带偏移的初始位置

Position at (5.2, -3.1, 0) with 90° rotation around Z-axis:

位置在 (5.2, -3.1, 0)，绕 Z 轴旋转 90°：

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/home/user/maps/warehouse_map.bin"
  custom_init_pos: [5.2, -3.1, 0.0, 0.0, 0.0, 0.707, 0.707]
```

### Example 4: Docking Station Pose / 充电桩位置

Known docking station at (10.5, 2.3, 0) facing -X direction (180° rotation):

已知充电桩位置在 (10.5, 2.3, 0)，朝向 -X 方向（旋转 180°）：

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/home/user/maps/factory_map.bin"
  custom_init_pos: [10.5, 2.3, 0.0, 0.0, 0.0, 1.0, 0.0]
```

---

## Programmatic API / 编程接口

> **Runtime mode switch and map swap / 运行时切模式与换图**
>
> Besides configuring `custom_map_mode` / `relocalization_map_abs_path` at startup,
> you can switch the SLAM mode and swap the relocalization map **while the driver is
> running** through two ROS services: `/odin1/set_slam_mode` and `/odin1/set_reloc_map`.
> On supporting firmware the device restarts SLAM asynchronously so the change takes
> effect without relaunching the driver. See
> [docs/runtime_service_operations.md](docs/runtime_service_operations.md) sections 2.5–2.7.
>
> 除了在启动时配置 `custom_map_mode` / `relocalization_map_abs_path`，还可以在 driver
> **运行期间**通过 `/odin1/set_slam_mode` 和 `/odin1/set_reloc_map` 两个 ROS 服务切换
> SLAM 模式并更换重定位地图。在支持该特性的固件上，设备会异步重启 SLAM 使改动生效，
> 无需重启 driver。详见
> [docs/runtime_service_operations.md](docs/runtime_service_operations.md) 2.5–2.7 节。

### English

You can also set `init_pos` programmatically using the `lidar_set_custom_parameter` API. This is useful for:
- Dynamic relocalization during runtime
- Integration with external localization systems
- Setting initial pose from robot's last known position

#### API Function

```cpp
#include "lidar_api.h"

/**
 * @brief Set a custom parameter on the device
 * @param device      Device handle obtained from lidar_open_device()
 * @param param_name  Parameter name (e.g., "init_pos")
 * @param value_data  Pointer to the parameter data
 * @param value_length Size of the data in bytes
 * @return 0 on success, -1 on error, -2 if file transfer in progress
 */
int lidar_set_custom_parameter(device_handle device, 
                                const char* param_name, 
                                const void* value_data, 
                                size_t value_length);
```

#### Complete Example

```cpp
#include "lidar_api.h"
#include <cstdio>
#include <cmath>

// Helper function to create quaternion from yaw angle (rotation around Z-axis)
void yaw_to_quaternion(float yaw_rad, float* qx, float* qy, float* qz, float* qw) {
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = sinf(yaw_rad / 2.0f);
    *qw = cosf(yaw_rad / 2.0f);
}

int set_init_position(device_handle device, 
                      float x, float y, float z, 
                      float qx, float qy, float qz, float qw) {
    // init_pos format: [x, y, z, qx, qy, qz, qw] - 7 floats
    float init_pos[7] = {x, y, z, qx, qy, qz, qw};
    
    int result = lidar_set_custom_parameter(
        device,
        "init_pos",           // Parameter name
        init_pos,             // Data pointer
        sizeof(init_pos)      // 7 * sizeof(float) = 28 bytes
    );
    
    if (result == 0) {
        printf("Successfully set init_pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
               x, y, z, qx, qy, qz, qw);
    } else {
        printf("Failed to set init_pos, error code: %d\n", result);
    }
    
    return result;
}

// Usage examples:

// Example 1: Set position at origin with no rotation
void example_origin(device_handle device) {
    set_init_position(device, 
                      0.0f, 0.0f, 0.0f,      // x, y, z
                      0.0f, 0.0f, 0.0f, 1.0f // qx, qy, qz, qw (identity)
    );
}

// Example 2: Set position with 90° yaw rotation
void example_with_rotation(device_handle device) {
    float qx, qy, qz, qw;
    float yaw_degrees = 90.0f;
    float yaw_rad = yaw_degrees * M_PI / 180.0f;
    
    yaw_to_quaternion(yaw_rad, &qx, &qy, &qz, &qw);
    
    set_init_position(device,
                      5.2f, -3.1f, 0.0f,    // x, y, z
                      qx, qy, qz, qw        // quaternion from yaw
    );
}

// Example 3: Set position from external localization system
void example_from_external_localization(device_handle device,
                                         double ext_x, double ext_y, double ext_yaw) {
    float qx, qy, qz, qw;
    yaw_to_quaternion((float)ext_yaw, &qx, &qy, &qz, &qw);
    
    set_init_position(device,
                      (float)ext_x, (float)ext_y, 0.0f,
                      qx, qy, qz, qw
    );
}
```

#### ROS Integration Example

```cpp
#include "lidar_api.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // ROS1
// or
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // ROS2

// Callback for /initialpose topic (from RViz "2D Pose Estimate" tool)
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                         device_handle device) {
    float init_pos[7] = {
        (float)msg->pose.pose.position.x,
        (float)msg->pose.pose.position.y,
        (float)msg->pose.pose.position.z,
        (float)msg->pose.pose.orientation.x,
        (float)msg->pose.pose.orientation.y,
        (float)msg->pose.pose.orientation.z,
        (float)msg->pose.pose.orientation.w
    };
    
    int result = lidar_set_custom_parameter(device, "init_pos", init_pos, sizeof(init_pos));
    
    if (result == 0) {
        ROS_INFO("Set init_pos from RViz: [%.2f, %.2f, %.2f]", 
                 init_pos[0], init_pos[1], init_pos[2]);
    } else {
        ROS_ERROR("Failed to set init_pos: %d", result);
    }
}
```

#### Important Notes

1. **Call timing**: Set `init_pos` **before** starting the stream with `lidar_start_stream()`
2. **Map mode**: Ensure `custom_map_mode` is set to `2` (relocalization mode)
3. **Map file**: The relocalization map must be set via `lidar_set_relocalization_map()` or YAML config
4. **Thread safety**: `lidar_set_custom_parameter` is thread-safe but blocks until response received

### 中文

您也可以使用 `lidar_set_custom_parameter` API 以编程方式设置 `init_pos`。适用于：
- 运行时动态重定位
- 与外部定位系统集成
- 从机器人上次已知位置设置初始位姿

#### API 函数

```cpp
#include "lidar_api.h"

/**
 * @brief 在设备上设置自定义参数
 * @param device      从 lidar_open_device() 获取的设备句柄
 * @param param_name  参数名称（如 "init_pos"）
 * @param value_data  指向参数数据的指针
 * @param value_length 数据大小（字节）
 * @return 成功返回 0，错误返回 -1，文件传输中返回 -2
 */
int lidar_set_custom_parameter(device_handle device, 
                                const char* param_name, 
                                const void* value_data, 
                                size_t value_length);
```

#### 完整示例

```cpp
#include "lidar_api.h"
#include <cstdio>
#include <cmath>

// 辅助函数：从偏航角（绕 Z 轴旋转）创建四元数
void yaw_to_quaternion(float yaw_rad, float* qx, float* qy, float* qz, float* qw) {
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = sinf(yaw_rad / 2.0f);
    *qw = cosf(yaw_rad / 2.0f);
}

int set_init_position(device_handle device, 
                      float x, float y, float z, 
                      float qx, float qy, float qz, float qw) {
    // init_pos 格式: [x, y, z, qx, qy, qz, qw] - 7 个 float
    float init_pos[7] = {x, y, z, qx, qy, qz, qw};
    
    int result = lidar_set_custom_parameter(
        device,
        "init_pos",           // 参数名
        init_pos,             // 数据指针
        sizeof(init_pos)      // 7 * sizeof(float) = 28 字节
    );
    
    if (result == 0) {
        printf("成功设置 init_pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
               x, y, z, qx, qy, qz, qw);
    } else {
        printf("设置 init_pos 失败，错误码: %d\n", result);
    }
    
    return result;
}

// 使用示例：

// 示例 1：在原点设置位置，无旋转
void example_origin(device_handle device) {
    set_init_position(device, 
                      0.0f, 0.0f, 0.0f,      // x, y, z
                      0.0f, 0.0f, 0.0f, 1.0f // qx, qy, qz, qw（单位四元数）
    );
}

// 示例 2：设置带 90° 偏航旋转的位置
void example_with_rotation(device_handle device) {
    float qx, qy, qz, qw;
    float yaw_degrees = 90.0f;
    float yaw_rad = yaw_degrees * M_PI / 180.0f;
    
    yaw_to_quaternion(yaw_rad, &qx, &qy, &qz, &qw);
    
    set_init_position(device,
                      5.2f, -3.1f, 0.0f,    // x, y, z
                      qx, qy, qz, qw        // 从偏航角计算的四元数
    );
}

// 示例 3：从外部定位系统设置位置
void example_from_external_localization(device_handle device,
                                         double ext_x, double ext_y, double ext_yaw) {
    float qx, qy, qz, qw;
    yaw_to_quaternion((float)ext_yaw, &qx, &qy, &qz, &qw);
    
    set_init_position(device,
                      (float)ext_x, (float)ext_y, 0.0f,
                      qx, qy, qz, qw
    );
}
```

#### ROS 集成示例

```cpp
#include "lidar_api.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // ROS1
// 或
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // ROS2

// /initialpose 话题的回调函数（来自 RViz 的 "2D Pose Estimate" 工具）
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                         device_handle device) {
    float init_pos[7] = {
        (float)msg->pose.pose.position.x,
        (float)msg->pose.pose.position.y,
        (float)msg->pose.pose.position.z,
        (float)msg->pose.pose.orientation.x,
        (float)msg->pose.pose.orientation.y,
        (float)msg->pose.pose.orientation.z,
        (float)msg->pose.pose.orientation.w
    };
    
    int result = lidar_set_custom_parameter(device, "init_pos", init_pos, sizeof(init_pos));
    
    if (result == 0) {
        ROS_INFO("从 RViz 设置 init_pos: [%.2f, %.2f, %.2f]", 
                 init_pos[0], init_pos[1], init_pos[2]);
    } else {
        ROS_ERROR("设置 init_pos 失败: %d", result);
    }
}
```

#### 重要说明

1. **调用时机**：在调用 `lidar_start_stream()` 启动数据流**之前**设置 `init_pos`
2. **地图模式**：确保 `custom_map_mode` 设置为 `2`（重定位模式）
3. **地图文件**：必须通过 `lidar_set_relocalization_map()` 或 YAML 配置设置重定位地图
4. **线程安全**：`lidar_set_custom_parameter` 是线程安全的，但会阻塞直到收到响应

---

## Troubleshooting / 故障排除

### Relocalization Fails / 重定位失败

**English**:
- Ensure starting position is within recommended range (1m/10°)
- Check that the map file path is correct and file exists
- Verify the environment hasn't changed significantly since mapping
- Try gently moving the device to provide more observations

**中文**：
- 确保起始位置在推荐范围内（1m/10°）
- 检查地图文件路径是否正确且文件存在
- 验证环境自建图以来没有显著变化
- 尝试轻轻移动设备以提供更多观测

### init_pos Not Taking Effect / init_pos 未生效

**English**:
- Verify `custom_map_mode` is set to `2`
- Check that `custom_init_pos` has exactly 7 values
- Ensure quaternion is normalized (sum of squares ≈ 1)
- Restart the driver after modifying configuration

**中文**：
- 验证 `custom_map_mode` 设置为 `2`
- 检查 `custom_init_pos` 是否恰好有 7 个值
- 确保四元数已归一化（平方和 ≈ 1）
- 修改配置后重启驱动程序

### TF Not Published / TF 未发布

**English**:
- Relocalization may still be in progress
- Check ROS logs for relocalization status messages
- System operates in fallback mode until relocalization succeeds

**中文**：
- 重定位可能仍在进行中
- 检查 ROS 日志中的重定位状态消息
- 系统在重定位成功前以后备模式运行

### Map File Not Found / 地图文件未找到

**English**:
- Use absolute path (starting with `/`)
- Check file permissions
- Verify file extension is `.bin`

**中文**：
- 使用绝对路径（以 `/` 开头）
- 检查文件权限
- 验证文件扩展名为 `.bin`

---

## Related Topics / 相关话题

| Topic | Description |
|-------|-------------|
| `/odin1/odometry` | Odometry in odom frame |
| `/odin1/odometry_highfreq` | High-frequency odometry |
| `/odin1/cloud_slam` | SLAM point cloud in odom frame |
| `/tf` | Transform tree (includes map→odom after successful relocalization) |

---

## See Also / 参见

- [README.md](README.md) - Main documentation
- [docs/runtime_service_operations.md](docs/runtime_service_operations.md) - Runtime services (set_slam_mode / set_reloc_map) / 运行时服务
- [config/control_command.yaml](config/control_command.yaml) - Configuration file

