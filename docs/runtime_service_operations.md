# Runtime Service and Parameter Operations / 运行时服务与参数操作

This document describes how to control the Odin ROS driver at runtime through `set_param.sh` and ROS services.

本文档说明如何通过 `set_param.sh` 命令文件和 ROS Service 在运行时控制 Odin ROS driver。

---

## 1. set_param.sh (Command File) / 命令文件

`set_param.sh` writes a `set <key> <value>` line to `/tmp/odin_command.txt`. The driver polls this file every second and executes the command.

`set_param.sh` 会把 `set <key> <value>` 写入 `/tmp/odin_command.txt`，driver 每秒轮询该文件并执行。

```shell
# Save map / 保存地图
./set_param.sh save_map 1

# Reset algorithm / 复位算法
./set_param.sh algo_reset 1

# Unknown keys are rejected by the device with an error / 未知键会被设备拒绝并报错
./set_param.sh map 1          # ERROR: "map" is not a valid device parameter
```

### Supported command parameters / 支持的命令参数

| Parameter / 参数 | Accepted values / 取值 | Description / 说明 |
|---|---|---|
| `save_map` | `0` or `1` | `1` triggers map saving in background; `0` sets the parameter to 0. `1` 在后台触发地图保存，`0` 设置参数为 0。 |
| `algo_reset` | `0` or `1` | Sends the `algo_reset` command to the device. 向设备发送 `algo_reset` 命令。 |
| `spad_filter` | `0` or `1` | Enables/disables the host-side SPAD crosstalk filter. 启用/禁用主机端 SPAD 串扰滤波。 |

---

## 2. ROS Services / ROS 服务

The driver registers the following services while running. All services return `success` (bool) and `rc` (int32). Device must be connected before calling, otherwise `rc = -100` is returned.

driver 运行期间注册以下服务。所有服务返回 `success` (bool) 和 `rc` (int32)。调用前设备必须已连接，否则返回 `rc = -100`。

### 2.1 AE/AWB Services / AE/AWB 服务

See `README.md` section `4.6` for `/odin1/get_ae`, `/odin1/get_awb`, `/odin1/set_ae`, `/odin1/set_awb`.

AE/AWB 服务 `/odin1/get_ae`、`/odin1/get_awb`、`/odin1/set_ae`、`/odin1/set_awb` 详见 `README.md` 4.6 节。

### 2.2 Device Log Retrieval / 设备日志获取

**Service / 服务**: `/odin1/get_device_logs`  
**Type / 类型**: `odin_ros_driver/srv/GetDeviceLogs`

**Request / 请求**

| Field | Type | Description / 说明 |
|---|---|---|
| `dest_dir` | `string` | Destination directory for the device log archive. 设备日志压缩包保存目录。 |

**Response / 响应**

| Field | Type | Description / 说明 |
|---|---|---|
| `success` | `bool` | `true` if `rc == 0`. 当 `rc == 0` 时为 true。 |
| `rc` | `int32` | `0`=ok, `-1`=invalid args, `-2`=transfer in progress, `-3`=timeout/stall. |

**Usage / 调用**

ROS2:
```bash
ros2 service call /odin1/get_device_logs odin_ros_driver/srv/GetDeviceLogs '{dest_dir: "/tmp/odin_logs"}'
```

ROS1:
```bash
rosservice call /odin1/get_device_logs '{dest_dir: "/tmp/odin_logs"}'
```

### 2.3 Save Map / 保存地图

**Service / 服务**: `/odin1/save_map`  
**Type / 类型**: `odin_ros_driver/srv/SaveMap`

**Request / 请求**

| Field | Type | Description / 说明 |
|---|---|---|
| `value` | `int32` | `1` triggers map saving in background. Other values are forwarded as the `save_map` custom parameter. `1` 在后台触发地图保存，其他值转发为 `save_map` 自定义参数。 |

**Response / 响应**

| Field | Type | Description / 说明 |
|---|---|---|
| `success` | `bool` | `true` if the request was accepted. 请求被接受时为 true。 |
| `rc` | `int32` | `0`=ok, `-2`=another map transfer already in progress. |

**Usage / 调用**

ROS2:
```bash
ros2 service call /odin1/save_map odin_ros_driver/srv/SaveMap '{value: 1}'
```

ROS1:
```bash
rosservice call /odin1/save_map '{value: 1}'
```

When `value = 1`, the driver builds the output path from `mapping_result_dest_dir` and `mapping_result_file_name` in `config/control_command.yaml`, then starts a background transfer. Progress is printed in the driver log.

`value = 1` 时，driver 根据 `config/control_command.yaml` 中的 `mapping_result_dest_dir` 和 `mapping_result_file_name` 构建输出路径，然后在后台开始传输，进度会打印在 driver 日志中。

### 2.4 Reset Algorithm / 复位算法

**Service / 服务**: `/odin1/reset_algo`  
**Type / 类型**: `odin_ros_driver/srv/ResetAlgo`

**Request / 请求**

| Field | Type | Description / 说明 |
|---|---|---|
| `value` | `int32` | `1` sends the `algo_reset` command. `1` 发送 `algo_reset` 命令。 |

**Response / 响应**

| Field | Type | Description / 说明 |
|---|---|---|
| `success` | `bool` | `true` if `rc == 0`. |
| `rc` | `int32` | SDK return code from `lidar_set_custom_parameter`. `lidar_set_custom_parameter` 的返回码。 |

**Usage / 调用**

ROS2:
```bash
ros2 service call /odin1/reset_algo odin_ros_driver/srv/ResetAlgo '{value: 1}'
```

ROS1:
```bash
rosservice call /odin1/reset_algo '{value: 1}'
```

---

## 3. Return Code Convention / 返回码说明

All services and command-file operations share the same `rc` convention where applicable:

| `rc` | Meaning / 含义 |
|---|---|
| `0` | Success / 成功 |
| `-1` | Invalid arguments / 参数无效 |
| `-2` | Another transfer already in progress / 另一次传输正在进行 |
| `-3` | Transfer timeout or stalled / 传输超时或卡死 |
| `-100` | Driver has not connected to the device yet / driver 尚未连接到设备 |

---

## 4. Configuration Parameters / 配置参数

Parameters related to runtime operations in `config/control_command.yaml`:

| Parameter / 参数 | Description / 说明 |
|---|---|
| `custom_map_mode` | `0` = Odometry, `1` = SLAM (supports `save_map`), `2` = Relocalization. `0` 里程计模式，`1` SLAM 建图模式（支持 `save_map`），`2` 重定位模式。 |
| `mapping_result_dest_dir` | Directory for saved maps. 保存地图的目录。 |
| `mapping_result_file_name` | File name for saved map. 保存地图的文件名。 |
| `relocalization_map_abs_path` | Absolute path to pre-built map for relocalization. 重定位所需的预建地图绝对路径。 |
