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

See `README.md` section `4.6` for `/odin1/get_ae`, `/odin1/get_awb`, `/odin1/set_ae`, `/odin1/set_ae_max_exposure`, `/odin1/set_awb`.

AE/AWB 服务 `/odin1/get_ae`、`/odin1/get_awb`、`/odin1/set_ae`、`/odin1/set_ae_max_exposure`、`/odin1/set_awb` 详见 `README.md` 4.6 节。

**Set AE upper exposure limit / 设置 AE 最大曝光上限**

**Service / 服务**: `/odin1/set_ae_max_exposure`  
**Type / 类型**: `odin_ros_driver/srv/SetAeMaxExposure`

Clamps the upper exposure limit used by the AE AUTO convergence loop (AE opcode 0x26). It does **not** switch AE into manual mode, so it is complementary to `/odin1/set_ae`, not a replacement. The command is only accepted by the device **after the data stream has started**; the device enforces a 5.0 ms minimum and returns `rc = 403` for smaller values.

限制 AE 自动收敛循环可用的最大曝光时间（AE opcode 0x26）。它**不会**把 AE 切到手动模式，因此与 `/odin1/set_ae` 互补而非替代。设备**仅在数据流启动后**接受该命令；设备最小值为 5.0 ms，更小的值返回 `rc = 403`。

| Field | Type | Description / 说明 |
|---|---|---|
| `max_exposure_ms` | `float32` | Maximum exposure time in milliseconds (`>= 5.0`; practical ceiling `33.0` at `dtof_fps=290`). `max_exposure_ms` 最大曝光时间（毫秒，`>= 5.0`；`dtof_fps=290` 时实用上限 33.0）。 |

Response / 响应: `success` (`bool`), `rc` (`int32`, see section 3 and `README.md` 4.6.3). There is no SDK getter for this value, so the service returns no readback field. / 本服务只返回 `success` 与 `rc`（见第 3 节及 `README.md` 4.6.3）；SDK 无对应查询接口，因此不回读该值。

**Usage / 调用**

ROS2:
```bash
ros2 service call /odin1/set_ae_max_exposure odin_ros_driver/srv/SetAeMaxExposure '{max_exposure_ms: 20.0}'
```

ROS1:
```bash
rosservice call /odin1/set_ae_max_exposure '{max_exposure_ms: 20.0}'
```

The same limit can be set at startup with `ae_max_exposure_time` in `config/control_command.yaml` (`0` = keep the device default). / 同一上限也可通过 `config/control_command.yaml` 中的 `ae_max_exposure_time` 在启动时设置（`0` 表示保持设备默认值）。

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

### 2.5 Set SLAM Mode / 切换 SLAM 模式

**Service / 服务**: `/odin1/set_slam_mode`  
**Type / 类型**: `odin_ros_driver/srv/SetSlamMode`

Switches the SLAM `map_mode` at runtime. The value is forwarded as the `map_mode` custom parameter (control command `SYS_CONTROL_ALGO_SETTING`). On firmware that supports it, changing `map_mode` while SLAM is running makes the device **restart SLAM asynchronously** so the new mode takes effect. The reply only confirms the value was persisted, not that the restart finished — wait for the SLAM data stream (odometry / cloud) to resume.

在运行时切换 SLAM 的 `map_mode`。取值会作为 `map_mode` 自定义参数下发（控制命令 `SYS_CONTROL_ALGO_SETTING`）。在支持该特性的固件上，SLAM 运行期间改变 `map_mode` 会让设备**异步重启 SLAM** 使新模式生效。应答仅表示配置已落盘，不代表重启已完成——请等待 SLAM 数据流（里程计 / 点云）恢复。

**Request / 请求**

| Field | Type | Description / 说明 |
|---|---|---|
| `mode` | `int32` | Target `map_mode`: `0` = odom, `1` = mapping, `2` = reloc. 目标 `map_mode`：`0` 里程计，`1` 建图，`2` 重定位。 |

**Response / 响应**

| Field | Type | Description / 说明 |
|---|---|---|
| `success` | `bool` | `true` if `rc == 0`. 当 `rc == 0` 时为 true。 |
| `rc` | `int32` | `0`=ok, `-1`=invalid mode (not 0/1/2) or SDK error, `-100`=device not connected. `0` 成功，`-1` 非法模式或 SDK 错误，`-100` 设备未连接。 |

**Usage / 调用**

ROS2:
```bash
ros2 service call /odin1/set_slam_mode odin_ros_driver/srv/SetSlamMode '{mode: 2}'
```

ROS1:
```bash
rosservice call /odin1/set_slam_mode '{mode: 2}'
```

### 2.6 Set Relocalization Map / 运行时换图

**Service / 服务**: `/odin1/set_reloc_map`  
**Type / 类型**: `odin_ros_driver/srv/SetRelocMap`

Transfers a new relocalization map file to the device at runtime. On firmware that supports it, if the device is currently in reloc mode (`map_mode == 2`) and SLAM is running, finishing the transfer makes the device **restart SLAM asynchronously** and reload the new map. Otherwise the new map takes effect at the next SLAM start. The transfer may take a while, so it runs in a background thread: `rc`/`success` only report whether the request was **accepted**; the real transfer result is printed in the driver log.

在运行时向设备传输新的重定位地图文件。在支持该特性的固件上，若设备当前处于重定位模式（`map_mode == 2`）且 SLAM 正在运行，传输完成会让设备**异步重启 SLAM** 并加载新地图；否则新地图在下次 SLAM 启动时生效。传输可能耗时，因此在后台线程执行：`rc`/`success` 仅表示请求是否被**接受**，真正的传输结果打印在 driver 日志中。

**Request / 请求**

| Field | Type | Description / 说明 |
|---|---|---|
| `map_path` | `string` | Absolute path to the relocalization map file (`.bin`). 重定位地图文件的绝对路径（`.bin`）。 |

**Response / 响应**

| Field | Type | Description / 说明 |
|---|---|---|
| `success` | `bool` | `true` if the transfer request was accepted. 传输请求被接受时为 true。 |
| `rc` | `int32` | `0`=accepted, `-1`=invalid/missing file, `-2`=another transfer in progress, `-100`=device not connected. `0` 已受理，`-1` 文件无效或不存在，`-2` 另一次传输进行中，`-100` 设备未连接。 |

**Usage / 调用**

ROS2:
```bash
ros2 service call /odin1/set_reloc_map odin_ros_driver/srv/SetRelocMap '{map_path: "/abs/path/map_merged.bin"}'
```

ROS1:
```bash
rosservice call /odin1/set_reloc_map '{map_path: "/abs/path/map_merged.bin"}'
```

### 2.7 Switch to Reloc and Swap Map Together / 切重定位并同时换图

To switch from mapping to relocalization **with a new map** while restarting SLAM only **once**, transfer the map first, then set the mode (the map transfer alone does not restart SLAM while `map_mode != 2`):

要从建图切到重定位并**同时换新图**、且全程只重启**一次** SLAM，应**先换图、后切模式**（`map_mode != 2` 时单纯换图不会触发重启）：

```bash
# 1) Transfer the new map (no restart yet, still in mapping mode)
#    先传新图（此时仍在建图模式，不触发重启）
ros2 service call /odin1/set_reloc_map odin_ros_driver/srv/SetRelocMap '{map_path: "/abs/path/map_merged.bin"}'
# wait for "Relocalization map swap completed" in the driver log
# 等待 driver 日志出现 "Relocalization map swap completed"

# 2) Switch to reloc mode (triggers the single restart, loads the new map)
#    再切到重定位模式（触发唯一一次重启，加载新图）
ros2 service call /odin1/set_slam_mode odin_ros_driver/srv/SetSlamMode '{mode: 2}'
```

> The reverse order (set mode first, then transfer) works but restarts SLAM **twice**. 反向顺序（先切模式、后换图）功能正确，但会重启 SLAM **两次**。

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
| `ae_max_exposure_time` | AE (auto exposure) upper exposure limit in milliseconds, applied **after the data stream has started** (the device only accepts it while streaming). `0` = disabled (device default); valid `>= 5.0`; practical ceiling `33.0`. AE 自动曝光上限（毫秒），在**数据流启动后**下发（设备仅在开流后接受）。`0` 表示关闭（设备默认）；有效值 `>= 5.0`；实用上限 33.0。 |
