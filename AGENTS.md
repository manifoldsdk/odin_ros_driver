# 项目验证 / Project verification

## ROS 2 编译 / ROS 2 build

- 在已配置的 ROS 2 环境中执行。以下路径以当前包目录为基准。
- Run in a configured ROS 2 environment. Paths below are relative to this package directory.

```bash
cmake -S . -B ../../build/odin_ros_driver -DBUILD_SYSTEM=ROS2 -DBUILD_TESTING=ON
cmake --build ../../build/odin_ros_driver -j2
```

## 回归测试 / Regression tests

```bash
ctest --test-dir ../../build/odin_ros_driver -R '^driver_lifecycle$' --output-on-failure
```

- 测试覆盖热插拔事件代际、旧连接失效、并发通知、工作线程停止和标定内容校验；不会连接设备。
- Tests cover hotplug generations, stale connections, concurrent notifications, worker shutdown, and calibration validation without connecting to a device.
- 硬件验证仍需实际拔插，检查话题恢复以及正常和断连状态下的退出。
- Hardware validation still requires actual unplug/replug cycles, topic recovery checks, and shutdown checks both connected and disconnected.

## 安全构建注意事项 / Safe build notes

- `script/build_ros2.sh` 包含删除构建、安装和日志目录的命令；不要将其用于未经确认的增量验证。
- `script/build_ros2.sh` deletes build, install, and log directories; do not use it for incremental verification without explicit confirmation.
- 驱动链接包内的预编译 SDK 静态库 `lib/liblydHostApi_amd.a`；修改 SDK 源码后必须重建并刷新该归档，驱动才会生效。
- The driver links the packaged precompiled SDK archive `lib/liblydHostApi_amd.a`; after editing SDK sources it must be rebuilt and the archive refreshed before the driver picks up the change.

## SDK 重建 / Rebuilding the SDK

- SDK 源码位于 `/home/xxx/work/odin1/sdk/lyd_host_sdk`。重建并刷新 x86 归档：
- The SDK sources live in `/home/xxx/work/odin1/sdk/lyd_host_sdk`. Rebuild and refresh the x86 archive:

```bash
cmake --build /home/xxx/work/odin1/sdk/lyd_host_sdk/.build --target lydHostApi -j2
cp /home/xxx/work/odin1/sdk/lyd_host_sdk/.build/sdk/modules-test/host_sdk_sample/liblydHostApi.a \
   lib/liblydHostApi_amd.a
```

- 刷新归档前会保留 `lib/liblydHostApi_amd.a.bak_<date>` 备份；确认无误后可删除。
- A `lib/liblydHostApi_amd.a.bak_<date>` backup is kept before refreshing; delete it once verified.
- USB 分包重组回归测试（不连接设备）：
- USB packet-reassembly regression test (no device needed):

```bash
g++ -std=c++17 -O1 -pthread \
  -I/home/xxx/work/odin1/sdk/lyd_host_sdk/sdk/comm/interface \
  -I/home/xxx/work/odin1/sdk/lyd_host_sdk/sdk/comm/libusb \
  -I/home/xxx/work/odin1/sdk/lyd_host_sdk/sdk/common/thread \
  -I/home/xxx/work/odin1/sdk/lyd_host_sdk/sdk/logger \
  -I/home/xxx/work/odin1/sdk/lyd_host_sdk/libs/libusb/inc \
  /home/xxx/work/odin1/sdk/lyd_host_sdk/sdk/modules-test/dxfer/packet_group_test.cpp \
  lib/liblydHostApi_amd.a -lusb-1.0 -lrt -o /tmp/packet_group_test && /tmp/packet_group_test
```

- 注意：仓库里的 arm 归档 `lib/liblydHostApi_arm.a` 未重建；ARM 目标需用交叉工具链单独重建后同样刷新。
- Note: the committed ARM archive `lib/liblydHostApi_arm.a` was NOT rebuilt; rebuild it with the cross toolchain and refresh it the same way for ARM targets.
