#!/bin/bash
# ROS1 launch script with automatic device detection
# Usage: ./launch_ros1.sh [num_devices]

# Detect connected devices
DETECTED_DEVICES=$(lsusb | grep -cE "2207:0019|2207:001a" || echo "1")
if [ "$DETECTED_DEVICES" -eq 0 ]; then
    DETECTED_DEVICES=1
fi

# Use provided num_devices or detected count
NUM_DEVICES=${1:-$DETECTED_DEVICES}

echo "[launch_ros1.sh] Detected $DETECTED_DEVICES connected device(s), launching $NUM_DEVICES device node(s)"

# Export for launch file
export ODIN_NUM_DEVICES=$NUM_DEVICES

# Get script directory and package path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS workspace
if [ -f "$PKG_DIR/../../../devel/setup.bash" ]; then
    source "$PKG_DIR/../../../devel/setup.bash"
fi

# Launch with num_devices argument
roslaunch odin_ros_driver odin1_ros1.launch num_devices:=$NUM_DEVICES
