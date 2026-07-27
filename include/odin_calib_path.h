/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef ODIN_CALIB_PATH_H
#define ODIN_CALIB_PATH_H

#include <cstdlib>
#include <string>

// Single source of truth for the calibration file location, shared by the
// writer (host_sdk_sample) and every reader (cloud_reprojection, pcd2depth).
// Header-only inline functions avoid multiple-definition across translation units.
namespace odin_ros_driver {

// Resolve the writable runtime directory that stores calib.yaml.
// Priority: ODIN_CALIB_DIR -> ROS_HOME -> ~/.ros -> /tmp.
// Works for both ROS1 and ROS2 (~/.ros is the shared default ROS_HOME).
inline std::string GetOdinRuntimeDir() {
    if (const char* custom = std::getenv("ODIN_CALIB_DIR")) {
        return std::string(custom);
    }
    if (const char* ros_home = std::getenv("ROS_HOME")) {
        return std::string(ros_home) + "/odin_ros_driver";
    }
    if (const char* home = std::getenv("HOME")) {
        return std::string(home) + "/.ros/odin_ros_driver";
    }
    return "/tmp/odin_ros_driver";
}

// Canonical calib.yaml path shared by writer and all readers.
inline std::string GetOdinCalibFile() {
    return GetOdinRuntimeDir() + "/calib.yaml";
}

}  // namespace odin_ros_driver

#endif  // ODIN_CALIB_PATH_H
