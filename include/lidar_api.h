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
#ifndef LIDAR_API_H
#define LIDAR_API_H

/**
 * @file lidar_api.h
 * @brief LiDAR device API for controlling and accessing LiDAR sensor data
 * 
 * This header provides the public interface for interacting with LiDAR devices.
 * It includes functions for device management, data streaming control, and
 * device configuration.
 *
 * @copyright Copyright (c) 2025, Manifold Tech Limited, All Rights Reserved
 * @version 1.0
 */

#include "lidar_api_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the LiDAR system
 * 
 * Must be called before any other lidar function to set up the system resources.
 * 
 * @param cb Callback function for device events (connection, disconnection)
 * @return int 0 on success, negative error code on failure
 */
int lidar_system_init(lidar_device_callback_t cb);

/**
 * @brief Deinitialize the LiDAR system
 * 
 * Releases all resources allocated by the system. Should be called when
 * application is shutting down.
 * 
 * @return int 0 on success, negative error code on failure
 */
int lidar_system_deinit(void);

/**
 * @brief Create a handle for a LiDAR device
 * 
 * @param dev_info Information about the LiDAR device to create
 * @param device Pointer to receive the device handle upon success
 * @return int 0 on success, negative error code on failure
 */
int lidar_create_device(lidar_device_info_t *dev_info, device_handle *device);

/**
 * @brief Destroy a LiDAR device handle
 * 
 * Releases resources associated with the device handle. Must be called 
 * when the device is no longer needed.
 * 
 * @param device Handle to the device to destroy
 * @return int 0 on success, negative error code on failure
 */
int lidar_destory_device(device_handle device);

/**
 * @brief Register callback function for receiving LiDAR data streams
 * 
 * Sets up a callback function that will be called when new data is available.
 * 
 * @param device Handle to the target device
 * @param cb Callback information containing function pointers for different data types
 * @return int 0 on success, negative error code on failure
 */
int lidar_register_stream_callback(device_handle device, lidar_data_callback_info_t cb);

/**
 * @brief Unregister stream callback for a device
 * 
 * Stops the device from calling back when new data is available.
 * 
 * @param device Handle to the target device
 * @return int 0 on success, negative error code on failure
 */
int lidar_unregister_stream_callback(device_handle device);

/**
 * @brief Open a LiDAR device for communication
 * 
 * Establishes a connection to the physical device.
 * 
 * @param device Handle to the device to open
 * @return int 0 on success, negative error code on failure
 */
int lidar_open_device(device_handle device);

/**
 * @brief Close a LiDAR device
 * 
 * Closes the connection to the physical device.
 * 
 * @param device Handle to the device to close
 * @return int 0 on success, negative error code on failure
 */
int lidar_close_device(device_handle device);

/**
 * @brief Set the operating mode of the LiDAR device
 * 
 * @param device Handle to the target device
 * @param mode Operating mode to set (see mode definitions in lidar_api_type.h)
 * @return int 0 on success, negative error code on failure
 */
int lidar_set_mode(device_handle device, int mode);

/**
 * @brief Start data streaming from the device
 * 
 * Begins the flow of data from the device for the specified type.
 * 
 * @param device Handle to the target device
 * @param type Type of data stream to start (see stream type definitions in lidar_api_type.h)
 * @param dtof_subframe_odr DTOF subframe ODR from device, used for raw point cloud per-point time offset calculation
 * @return int 0 on success, negative error code on failure
 */
int lidar_start_stream(device_handle device, int type, uint32_t &dtof_subframe_odr);

/**
 * @brief Stop data streaming from the device
 * 
 * Stops the flow of data from the device for the specified type.
 * 
 * @param device Handle to the target device
 * @param type Type of data stream to stop
 * @return int 0 on success, negative error code on failure
 */
int lidar_stop_stream(device_handle device, int type);

/**
 * @brief Activate a specific stream type on the device
 * 
 * Enables a specific data stream type in the device configuration.
 * 
 * @param device Handle to the target device
 * @param type Type of data stream to activate
 * @return int 0 on success, negative error code on failure
 */
int lidar_activate_stream_type(device_handle device, int type);

/**
 * @brief Deactivate a specific stream type on the device
 * 
 * Disables a specific data stream type in the device configuration.
 * 
 * @param device Handle to the target device
 * @param type Type of data stream to deactivate
 * @return int 0 on success, negative error code on failure
 */
int lidar_deactivate_stream_type(device_handle device, int type);

/**
 * @brief Get calibration file from the device
 * 
 * Retrieves the calibration file from the device.
 * 
 * @param device Handle to the target device
 * @param path Path to save the calibration file
 * @return int 0 on success, negative error code on failure
 */
int lidar_get_calib_file(device_handle device, const char* path);

/**
 * @brief Set log verbosity level
 * 
 * Controls the amount of log information generated by the LiDAR API.
 * 
 * @param level Log level to set (see level definitions in lidar_api_type.h)
 */
void lidar_log_set_level(lidar_log_level_e level);

/**
 * @brief Get the version information of the LiDAR device
 * 
 * Retrieves version information including firmware, system, and application versions.
 * 
 * @param device Handle to the target device
 * @param version struct Pointer to receive the version information
 * @return int 0 on success, negative error code on failure
 */
int lidar_get_version(device_handle device);

/**
 * @brief Set custom algorithm parameters for the device
 * 
 * Sends custom parameter settings to the device.
 *
 * @param device Handle to the target device
 * @param param_name String name of the parameter to set
 * @param value_data Pointer to the value data to set for the parameter
 * @param value_length Length of the value data in bytes
 * @return int 0 on success, negative error code on failure
 */
int lidar_set_custom_parameter(device_handle device, const char* param_name, const void* value_data, size_t value_length);

/**
 * @brief Get custom algorithm parameters for the device
 * 
 * Get custom parameter settings from the device.
 *
 * @param device Handle to the target device
 * @param param_name String name of the parameter to get
 * @param value Integer value to get for the parameter
 * @return int 0 on success, negative error code on failure
 */
int lidar_get_custom_parameter(device_handle device, const char* param_name, int* value);

/**
 * @brief Set the map file used for relocalization
 *
 * Read & send specified map file to device for relocalization
 *
 * @param device Handle to the target device
 * @param abs_path Absolute path to the map file
 * @return int 0 on success, otherwise on failure
 */
 int lidar_set_relocalization_map(device_handle device, const char* abs_path);

 /**
 * @brief Get the mapping result file from device
 *
 * Read & send specified map file from device to host
 *
 * @param device Handle to the target device
 * @param dest_dir Destination directory to save the map file
 * @param file_name File name to save the map file
 * @return int 0 on success, -1 on failure without error code, error code (> 0) otherwise
 */
 int lidar_get_mapping_result(device_handle device, const char* dest_dir, const char* file_name);

  /**
 * @brief enable device log
 *
 *
 * @param device Handle to the target device
 * @param dest_dir Destination directory to save the logs
 * @return int 0 on success, -1 on failure
 */
 int lidar_enable_encrypted_device_log(device_handle device, const char* dest_dir);

#ifdef __cplusplus
}
#endif

#endif // LIDAR_API_H