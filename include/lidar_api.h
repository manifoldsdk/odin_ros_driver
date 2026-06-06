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
 * 
 * ┌──────────────────────────────────────────────────────────────────────────────┐
 * │                              QUICK START                                     │
 * ├──────────────────────────────────────────────────────────────────────────────┤
 * │ 1. API Call Sequence (typical usage):                                        │
 * │                                                                              │
 * │    lidar_system_init(device_cb)     // Initialize system, register device cb│
 * │           ↓                                                                  │
 * │    [Wait for device_cb with attach=true to get device info]                  │
 * │           ↓                                                                  │
 * │    lidar_create_device(&dev_info, &handle)   // Create device handle         │
 * │           ↓                                                                  │
 * │    lidar_register_stream_callback(handle, cb_info)  // Register data callback│
 * │           ↓                                                                  │
 * │    lidar_open_device(handle)        // Connect to device                     │
 * │           ↓                                                                  │
 * │    lidar_set_mode(handle, mode)     // Set RAW or SLAM mode                  │
 * │           ↓                                                                  │
 * │    lidar_start_stream(handle, type, odr)  // Start specific data stream      │
 * │           ↓                                                                  │
 * │    [Data arrives via registered callback]                                    │
 * │           ↓                                                                  │
 * │    lidar_stop_stream(handle, type)  // Stop data stream                      │
 * │           ↓                                                                  │
 * │    lidar_close_device(handle)       // Disconnect                            │
 * │           ↓                                                                  │
 * │    lidar_destory_device(handle)     // Release device handle                 │
 * │           ↓                                                                  │
 * │    lidar_system_deinit()            // Cleanup system resources              │
 * │                                                                              │
 * ├──────────────────────────────────────────────────────────────────────────────┤
 * │ 2. Operating Modes and Available Data Types:                                 │
 * │                                                                              │
 * │    LIDAR_MODE_RAW:                                                           │
 * │      - LIDAR_DT_RAW_RGB      (RGB camera image, NV12 format)                 │
 * │      - LIDAR_DT_RAW_IMU      (IMU data at 400Hz)                             │
 * │      - LIDAR_DT_RAW_DTOF     (DTOF depth + point cloud + confidence)         │
 * │      - LIDAR_DT_DEV_STATUS   (Device status info)                            │
 * │      - LIDAR_DT_NTP          (PTP/NTP sync data)                             │
 * │                                                                              │
 * │    LIDAR_MODE_SLAM:                                                          │
 * │      - All RAW mode types, plus:                                             │
 * │      - LIDAR_DT_SLAM_CLOUD           (SLAM point cloud, XYZRGBA)             │
 * │      - LIDAR_DT_SLAM_ODOMETRY        (SLAM odometry at ~10Hz)                │
 * │      - LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ (Odometry at IMU rate ~400Hz)         │
 * │      - LIDAR_DT_SLAM_ODOMETRY_TF     (Map-Odom transform)                    │
 * │                                                                              │
 * ├──────────────────────────────────────────────────────────────────────────────┤
 * │ 3. activate_stream_type vs start_stream:                                     │
 * │                                                                              │
 * │    lidar_activate_stream_type():                                             │
 * │      - Configure which stream types are enabled in device                    │
 * │      - Can enable multiple types before starting                             │
 * │      - Does NOT start data transmission                                      │
 * │                                                                              │
 * │    lidar_start_stream():                                                     │
 * │      - Actually starts data transmission for the specified type              │
 * │      - Callback will begin receiving data after this call                    │
 * │                                                                              │
 * │    Typical flow:                                                             │
 * │      activate_stream_type(LIDAR_DT_RAW_IMU);   // Enable IMU                 │
 * │      activate_stream_type(LIDAR_DT_RAW_DTOF);  // Enable DTOF                │
 * │      start_stream(LIDAR_DT_RAW_IMU, ...);      // Start IMU stream           │
 * │      start_stream(LIDAR_DT_RAW_DTOF, ...);     // Start DTOF stream          │
 * │                                                                              │
 * ├──────────────────────────────────────────────────────────────────────────────┤
 * │ 4. Callback Notes (IMPORTANT):                                               │
 * │                                                                              │
 * │    Thread Safety:                                                            │
 * │      - Callbacks are invoked from internal SDK threads                       │
 * │      - Different data types may use different threads                        │
 * │      - User callback code must be thread-safe                                │
 * │                                                                              │
 * │    Data Lifetime:                                                            │
 * │      - Data pointers (pAddr) are ONLY valid during callback execution        │
 * │      - If you need to keep data, COPY it before callback returns             │
 * │      - Do NOT store or dereference pAddr after callback returns              │
 * │                                                                              │
 * │    Performance:                                                              │
 * │      - Avoid blocking or time-consuming operations in callback               │
 * │      - Long callback execution may cause data loss or jitter                 │
 * │      - For heavy processing, copy data and process in separate thread        │
 * │                                                                              │
 * ├──────────────────────────────────────────────────────────────────────────────┤
 * │ 5. Error Codes:                                                              │
 * │      0  : Success                                                            │
 * │     -1  : General failure                                                    │
 * │     -2  : Invalid parameter                                                  │
 * │     -3  : Device not found / not connected                                   │
 * │     -4  : Operation timeout                                                  │
 * │     -5  : Resource allocation failed                                         │
 * │                                                                              │
 * └──────────────────────────────────────────────────────────────────────────────┘
 */

#include "lidar_api_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the LiDAR system
 * 
 * Must be called before any other lidar function to set up the system resources.
 * This function starts device discovery and will invoke the callback when devices
 * are found or disconnected.
 * 
 * @param cb Callback function for device events:
 *           - Called with attach=true when a new device is discovered
 *           - Called with attach=false when a device is disconnected
 *           - The lidar_device_info_t contains serial number to identify the device
 * 
 * Example callback:
 *   void device_callback(const lidar_device_info_t* info, bool attach) {
 *       if (attach) {
 *           printf("Device connected: %s\n", info->serial);
 *           // Save info for lidar_create_device()
 *       } else {
 *           printf("Device disconnected: %s\n", info->serial);
 *       }
 *   }
 * 
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
 * Creates a device handle using the device info received from the device callback.
 * The handle is used for all subsequent operations on the device.
 * 
 * @param dev_info Information about the LiDAR device (from lidar_system_init callback)
 *                 - serial: Device serial number (required, used to identify device)
 *                 - model: Device model string
 *                 - online: Device connection status
 * @param device [OUTPUT] Pointer to receive the device handle upon success
 * @return int 0 on success, negative error code on failure
 * 
 * @note The dev_info should be the same structure received from the device callback
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
 * All data types use the same callback; use lidar_data_t.type to distinguish.
 * 
 * @param device Handle to the target device
 * @param cb Callback info structure:
 *           - data_callback: Function pointer, signature: void(const lidar_data_t*, void*)
 *           - user_data: User context pointer passed to callback (can be NULL)
 * 
 * Example:
 *   void data_callback(const lidar_data_t* data, void* user_data) {
 *       switch (data->type) {
 *           case LIDAR_DT_RAW_IMU:
 *               imu_convert_data_t* imu = (imu_convert_data_t*)data->stream.imageList[0].pAddr;
 *               // Process IMU data (COPY if needed, pointer invalid after return)
 *               break;
 *           case LIDAR_DT_RAW_DTOF:
 *               // data->stream.imageList[0]: depth
 *               // data->stream.imageList[1]: point cloud XYZ
 *               // data->stream.imageList[2]: confidence
 *               // data->stream.imageList[3]: intensity
 *               break;
 *       }
 *   }
 * 
 * @return int 0 on success, negative error code on failure
 * 
 * @warning Callback is invoked from SDK internal threads. Avoid blocking operations.
 * @warning Data pointers are only valid during callback execution. Copy if needed.
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
 * Must be called after lidar_open_device() and before lidar_start_stream().
 * Mode determines which data types are available for streaming.
 * 
 * @param device Handle to the target device
 * @param mode Operating mode to set:
 *              - LIDAR_MODE_RAW: Raw sensor data (RGB, IMU, DTOF)
 *              - LIDAR_MODE_SLAM: SLAM processing enabled (adds odometry, point cloud)
 * @return int 0 on success, negative error code on failure
 */
int lidar_set_mode(device_handle device, int mode);

/**
 * @brief Start data streaming from the device
 * 
 * Begins the flow of data from the device for the specified type.
 * After calling this function, registered callbacks will start receiving data.
 * 
 * @param device Handle to the target device
 * @param type Type of data stream to start (lidar_data_type_e):
 *              - LIDAR_DT_RAW_RGB: RGB camera frames
 *              - LIDAR_DT_RAW_IMU: IMU data at 400Hz
 *              - LIDAR_DT_RAW_DTOF: Depth sensor data
 *              - LIDAR_DT_SLAM_CLOUD: SLAM point cloud (requires SLAM mode)
 *              - LIDAR_DT_SLAM_ODOMETRY: SLAM odometry (requires SLAM mode)
 *              - etc. (see lidar_data_type_e in lidar_api_type.h)
 * @param dtof_subframe_odr [OUTPUT] Returns DTOF subframe interval in microseconds.
 *                          Only meaningful when type=LIDAR_DT_RAW_DTOF.
 *                          For other types, this value can be ignored.
 * @return int 0 on success, negative error code on failure
 * 
 * @note You can start multiple stream types simultaneously by calling this
 *       function multiple times with different types.
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
 * This configures the device to be ready for the stream type, but does NOT
 * start actual data transmission. Call lidar_start_stream() to begin streaming.
 * 
 * Use this to pre-configure multiple stream types before starting them:
 *   activate_stream_type(handle, LIDAR_DT_RAW_IMU);
 *   activate_stream_type(handle, LIDAR_DT_RAW_DTOF);
 *   start_stream(handle, LIDAR_DT_RAW_IMU, odr);
 *   start_stream(handle, LIDAR_DT_RAW_DTOF, odr);
 * 
 * @param device Handle to the target device
 * @param type Type of data stream to activate (lidar_data_type_e)
 * @return int 0 on success, negative error code on failure
 * 
 * @see lidar_start_stream() to actually begin data transmission
 * @see lidar_deactivate_stream_type() to disable a stream type
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
 * @brief Get the calibration file from the device
 * @param device Handle to the target device
 * @return int 0 on success, negative error code on failure
 */
int lidar_get_calib_file(device_handle device, const char* path);

/**
 * @brief Get device calibration parameters
 * 
 * Retrieves the current calibration parameters from the device.
 * 
 * @param device Handle to the target device
 * @param param Pointer to receive the calibration parameters
 * @return int 0 on success, negative error code on failure
 */
int lidar_get_calibration(device_handle device, lidar_calibration_t* param);

/**
 * @brief Set device calibration parameters
 * 
 * Applies new calibration parameters to the device.
 * 
 * @param device Handle to the target device
 * @param param Pointer to the calibration parameters to set
 * @return int 0 on success, negative error code on failure
 */
int lidar_set_calibration(device_handle device, const lidar_calibration_t *param);

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
int lidar_get_version(device_handle device,lidar_fireware_version_t *version);

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
 * @brief Save the current map to a file on the host. Synchronous, all-in-one API.
 *
 * Internally drives the complete save-map state machine, so callers do not need
 * to coordinate the multi-step protocol themselves:
 *   1) Set custom parameter save_map = 1 on the device (kicks off generation).
 *   2) Poll save_map until the device resets it to 0 (map generation finished).
 *   3) Pull the resulting file via the standard mapping-result transfer (same as
 *      lidar_get_mapping_result).
 *   4) Return the final status to the caller.
 *
 * This is the recommended entry point for saving a map. The lower-level pair
 * (lidar_set_custom_parameter("save_map") + lidar_get_mapping_result) remains
 * available for advanced use cases.
 *
 * 同步保存地图到主机上的文件。一站式 API，内部完成「触发生成 → 等待设备完成 →
 * 拉取文件」的完整流程，调用方无需自行轮询和处理异步状态。
 *
 * @param device         Device handle.
 * @param dest_dir       Host directory to save the file into (must already exist).
 * @param file_name      File name (e.g. "map.bin").
 * @param gen_timeout_ms Maximum time (ms) to wait for the device to finish
 *                       generating the map. Pass 0 to use the default (120000 ms).
 * @return int
 *          0  success, file saved at dest_dir/file_name
 *         -1  invalid arguments (null device/dir, SDK not initialized, etc.)
 *         -2  device is busy with another file transfer
 *         -3  timed out waiting for the device to finish map generation
 *         -4  file transfer stalled or failed (see logs for details)
 *         other negative values are propagated from the underlying transfer.
 */
int lidar_save_map(device_handle device,
                   const char *dest_dir,
                   const char *file_name,
                   uint32_t gen_timeout_ms);

/**
 * @brief Set the image mask file for the device
 *
 * Read & send specified image mask file to device
 *
 * @param device Handle to the target device
 * @param abs_path Absolute path to the image mask file (e.g., mask.png)
 * @return int 0 on success, -1 on failure, -2 if file transfer in progress
 */
 int lidar_set_image_mask(device_handle device, const char* abs_path);

  /**
 * @brief enable encrypted device log
 *
 * enable encrypted device log, save to specified directory
 * Please send to our support when needed
 *
 * @param device Handle to the target device
 * @param dest_dir Destination directory to save the encrypted logs
 * @return int 0 on success, -1 on failure
 */
 int lidar_enable_encrypted_device_log(device_handle device, const char* dest_dir);


/**
 * @brief Set the depth parameters for the device
 * 
 * This function must be called before starting data stream.
 *
 * @param device Handle to the target device
 * @param params Pointer to the depth parameters to set
 * @return int 0 on success, negative error code on failure
 */
int lidar_set_depth_parameter(device_handle device, const lidar_depth_para_t *params);

/**
 * @brief Enable or disable IMU smooth sending feature
 *
 * When enabled, IMU data will be sent at precise intervals (default 400Hz) 
 * using a dedicated high-priority thread to reduce jitter and timing variance.
 * When disabled, IMU data will be sent immediately upon reception.
 *
 * @param enable 1 to enable smooth sending, 0 to disable
 * @return int 0 on success, -1 on failure
 */
int lidar_enable_imu_smooth_sending(int enable);

/**
 * @brief Set IMU smooth sending frequency
 *
 * Set the target frequency for IMU smooth sending. Only effective when 
 * smooth sending is enabled via lidar_enable_imu_smooth_sending().
 *
 * @param frequency_hz Target frequency in Hz (1-1000 Hz, recommended 400 Hz)
 * @return int 0 on success, -1 on failure
 */
int lidar_set_imu_smooth_frequency(uint32_t frequency_hz);

/**
 * @brief Reset the USB connection to the device
 *
 * Performs a USB port reset on the device (libusb_reset_device). This
 * re-enumerates the device on the host without requiring a physical
 * re-plug. The device handle should typically be reopened after the
 * device reconnects.
 *
 * @param device Handle to the target device
 * @return int 0 on success, negative error code on failure
 */
int lidar_reset_usb(device_handle device);

/**
 * @brief Query the current device initial/running state.
 *
 * Returns the latest state reported by the device through heartbeats. The
 * value corresponds to ::lidar_device_initial_state_e (NONE,
 * NOT_INITIALIZED, INITIALIZED, STREAMING, STREAM_STOPPED). Callers that
 * need to wait until the device is fully booted (for example before
 * uploading a relocalization map) should poll this until it becomes
 * LIDAR_DEVICE_STREAMING (or LIDAR_DEVICE_INITIALIZED at minimum).
 *
 *
 * @param state Output pointer that receives the current state. Must not be NULL.
 * @return int 0 on success, negative error code on failure.
 */
int lidar_get_device_state(lidar_device_initial_state_e *state);

/**
 * @brief Send a host-defined pass-through user-data blob to the device.
 *
 * The device forwards data[] as-is to SLAM via shared memory
 * ("user_data_shm"). No acknowledgment is returned from the device
 * (fire-and-forget). An SDK-internal monotonic counter is packed into
 * the on-wire frameId field so SLAM can still distinguish consecutive
 * frames without the caller having to manage an id.
 *
 * Preconditions:
 *  - SDK initialized and the device opened (lidar_open_device).
 *  - SLAM has been started on the device side; otherwise the device
 *    silently drops the frame and logs a warning.
 *
 * Constraints:
 *  - blob != NULL.
 *  - 0 < blob_len <= 8 MiB (8 * 1024 * 1024).
 *  - Sending faster than SLAM can consume causes device-side timeouts
 *    and dropped frames. Pace according to SLAM throughput.
 *
 * @param device   Handle returned by lidar_create_device / lidar_open_device.
 * @param blob     Pointer to user payload.
 * @param blob_len Length in bytes.
 * @return int 0 on success, negative error code on failure.
 */
int lidar_send_user_data(device_handle device,
                         const void *blob, uint32_t blob_len);

/* ---------------------------------------------------------------------
 * Camera AE / AWB control APIs.
 *
 * All four APIs are synchronous (send + wait response) and serialised by
 * an internal mutex (the same mutex protecting other control commands).
 * Return value convention:
 *   == 0   success
 *   >  0   device-side error, see lidar_ae_error_e (400..405 or 0xFF)
 *   <  0   SDK-side error (not initialised, bad argument, USB failure,
 *          response timeout, malformed reply, ...)
 *
 * Wire-level details: sdk/api/Host_USB_AE_Protocol.md.
 * ------------------------------------------------------------------- */

/**
 * @brief Query current AE (auto exposure) state.
 *
 * Sends AE opcode 0x01 and decodes the 25-byte little-endian payload
 * returned by the device-side ae_control service.
 *
 * Output fields and their physical meaning:
 *   exposure_time : current exposure time in seconds (manual range
 *                   0.0001 .. 0.033; in auto mode it varies with
 *                   scene illumination).
 *   gain          : current analog gain (manual range 1.0 .. 64.0;
 *                   higher = brighter but noisier).
 *   iso           : equivalent ISO, typically 100 .. 6400.
 *   brightness    : average frame brightness (0 .. 255).
 *   is_converged  : 1 = AE has settled, 0 = still adjusting.
 *   env_lv        : ambient luminance index, typically 0 .. 15
 *                   (higher = brighter scene).
 *   fps           : actual frame rate, follows dtof_fps config
 *                   (~10 / 14.5 / 29 Hz).
 *
 * @param device Device handle returned by lidar_create_device / lidar_open_device.
 * @param out    Output buffer, must not be NULL.
 * @return See return value convention above.
 */
int lidar_get_ae_info(device_handle device, lidar_ae_info_t *out);

/**
 * @brief Query current AWB (auto white balance) state.
 *
 * Sends AE opcode 0x30 and decodes the 25-byte little-endian payload.
 *
 * Output fields and their physical meaning:
 *   rgain        : R  channel gain (manual range 0.1 .. 4.0).
 *   grgain       : Gr channel gain, always 1.0 (device-fixed).
 *   gbgain       : Gb channel gain, always 1.0 (device-fixed).
 *   bgain        : B  channel gain (manual range 0.1 .. 4.0).
 *   cct          : correlated color temperature in Kelvin
 *                  (typically 2500 .. 8000 K).
 *   ccri         : color temperature deviation index (-50 .. 50,
 *                  signed; 0 means on the Planckian locus).
 *   is_converged : 1 = AWB has settled, 0 = still adjusting.
 *
 * @param device Device handle.
 * @param out    Output buffer, must not be NULL.
 * @return See return value convention above.
 */
int lidar_get_awb_info(device_handle device, lidar_awb_info_t *out);

/**
 * @brief Set AE mode and (in manual mode) exposure/gain.
 *
 * Behaviour:
 *   - mode == LIDAR_CAM_MODE_AUTO   : sends opcode 0x02 only;
 *                                     exposure_time / gain are ignored.
 *   - mode == LIDAR_CAM_MODE_MANUAL : sends opcode 0x03 to switch to
 *                                     manual AE, then opcode 0x06 to
 *                                     apply (exposure_time, gain).
 *
 * Parameter ranges and physical meaning
 * (manual mode only; out-of-range returns rc = 403):
 *
 *   exposure_time : 0.0001 s .. 0.033 s
 *       Sensor exposure time per frame. Longer = brighter but more
 *       motion blur and lower effective fps if it exceeds the frame
 *       period (1/fps). For dtof_fps = 290 (29 Hz, period ~34 ms) the
 *       upper bound 0.033 s is already at the frame limit.
 *
 *   gain          : 1.0 .. 64.0
 *       Analog gain applied to the raw sensor signal. Higher = brighter
 *       output but worse SNR. Typical sweet spot: 1.0 .. 8.0 for daylight,
 *       8.0 .. 32.0 for indoor / dim light, 32.0 .. 64.0 only when image
 *       must be visible at any cost.
 *
 * Recommended starting points by scene:
 *   bright outdoor : exposure 0.001~0.005 s, gain 1.0~2.0
 *   normal indoor  : exposure 0.008~0.015 s, gain 2.0~8.0
 *   dim light      : exposure 0.020~0.030 s, gain 8.0~32.0
 *
 * @param device        Device handle.
 * @param mode          See lidar_cam_mode_e.
 * @param exposure_time Exposure time in seconds (manual mode only).
 * @param gain          Analog gain (manual mode only).
 * @return See return value convention above.
 */
int lidar_set_ae_param(device_handle device, lidar_cam_mode_e mode,
                       float exposure_time, float gain);

/**
 * @brief Set AWB mode and (in manual mode) R/B channel gains.
 *
 * Behaviour:
 *   - mode == LIDAR_CAM_MODE_AUTO   : sends opcode 0x31 only;
 *                                     rgain / bgain are ignored.
 *   - mode == LIDAR_CAM_MODE_MANUAL : sends opcode 0x32 to switch to
 *                                     manual AWB, then opcode 0x33 to
 *                                     apply (rgain, bgain). Gr/Gb are
 *                                     fixed to 1.0 by the device.
 *
 * Parameter ranges and physical meaning
 * (manual mode only; out-of-range returns rc = 403):
 *
 *   rgain : 0.1 .. 4.0
 *       Multiplier on the R channel before color matrix. Higher rgain
 *       relative to bgain shifts the image toward warm (yellow/red).
 *
 *   bgain : 0.1 .. 4.0
 *       Multiplier on the B channel. Higher bgain relative to rgain
 *       shifts the image toward cool (blue).
 *
 * Color-temperature cookbook (approximate):
 *   warm (tungsten, sunset) : rgain ~2.0..2.5, bgain ~1.0..1.2
 *   neutral (daylight D65)  : rgain ~1.5..1.7, bgain ~1.8..2.0
 *   cool (cloudy, fluor.)   : rgain ~1.2..1.4, bgain ~2.2..2.6
 *
 * @param device Device handle.
 * @param mode   See lidar_cam_mode_e.
 * @param rgain  R channel gain (manual mode only).
 * @param bgain  B channel gain (manual mode only).
 * @return See return value convention above.
 */
int lidar_set_awb_param(device_handle device, lidar_cam_mode_e mode,
                        float rgain, float bgain);

#ifdef __cplusplus
}
#endif

#endif // LIDAR_API_H
