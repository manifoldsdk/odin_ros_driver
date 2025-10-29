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
#ifndef LIDAR_TYPES_H
#define LIDAR_TYPES_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_SERIAL_MAX 64
#define LIDAR_MODEL_MAX  64
#define LIDAR_IP_MAX     64

typedef void * device_handle;

typedef enum {
    LIDAR_LOG_ERROR = 0,
    LIDAR_LOG_WARN,
    LIDAR_LOG_INFO,
    LIDAR_LOG_DEBUG,
} lidar_log_level_e;

typedef enum {
    LIDAR_OTA_ALGORITHM,
    LIDAR_OTA_FIRMWARE,
    LIDAR_OTA_SCRIPT,
    LIDAR_OTA_CALIBRATION
} lidar_ota_type_e;

typedef enum {
    LIDAR_MODE_RAW,
    LIDAR_MODE_SLAM,
} lidar_mode_e;

typedef enum {
    LIDAR_DT_NONE = 0,
    LIDAR_DT_RAW_RGB,
    LIDAR_DT_RAW_IMU,
    LIDAR_DT_RAW_DTOF,
    LIDAR_DT_SLAM_CLOUD,
    LIDAR_DT_SLAM_ODOMETRY,
    LIDAR_DT_DEV_STATUS,
    LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ,
    LIDAR_DT_SLAM_ODOMETRY_TF,
} lidar_data_type_e;

typedef struct {
    int8_t serial[LIDAR_SERIAL_MAX];
    int8_t model[LIDAR_MODEL_MAX];
    bool online;
} lidar_device_info_t;

typedef struct {
    float x, y, z;
    float intensity;
} lidar_point_t;


typedef struct {
    float intrinsics[9];
    float extrinsics[16];
} lidar_calibration_t;


#define DEVICE_MAX_CH_NUMBER 4

typedef struct {
    uint64_t timestamp_ns;
    int64_t pos[3];
    int64_t orient[4];
} ros2_odom_convert_t;

typedef struct {
    uint64_t timestamp_ns;
    int64_t pos[3];
    int64_t orient[4];
    int64_t linear_velocity[3];
    int64_t angular_velocity[3];
    int64_t cov[3 * 3 * 2];
} ros_odom_convert_complete_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint64_t stamp;
    uint64_t sequence;
} imu_convert_data_t;

 typedef struct {
    uint32_t length;
    uint64_t sequence;
    uint64_t timestamp;
    uint64_t interval;
    void* pAddr;
    uint32_t width;
    uint32_t height;
} buffer_List_t;

typedef struct capture_Image_List_t {
    uint32_t imageCount;
    buffer_List_t imageList[DEVICE_MAX_CH_NUMBER];
} capture_Image_List_t;

typedef struct {
    uint32_t type;
    capture_Image_List_t stream;
} lidar_data_t;

typedef void (*lidar_device_callback_t)(const lidar_device_info_t* device, bool attach);
typedef void (*lidar_data_callback_t)(const lidar_data_t *data, void *user_data);

typedef struct {
    lidar_data_callback_t data_callback;
    void *user_data;
} lidar_data_callback_info_t;

typedef struct {
    char mcu_version[64];
    char sys_version[64];
    char slam_version[64];
    char dev_app_version[64];
    char host_app_version[64];
} lidar_version_t;

/**
 * @brief RGB image sensor frame rate
 * 
 */
 typedef struct{

    int configured_odr; /* rgb image sensor configured output data rate */
    int tx_odr;         /* rgb image sensor tx output data rate */

} lidar_rgb_sensor_status_t;

/**
 * @brief DTOF Lidar frame rate
 * 
 */
typedef struct{

    int configured_odr; /* dtof lidar sensor configured output data rate */
    int tx_odr;         /* dtof lidar sensor tx output data rate */
    int subframe_odr;   /* dtof lidar sensor subframe output data rate */
    short tx_temp;      /* dtof lidar tx module temp */
    short rx_temp;      /* dtof lidar rx module temp */

} lidar_dtof_sensor_status_t;

/**
 * @brief IMU Sensor
 * 
 */
typedef struct{

    int configured_odr; /* imu sensor configured output data rate */
    int tx_odr;         /* imu sensor tx output data rate */

} lidar_imu_sensor_status_t;

typedef struct{

    int package_temp;   /* soc package temp */
    int cpu_temp;       /* cpu temp */
    int center_temp;    /* center temp */
    int gpu_temp;       /* gpu temp */
    int npu_temp;       /* npu temp */

} lidar_soc_thermal_t;
typedef struct
{
    lidar_soc_thermal_t soc_thermal; 

    int cpu_use_rate[8];              /* cpu usage rate */
    int ram_use_rate;                 /* ram usage rate */

    lidar_rgb_sensor_status_t rgb_sensor;
    lidar_dtof_sensor_status_t dtof_sensor;
    lidar_imu_sensor_status_t imu_sensor;

    int slam_cloud_tx_odr;          /* slam cloud tx output data rate */
    int slam_odom_tx_odr;           /* slam odom tx output data rate */
    int slam_odom_highfreq_tx_odr;  /* slam odom high freq tx output data rate */

} lidar_device_status_t;

#ifdef __cplusplus
}
#endif

#endif
