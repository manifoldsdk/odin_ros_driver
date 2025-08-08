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
    LIDAR_DT_NONE               = 0,
    LIDAR_DT_RAW_RGB            = 1 << 1,
    LIDAR_DT_RAW_IMU            = 1 << 2,
    LIDAR_DT_RAW_DTOF           = 1 << 3,
    LIDAR_DT_SLAM_CLOUD         = 1 << 4,
    LIDAR_DT_SLAM_ODOMETRY      = 1 << 5,
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

typedef struct icm_6aixs_data_t {
	int16_t aacx;
	int16_t aacy; 
	int16_t aacz;
	int16_t gyrox;
	int16_t gyroy;
	int16_t gyroz;
	uint8_t valid;
	uint32_t nums;
	uint8_t fsync_pack;
	uint16_t interval;
	uint64_t stamp;
} icm_6aixs_data_t;

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

#ifdef __cplusplus
}
#endif

#endif
