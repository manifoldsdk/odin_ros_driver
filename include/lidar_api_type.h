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

/**
 * @brief Data types for lidar_data_callback_t
 * 
 * Each type corresponds to a specific stream format in lidar_data_t.stream (capture_Image_List_t).
 * 
 * ┌─────────────────────────────────────────────────────────────────────────────────────────────┐
 * │ LIDAR_DT_RAW_RGB                                                                            │
 * │   imageCount: 1                                                                             │
 * │   imageList[0]: NV12 image data                                                             │
 * │     - pAddr: uint8_t* (Y plane followed by UV plane)                                        │
 * │     - width: 1536, height: 1280                                                             │
 * │     - length: width * height * 3 / 2 bytes                                                  │
 * ├─────────────────────────────────────────────────────────────────────────────────────────────┤
 * │ LIDAR_DT_RAW_IMU                                                                            │
 * │   imageCount: 1                                                                             │
 * │   imageList[0]: IMU data                                                                    │
 * │     - pAddr: imu_convert_data_t*                                                            │
 * │       - accel[3]: float (m/s^2)                                                             │
 * │       - gyro[3]: float (rad/s)                                                              │
 * │       - stamp: uint64_t (ns)                                                                │
 * │       - sequence: uint64_t                                                                  │
 * │     - length: sizeof(imu_convert_data_t)                                                    │
 * ├─────────────────────────────────────────────────────────────────────────────────────────────┤
 * │ LIDAR_DT_RAW_DTOF                                                                           │
 * │   imageCount: 4                                                                             │
 * │   Resolution: 256 x 192                                                                     │
 * │   imageList[0]: Depth image                                                                 │
 * │     - pAddr: float* (depth in meters)                                                       │
 * │     - length: 256 * 192 * sizeof(float)                                                     │
 * │   imageList[1]: Point cloud XYZ                                                             │
 * │     - pAddr: float* (x,y,z interleaved)                                                     │
 * │     - length: 256 * 192 * 3 * sizeof(float)                                                 │
 * │   imageList[2]: Confidence                                                                  │
 * │     - pAddr: uint8_t*                                                                       │
 * │     - length: 256 * 192 * sizeof(uint8_t)                                                   │
 * │   imageList[3]: Intensity/Reflectivity                                                      │
 * │     - pAddr: uint16_t*                                                                      │
 * │     - length: 256 * 192 * sizeof(uint16_t)                                                  │
 * ├─────────────────────────────────────────────────────────────────────────────────────────────┤
 * │ LIDAR_DT_SLAM_CLOUD                                                                         │
 * │   imageCount: 1                                                                             │
 * │   imageList[0]: SLAM point cloud (XYZRGBA, fixed-point on the wire)                         │
 * │     - pAddr: slam_cloud_point_t*  (7 * int32_t per point, see struct below)                 │
 * │     - length: num_points * sizeof(slam_cloud_point_t)  ( == num_points * 28 bytes )         │
 * │     xyz are stored in 0.1 mm units: meters = xyz * SLAM_CLOUD_XYZ_TO_M                      │
 * ├─────────────────────────────────────────────────────────────────────────────────────────────┤
 * │ LIDAR_DT_SLAM_ODOMETRY / LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ / LIDAR_DT_SLAM_ODOMETRY_TF        │
 * │   imageCount: 1                                                                             │
 * │   imageList[0]: Odometry data                                                               │
 * │     - pAddr: ros_odom_convert_complete_t*                                                   │
 * │       - timestamp_ns: uint64_t                                                              │
 * │       - pos[3]: int64_t (x,y,z in μm, divide by 1e6 for meters)                             │
 * │       - orient[4]: int64_t (quaternion x,y,z,w, divide by 1e6)                              │
 * │       - linear_velocity[3]: int64_t                                                         │
 * │       - angular_velocity[3]: int64_t                                                        │
 * │       - pose_cov[36]: double                                                                │
 * │       - twist_cov[36]: double                                                               │
 * │     - length: sizeof(ros_odom_convert_complete_t)                                           │
 * ├─────────────────────────────────────────────────────────────────────────────────────────────┤
 * │ LIDAR_DT_DEV_STATUS                                                                         │
 * │   imageCount: 1                                                                             │
 * │   imageList[0]: Device status                                                               │
 * │     - pAddr: lidar_device_status_t*                                                         │
 * │     - length: sizeof(lidar_device_status_t)                                                 │
 * ├─────────────────────────────────────────────────────────────────────────────────────────────┤
 * │ LIDAR_DT_NTP                                                                                │
 * │   imageCount: 1                                                                             │
 * │   imageList[0]: PTP/NTP sync data                                                           │
 * │     - pAddr: ptp_sync_data_t*                                                               │
 * │       - delay: double                                                                       │
 * │       - offset: double                                                                      │
 * │     - length: sizeof(ptp_sync_data_t)                                                       │
 * └─────────────────────────────────────────────────────────────────────────────────────────────┘
 */
typedef enum {
    LIDAR_DT_NONE = 0,              /**< No data */
    LIDAR_DT_RAW_RGB,               /**< RGB image (NV12 format, 1536x1280) */
    LIDAR_DT_RAW_IMU,               /**< IMU data (imu_convert_data_t) */
    LIDAR_DT_RAW_DTOF,              /**< DTOF raw data (depth + xyz + confidence + intensity, 256x192) */
    LIDAR_DT_SLAM_CLOUD,            /**< SLAM point cloud (XYZRGBA) */
    LIDAR_DT_SLAM_ODOMETRY,         /**< SLAM odometry (ros_odom_convert_complete_t) */
    LIDAR_DT_DEV_STATUS,            /**< Device status (lidar_device_status_t) */
    LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ,/**< High frequency odometry (ros_odom_convert_complete_t) */
    LIDAR_DT_SLAM_ODOMETRY_TF,      /**< Map-Odom TF transform (ros_odom_convert_complete_t) */
    LIDAR_DT_SLAM_WIWC,             /**< WIWC odometry */
    LIDAR_DT_NTP                    /**< PTP/NTP sync data (ptp_sync_data_t) */
} lidar_data_type_e;

typedef struct {
    int8_t serial[LIDAR_SERIAL_MAX];
    int8_t model[LIDAR_MODEL_MAX];
    bool online;
    uint32_t initial_state;
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
    int64_t pos[3];       // x, y, z in μm
    int64_t orient[4];    // quaternion x, y, z, w in 1e6 precision
    int64_t linear_velocity[3];
    int64_t angular_velocity[3];
    double pose_cov[36];
    double twist_cov[36];
} ros_odom_convert_complete_t;

typedef struct {
    double delay;
    double offset;
} ptp_sync_data_t;

/* ---------------------------------------------------------------------
 * SLAM cloud wire format.
 *
 * One LIDAR_DT_SLAM_CLOUD point on the bus is a 7 * int32_t record:
 *     xyz[0..2] : x, y, z in 0.1 mm fixed-point.
 *                 meters = xyz * SLAM_CLOUD_XYZ_TO_M.
 *     rgba[0..3]: r, g, b, a; each stored in the low byte of an int32_t.
 *
 * Consumers (SDK hooks, ROS driver, etc.) should reference this struct
 * and the scale macro below as the single source of truth rather than
 * re-hardcoding the stride or the divisor.
 * ------------------------------------------------------------------- */
#define SLAM_CLOUD_XYZ_TO_M   (1.0e-4)   /* device 0.1mm units -> meters */
#define SLAM_CLOUD_XYZ_FROM_M (1.0e4)    /* meters -> device 0.1mm units */

typedef struct {
    int32_t xyz[3];    /* x, y, z in 0.1 mm fixed-point */
    int32_t rgba[4];   /* r, g, b, a; only low byte of each is meaningful */
} slam_cloud_point_t;

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
    int major;
    int minor;
    int patch;
}lidar_version_t;

typedef struct {
    lidar_version_t kernel_version;
    lidar_version_t mcu_version;
    lidar_version_t soc_version;
    lidar_version_t Daemon_proc_version;
    lidar_version_t slam_version;
} lidar_fireware_version_t;

/**
 * @brief RGB image sensor frame rate
 * 
 */
 typedef struct{

    int configured_odr;/*rgb image sensor frame rate, offset: */
    int tx_odr;/*The actual rgb image sensor frame rate, offset: */

} lidar_rgb_sensor_status_t;

/**
 * @brief DTOF Lidar frame rate
 * 
 */
typedef struct{

    int configured_odr;/* dtof lidar sensor frame rate, offset: */
    int tx_odr;/*The actual dtof lidar sensor frame rate, offset: */
    int subframe_odr;/*DTOF 6行为一组 这个是组间隔时间*/
    short tx_temp;/* dtof lidar tx temp  offset: */
    short rx_temp;/* dtof lidar rx temp offset:  */

} lidar_dtof_sensor_status_t;

/**
 * @brief IMU Sensor
 * 
 */
typedef struct{

    int configured_odr;
    int tx_odr;

} lidar_imu_sensor_status_t;

typedef struct{

    int package_temp;/*SOC整体温度*/
    // int bigcore_temp;/*大核集群温度：4*A76*/
    // int littlecore_temp;/*小核集群温度：4*A53*/
    int cpu_temp;
    int center_temp;/*SOC中心温度：4*A53*/
    int gpu_temp;/* GPU模块温度 */
    int npu_temp;/* NPU模块温度 */

} lidar_soc_thermal_t;
typedef struct
{
    double uptime_seconds;
    lidar_soc_thermal_t soc_thermal; /*offset: 0*/

    int cpu_use_rate[8];/*cpu 使用率,offset:  */
    int ram_use_rate;/*运行内存使用率 ,offset:  */

    lidar_rgb_sensor_status_t rgb_sensor;
    lidar_dtof_sensor_status_t dtof_sensor;
    lidar_imu_sensor_status_t imu_sensor;

    int slam_cloud_tx_odr; /* Actual frame rate of slam cloud offset:  */
    int slam_odom_tx_odr; /* Actual frame rate of slam odom offset:  */
    int slam_odom_highfreq_tx_odr; /* Actual frame rate of slam odom offset:  */

} lidar_device_status_t;

typedef enum {
    LIDAR_DEVICE_NONE = 0,
    LIDAR_DEVICE_NOT_INITIALIZED,
    LIDAR_DEVICE_INITIALIZED,
    LIDAR_DEVICE_STREAMING,
    LIDAR_DEVICE_STREAM_STOPPED,
} lidar_device_initial_state_e;

typedef enum {
    LIDAR_DEPTH_ODR_10HZ   = 0,
    LIDAR_DEPTH_ODR_14_5HZ = 1,
    LIDAR_DEPTH_ODR_29HZ   = 2,
} lidar_depth_odr_e;

typedef struct {
    lidar_depth_odr_e odr;
} lidar_depth_para_t;

/* ---------------------------------------------------------------------
 * Camera AE / AWB control types.
 *
 * The host SDK forwards AE/AWB requests through the USB control channel
 * (CMD_CODE_CONTROL_CMD + SYS_CONTROL_AE_UDP). The device-side lydapp
 * relays them via UDP loopback to its ISP service. See
 * sdk/api/Host_USB_AE_Protocol.md for the wire-level details.
 * ------------------------------------------------------------------- */

/**
 * @brief AE/AWB control mode used by lidar_set_ae_param / lidar_set_awb_param.
 *
 * - AUTO   : the device runs its own AE/AWB convergence loop. The two
 *            float parameters of the corresponding Set call are ignored.
 * - MANUAL : the device locks AE/AWB and applies the user-supplied
 *            (exposure_time, gain) or (rgain, bgain). Out-of-range
 *            values are rejected with rc = 403
 *            (LIDAR_AE_PARAM_OUT_OF_RANGE).
 */
typedef enum {
    LIDAR_CAM_MODE_AUTO   = 0, /**< switch to auto AE / AWB */
    LIDAR_CAM_MODE_MANUAL = 1, /**< switch to manual AE / AWB and apply params */
} lidar_cam_mode_e;

/**
 * @brief Current AE status returned by lidar_get_ae_info().
 *
 * Field-by-field meaning and typical range:
 *
 *   exposure_time : current sensor exposure time, in seconds.
 *                   Manual-mode valid range: 0.0001 .. 0.033.
 *                   In auto mode varies with scene illumination.
 *   gain          : current analog gain (linear, not dB).
 *                   Manual-mode valid range: 1.0 .. 64.0.
 *                   Higher value = brighter output but worse SNR.
 *   iso           : equivalent ISO speed, typically 100 .. 6400.
 *                   Derived from gain; informational only.
 *   brightness    : average frame brightness in [0, 255]. AE target
 *                   converges toward a mid-range value.
 *   is_converged  : 1 = AE has settled, 0 = still adjusting.
 *   env_lv        : ambient luminance index, typically 0 .. 15
 *                   (higher = brighter scene).
 *   fps           : actual frame rate in Hz, follows dtof_fps config
 *                   (~10 / 14.5 / 29).
 */
typedef struct {
    float    exposure_time; /**< current exposure time (s), 0.0001..0.033 */
    float    gain;          /**< current analog gain, 1.0..64.0 */
    int32_t  iso;           /**< equivalent ISO, ~100..6400 */
    float    brightness;    /**< average frame brightness, 0..255 */
    uint8_t  is_converged;  /**< 1 = AE converged, 0 = not converged */
    float    env_lv;        /**< ambient luminance level, ~0..15 */
    float    fps;           /**< current frame rate (Hz) */
} lidar_ae_info_t;

/**
 * @brief Current AWB status returned by lidar_get_awb_info().
 *
 * Field-by-field meaning and typical range:
 *
 *   rgain        : R channel gain. Manual-mode valid range: 0.1 .. 4.0.
 *                  Raising rgain relative to bgain shifts the image
 *                  toward warm (yellow/red).
 *   grgain       : Gr channel gain. Device-fixed at 1.0, not adjustable.
 *   gbgain       : Gb channel gain. Device-fixed at 1.0, not adjustable.
 *   bgain        : B channel gain. Manual-mode valid range: 0.1 .. 4.0.
 *                  Raising bgain relative to rgain shifts the image
 *                  toward cool (blue).
 *   cct          : correlated color temperature in Kelvin, typically
 *                  2500 .. 8000 K.
 *   ccri         : color temperature deviation index, signed value
 *                  roughly in -50 .. 50; 0 = on the Planckian locus.
 *   is_converged : 1 = AWB has settled, 0 = still adjusting.
 */
typedef struct {
    float    rgain;         /**< R channel gain, 0.1..4.0 */
    float    grgain;        /**< Gr channel gain, device-fixed 1.0 */
    float    gbgain;        /**< Gb channel gain, device-fixed 1.0 */
    float    bgain;         /**< B channel gain, 0.1..4.0 */
    float    cct;           /**< color temperature (K), ~2500..8000 */
    float    ccri;          /**< color temperature deviation, ~-50..50 */
    uint8_t  is_converged;  /**< 1 = AWB converged, 0 = not converged */
} lidar_awb_info_t;

/**
 * @brief AE/AWB device-side error codes.
 *
 * Mapped to positive return values of lidar_get_ae_info / lidar_set_ae_param /
 * lidar_get_awb_info / lidar_set_awb_param when the device replies with
 * CMD_CODE_FAIL. See Host_USB_AE_Protocol.md section 6.
 */
typedef enum {
    LIDAR_AE_OK                   = 0,
    LIDAR_AE_BAD_REQUEST          = 400, /**< payload too short */
    LIDAR_AE_UNSUPPORTED_OPCODE   = 401,
    LIDAR_AE_BAD_PARAM_LEN        = 402,
    LIDAR_AE_PARAM_OUT_OF_RANGE   = 403,
    LIDAR_AE_SOCKET_ERROR         = 404,
    LIDAR_AE_NO_RESPONSE          = 405, /**< ae_control UDP timeout */
    LIDAR_AE_UNKNOWN_OPCODE       = 0xFF,/**< status byte from ae_control */
} lidar_ae_error_e;

#ifdef __cplusplus
}
#endif

#endif
