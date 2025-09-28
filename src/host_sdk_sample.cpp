#include "host_sdk_sample.h"
#include "yaml_parser.h"
#include "rawCloudRender.h"
#include <filesystem> 
#include <thread>
#include <string>
#include <stdexcept>
#include <atomic>
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include <deque> 
#include <unistd.h> 
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <cstdio>
#include <array>
// #include <yaml-cpp/yaml.h>
#include <iomanip>
#ifdef ROS2
    #include <ament_index_cpp/get_package_share_directory.hpp>
    #include <rclcpp/rclcpp.hpp>
#else
    #include <ros/package.h>
    #include <ros/ros.h> 
#endif
#define ros_driver_version "0.5.0"
// Global variable declarations
static device_handle odinDevice = nullptr;
static std::atomic<bool> deviceConnected(false);
static std::atomic<bool> deviceDisconnected(false);  // Device disconnection flag
static std::mutex device_mutex;                      // Device operation mutex lock
static std::atomic<bool> g_connection_timeout(false);
static std::atomic<bool> g_usb_version_error(false);
#ifdef ROS2
    std::shared_ptr<MultiSensorPublisher> g_ros_object = nullptr;
#else
    MultiSensorPublisher* g_ros_object = nullptr;
#endif

int g_log_level = LOG_LEVEL_INFO;  
int g_show_fps = 0;  // FPS display toggle control

static std::mutex g_rgb_mutex;
static std::shared_ptr<cv::Mat> g_latest_bgr;
static uint64_t g_latest_rgb_timestamp = 0;
static bool g_has_rgb = false;
static capture_Image_List_t g_latest_rgb;
static bool g_renderer_initialized = false;
static std::shared_ptr<rawCloudRender> g_renderer = nullptr;
std::string calib_file_ = "";

 // usb device
static std::string TARGET_VENDOR = "2207";
static std::string TARGET_PRODUCT = "0019";
// Global configuration variables
int g_sendrgb = 1;
int g_sendimu = 1;
int g_senddtof = 1;
int g_sendodom = 1;
int g_sendcloudslam = 0;
int g_sendcloudrender = 0;
int g_sendrgb_compressed = 0;
int g_sendrgb_undistort = 0;
int g_record_data = 0;
int g_devstatus_log = 0;

const char* DEV_STATUS_CSV_FILE = "dev_status.csv";
FILE* dev_status_csv_file = nullptr;

typedef struct  {
    struct timespec start = {0, 0};
    double frame_count = 0.0;
    std::atomic<double> fps;
} fpsHandle;

static void sensor_fps(fpsHandle* handle, const char* name, bool print = false) 
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (handle->start.tv_sec == 0 && handle->start.tv_nsec == 0) {
        handle->start = now;
    }

    handle->frame_count += 1.0;

    double elapsed = (now.tv_sec - handle->start.tv_sec)
                   + (now.tv_nsec - handle->start.tv_nsec) / 1e9;

    if (elapsed >= 1.0) {
        handle->fps.store(handle->frame_count / elapsed);
        if (print) {
            #ifdef ROS2
                RCLCPP_INFO(rclcpp::get_logger("device_cb"), "%s FPS: %f", name, handle->fps.load());
            #else
                ROS_INFO("%s FPS: %f", name, handle->fps.load());
            #endif
        }
        handle->frame_count = 0;
        handle->start = now;
    }
}

static fpsHandle rgb_rx_fps;
static fpsHandle dtof_rx_fps;
static fpsHandle imu_rx_fps;
static fpsHandle slam_cloud_rx_fps;
static fpsHandle slam_odom_rx_fps;
static fpsHandle slam_odom_highfreq_rx_fps;

class RosNodeControlImpl : public RosNodeControlInterface {
    public:
        void setDtofSubframeODR(int interval) override {
            dtof_subframe_interval_time = interval;
        }
        
        int getDtofSubframeODR() const override {
            return dtof_subframe_interval_time;
        }
    
    private:
        int dtof_subframe_interval_time = 0;
    };
    
static RosNodeControlImpl g_rosNodeControlImpl;

RosNodeControlInterface* getRosNodeControl() {
    return &g_rosNodeControlImpl;
}

void clear_all_queues();

// detect USB3.0
bool isUsb3OrHigher(const std::string& vendorId, const std::string& productId) {
    std::string command = "lsusb -d " + vendorId + ":" + productId + " -v | grep 'bcdUSB'";
    
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    if (result.empty()) {
        #ifdef ROS2
            RCLCPP_ERROR(rclcpp::get_logger("usb_check"), "Failed to get USB version information");
        #else
            ROS_ERROR("Failed to get USB version information");
        #endif
        return false;
    }
    
    // find bcdUSB
    size_t pos = result.find("bcdUSB");
    if (pos == std::string::npos) {
        #ifdef ROS2
            RCLCPP_ERROR(rclcpp::get_logger("usb_check"), "bcdUSB field not found in lsusb output");
        #else
            ROS_ERROR("bcdUSB field not found in lsusb output");
        #endif
        return false;
    }
    
    std::string versionStr = result.substr(pos + 7); // "bcdUSB" + space
    float version = std::stof(versionStr);
    
    #ifdef ROS2
        RCLCPP_INFO(rclcpp::get_logger("usb_check"), "Detected USB version: %.1f", version);
    #else
        ROS_INFO("Detected USB version: %.1f", version);
    #endif
    return version >= 3.0;
}

bool isUsbDevicePresent(const std::string& vendorId, const std::string& productId) {
    std::ifstream devicesList("/sys/bus/usb/devices");
    if (devicesList.is_open()) {
        std::string line;
        while (std::getline(devicesList, line)) {
            if (line.find('.') != std::string::npos) continue;
            if (line.empty()) continue;
            
            std::string vendorPath = "/sys/bus/usb/devices/" + line + "/idVendor";
            std::ifstream vendorFile(vendorPath);
            if (vendorFile.is_open()) {
                std::string vendorContent;
                if (std::getline(vendorFile, vendorContent)) {
                    vendorContent.erase(vendorContent.find_last_not_of(" \n\r\t") + 1);
                    
                    std::string productPath = "/sys/bus/usb/devices/" + line + "/idProduct";
                    std::ifstream productFile(productPath);
                    if (productFile.is_open()) {
                        std::string productContent;
                        if (std::getline(productFile, productContent)) {
                            productContent.erase(productContent.find_last_not_of(" \n\r\t") + 1);
                            
                            if (vendorContent == vendorId && productContent == productId) {
                                return true;
                            }
                        }
                        productFile.close();
                    }
                }
                vendorFile.close();
            }
        }
        devicesList.close();
    }
    return false;
}
// Convert calib.yaml to cam_in_ex.txt
static bool convert_calib_to_cam_in_ex(const std::string& calib_path, const std::filesystem::path& out_path) {
    try {
        if (calib_path.empty()) {
            #ifdef ROS2
                RCLCPP_WARN(rclcpp::get_logger("device_cb"), "calib_file_ is empty, skip writing cam_in_ex.txt");
            #else
                ROS_WARN("calib_file_ is empty, skip writing cam_in_ex.txt");
            #endif
            return false;
        }

        YAML::Node root = YAML::LoadFile(calib_path);

        // Read Tcl_0 matrix (16 values)
        std::array<double, 16> Tcl{};
        YAML::Node tcl = root["Tcl_0"];
        for (size_t i = 0; i < 16; ++i) {
            if (tcl && tcl.IsSequence() && i < tcl.size()) {
                Tcl[i] = tcl[i].as<double>();
            } else {
                // Default last row to [0,0,0,1] if missing, others 0
                Tcl[i] = (i == 15) ? 1.0 : 0.0;
            }
        }

        // Read cam_0 parameters (with defaults)
        YAML::Node cam0 = root["cam_0"];
        auto get_i = [&](const char* key, int def) -> int {
            return (cam0 && cam0[key]) ? cam0[key].as<int>() : def;
        };
        auto get_d = [&](const char* key, double def) -> double {
            return (cam0 && cam0[key]) ? cam0[key].as<double>() : def;
        };

        int image_width = get_i("image_width", 0);
        int image_height = get_i("image_height", 0);
        double k2 = get_d("k2", 0.0);
        double k3 = get_d("k3", 0.0);
        double k4 = get_d("k4", 0.0);
        double k5 = get_d("k5", 0.0);
        double k6 = get_d("k6", 0.0);
        double k7 = get_d("k7", 0.0);
        double p1 = get_d("p1", 0.0);
        double p2 = get_d("p2", 0.0);
        double A11 = get_d("A11", 0.0);
        double A12 = get_d("A12", 0.0);
        double A22 = get_d("A22", 0.0);
        double u0 = get_d("u0", 0.0);
        double v0 = get_d("v0", 0.0);

        // Ensure parent directory exists
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);

        // Truncate file then write content
        std::ofstream ofs(out_path, std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to open cam_in_ex.txt for write: %s", out_path.string().c_str());
            #else
                ROS_ERROR("Failed to open cam_in_ex.txt for write: %s", out_path.string().c_str());
            #endif
            return false;
        }

        auto fmt = [](double v) {
            std::ostringstream ss; ss.setf(std::ios::fixed); ss << std::setprecision(6) << v; return ss.str();
        };

        // Write Tcl_0 with line breaks every 4 elements
        ofs << "Tcl_0: [";
        for (int i = 0; i < 16; ++i) {
            if (i > 0) {
                ofs << ", ";
                if (i % 4 == 0) ofs << "\n        ";
            }
            ofs << fmt(Tcl[i]);
        }
        ofs << "]\n";

        // Write cam_0 block
        ofs << "cam_0: \n";
        ofs << "   image_width: " << image_width << "\n";
        ofs << "   image_height: " << image_height << "\n";
        ofs << "   k2: " << fmt(k2) << "\n";
        ofs << "   k3: " << fmt(k3) << "\n";
        ofs << "   k4: " << fmt(k4) << "\n";
        ofs << "   k5: " << fmt(k5) << "\n";
        ofs << "   k6: " << fmt(k6) << "\n";
        ofs << "   k7: " << fmt(k7) << "\n";
        ofs << "   p1: " << fmt(p1) << "\n";
        ofs << "   p2: " << fmt(p2) << "\n";
        ofs << "   A11: " << fmt(A11) << "\n";
        ofs << "   A12: " << fmt(A12) << "\n";
        ofs << "   A22: " << fmt(A22) << "\n";
        ofs << "   u0: " << fmt(u0) << "\n";
        ofs << "   v0: " << fmt(v0) << "\n";

        ofs.flush();

        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Wrote cam_in_ex.txt to: %s", out_path.string().c_str());
        #else
            ROS_INFO("Wrote cam_in_ex.txt to: %s", out_path.string().c_str());
        #endif
        return true;
    } catch (const std::exception& e) {
        #ifdef ROS2
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to convert calib.yaml: %s", e.what());
        #else
            ROS_ERROR("Failed to convert calib.yaml: %s", e.what());
        #endif
        return false;
    }
}

// Get package share path
std::string get_package_share_path(const std::string& package_name) {
#ifdef ROS2
    try {
        return ament_index_cpp::get_package_share_directory(package_name);
    } catch (const std::exception& e) {
        throw std::runtime_error("Package not found: " + std::string(e.what()));
    }
#else
    try {
        return ros::package::getPath(package_name);
    } catch (const ros::InvalidNameException& e) {
        throw std::runtime_error("Package not found: " + std::string(e.what()));
    }
#endif
}

std::string get_package_source_directory() {
    // 获取当前源文件的绝对路径
    std::filesystem::path current_file(__FILE__);
    
    // 回溯到包根目录（包含package.xml的目录）
    auto path = current_file.parent_path();
    while (!path.empty() && !std::filesystem::exists(path / "package.xml")) {
        path = path.parent_path();
    }
    
    if (path.empty()) {
        throw std::runtime_error("Failed to locate package root directory");
    }
    
    return path.string();
}


std::string get_package_path(const std::string& package_name) {
    #ifdef ROS2
        return ament_index_cpp::get_package_share_directory(package_name);
    #else
        return ros::package::getPath(package_name);
    #endif
}

// Clear all queues
void clear_all_queues() {
    // Reset state variables
    g_latest_bgr.reset();
    g_latest_rgb_timestamp = 0;
    g_has_rgb = false;
}

// Lidar data callback
static void lidar_data_callback(const lidar_data_t *data, void *user_data)
{
    // If device is not connected, ignore all data
    if (!deviceConnected) {
        return;
    }
    
    device_handle *dev_handle = static_cast<device_handle *>(user_data);
    if(!dev_handle || !data) {
        printf("Invalid device handle or data.\n");
        return;
    }
    imu_convert_data_t *imudata = nullptr;
    lidar_device_status_t *dev_info_data;
    
    switch(data->type) {
        case LIDAR_DT_NONE:
            printf("empty lidar data type: %x\n", data->type);
            break;
        case LIDAR_DT_RAW_RGB:
            if (g_sendrgb) {
                g_ros_object->publishRgb((capture_Image_List_t *)&data->stream);
            }
            sensor_fps(&rgb_rx_fps, "rgb_rx");
            break;
        case LIDAR_DT_RAW_IMU:
            if (g_sendimu) {
                imudata = (imu_convert_data_t *)data->stream.imageList[0].pAddr;
                g_ros_object->publishImu(imudata);
            }
            sensor_fps(&imu_rx_fps, "imu_rx");
            break;
        case LIDAR_DT_RAW_DTOF:
            if (g_senddtof ) {
                g_ros_object->publishIntensityCloud((capture_Image_List_t *)&data->stream, 1);
            }
            sensor_fps(&dtof_rx_fps, "dtof_rx");
            break;
        case LIDAR_DT_SLAM_CLOUD:
            if (g_sendcloudslam) {
                g_ros_object->publishPC2XYZRGBA((capture_Image_List_t *)&data->stream, 0);
            }
            sensor_fps(&slam_cloud_rx_fps, "slam_cloud_rx");
            break;
        case LIDAR_DT_SLAM_ODOMETRY:
            if (g_sendodom) {
                g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream, false);
            }
            sensor_fps(&slam_odom_rx_fps, "slam_odom_rx");
            break;
        case LIDAR_DT_DEV_STATUS:
            dev_info_data = (lidar_device_status_t *)data->stream.imageList[0].pAddr;
            if (g_devstatus_log) {
                if (dev_status_csv_file) {
                    // append the data row
                    int rc = 0;
                    rc = std::fprintf(dev_status_csv_file, "%d,%d,%d,%d,%d,%d,", // %.0f
                                            // get_uptime_seconds(),
                                            0,
                                            dev_info_data->soc_thermal.package_temp,
                                            dev_info_data->soc_thermal.cpu_temp,
                                            dev_info_data->soc_thermal.center_temp,
                                            dev_info_data->soc_thermal.gpu_temp,
                                            dev_info_data->soc_thermal.npu_temp);
                    if (rc < 0) { 
                        printf("Failed to write to dev_status_csv_file\n");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%d,%d,", 
                        dev_info_data->dtof_sensor.tx_temp,
                        dev_info_data->dtof_sensor.rx_temp);
                    if (rc < 0) { 
                        printf("Failed to write to dev_status_csv_file\n");
                    }

                    for (int i = 0; i < 8; i++) {
                        rc = std::fprintf(dev_status_csv_file, "%d,", dev_info_data->cpu_use_rate[i]);
                    }
                    rc = std::fprintf(dev_status_csv_file, "%d,", dev_info_data->ram_use_rate);

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,", 
                        ((float)dev_info_data->rgb_sensor.configured_odr)/1000,
                        ((float)dev_info_data->rgb_sensor.tx_odr)/1000,
                        (rgb_rx_fps.fps.load()));
                    if (rc < 0) { 
                        printf("Failed to write to dev_status_csv_file\n");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,",
                        ((float)dev_info_data->dtof_sensor.configured_odr)/1000,
                        ((float)dev_info_data->dtof_sensor.tx_odr)/1000,
                        (dtof_rx_fps.fps.load()));
                    if (rc < 0) { 
                        printf("Failed to write to dev_status_csv_file\n");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,",
                        ((float)dev_info_data->imu_sensor.configured_odr)/1000,
                        ((float)dev_info_data->imu_sensor.tx_odr)/1000,
                        (imu_rx_fps.fps.load()));
                    if (rc < 0) { 
                        printf("Failed to write to dev_status_csv_file\n");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                        ((float)dev_info_data->slam_cloud_tx_odr)/1000,
                        (slam_cloud_rx_fps.fps.load()),
                        ((float)dev_info_data->slam_odom_tx_odr)/1000,
                        (slam_odom_rx_fps.fps.load()),
                        ((float)dev_info_data->slam_odom_highfreq_tx_odr)/1000,
                        (slam_odom_highfreq_rx_fps.fps.load()));
                    if (rc < 0) { 
                        printf("Failed to write to dev_status_csv_file\n");
                    }

                    std::fflush(dev_status_csv_file);
                }
            }
            if (g_show_fps) {
                printf("\n [dev_info] [soc_thermal]: package_temp:%dC \n", 
                    dev_info_data->soc_thermal.package_temp);
                printf("\n [dev_info] [soc_thermal]: cpu:%dC \n",
                    dev_info_data->soc_thermal.cpu_temp);                
                printf("\n [dev_info] [soc_thermal]: center_temp:%dC \n",
                    dev_info_data->soc_thermal.center_temp);
                printf("\n [dev_info] [soc_thermal]: gpu_temp:%dC \n",
                    dev_info_data->soc_thermal.gpu_temp);
                printf("\n [dev_info] [soc_thermal]: npu_temp:%dC \n",
                    dev_info_data->soc_thermal.npu_temp);

                for ( int i=0;i<8;i++) 
                {
                    printf("\n [dev_info] [cpu]: cpu_use_rate-core[%d]:%d%% \n",
                                                    i,
                                                    dev_info_data->cpu_use_rate[i]);

                }
                printf("\n [dev_info] [cpu]: ram_use_rate:%d%% \n",
                    dev_info_data->ram_use_rate);

                printf("\n [dev_info] [rgb]: configured_odr: %.2f HZ, tx_odr: %.2f HZ, rx_odr: %.2f HZ \n", 
                    ((float)dev_info_data->rgb_sensor.configured_odr)/1000,
                    ((float)dev_info_data->rgb_sensor.tx_odr)/1000,
                    (rgb_rx_fps.fps.load()));

                printf("\n [dev_info] [dtof]: configured_odr: %.2f HZ, tx_odr: %.2f HZ, rx_odr: %.2f HZ \n", 
                    ((float)dev_info_data->dtof_sensor.configured_odr)/1000,
                    ((float)dev_info_data->dtof_sensor.tx_odr)/1000,
                    (dtof_rx_fps.fps.load()));
                printf("\n [dev_info] [dtof]: subframe_odr: %.2f \n", 
                    ((float)dev_info_data->dtof_sensor.subframe_odr)/1000);
                printf("\n [dev_info] [dtof]: txtemp:%dC, rxtemp:%dC \n", dev_info_data->dtof_sensor.tx_temp, dev_info_data->dtof_sensor.rx_temp);

                printf("\n [dev_info] [imu]: configured_odr: %.2f HZ, tx_odr: %.2f HZ, rx_odr: %.2f HZ\n",
                    ((float) dev_info_data->imu_sensor.configured_odr)/1000,
                    ((float) dev_info_data->imu_sensor.tx_odr)/1000,
                    (imu_rx_fps.fps.load())
                );

                printf("\n [dev_info] [slam]: slam_cloud_tx_odr: %.2f HZ, rx_odr: %.2f HZ \n",
                    ((float)dev_info_data->slam_cloud_tx_odr)/1000,
                    (slam_cloud_rx_fps.fps.load())
                );

                printf("\n [dev_info] [slam]: slam_odom_tx_odr: %.2f HZ, rx_odr: %.2f HZ \n",
                    ((float)dev_info_data->slam_odom_tx_odr)/1000,
                    (slam_odom_rx_fps.fps.load())
                );

                printf("\n [dev_info] [slam]: slam_odom_highfreq_tx_odr: %.2f HZ, rx_odr: %.2f HZ \n",
                    ((float)dev_info_data->slam_odom_highfreq_tx_odr)/1000,
                    (slam_odom_highfreq_rx_fps.fps.load())
                );

                printf("\n------------------------------------------\n");
            }
            break;
            case LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ:
            {
                if (g_sendodom) {
                    g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream, true);
                }
                sensor_fps(&slam_odom_highfreq_rx_fps, "slam_odom_highfreq_rx");
            }
            break;
        default:
            printf("Unknown lidar data type: %x", data->type);
            return;
    }
}

static void lidar_device_callback(const lidar_device_info_t* device, bool attach)
{
    int type = LIDAR_MODE_SLAM;
    // int type = LIDAR_MODE_RAW;
    static std::chrono::steady_clock::time_point software_connect_start; 
    static bool software_connect_timing = false; 
    
    if(attach == true) {
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Hardware connected, starting software connection...");
        #else
            ROS_INFO("Hardware connected, starting software connection...");
        #endif
        if (!isUsb3OrHigher(TARGET_VENDOR, TARGET_PRODUCT)) {
            #ifdef ROS2
                RCLCPP_FATAL(rclcpp::get_logger("device_cb"), 
                            "Device connected to USB 2.0 port. This device requires USB 3.0 or higher. Exiting program.");
            #else
                ROS_FATAL("Device connected to USB 2.0 port. This device requires USB 3.0 or higher. Exiting program.");
            #endif

            g_usb_version_error = true;
            system("pkill -f rviz");
            exit(1);
            return;
        }

        software_connect_start = std::chrono::steady_clock::now();
        software_connect_timing = true;
        
        if (odinDevice) {
            odinDevice = nullptr;
        }
        
        if (lidar_create_device(const_cast<lidar_device_info_t*>(device), &odinDevice)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Create device failed");
            #else
                ROS_ERROR("Create device failed");
            #endif
            return;
        }
        
        if (lidar_open_device(odinDevice)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Open device failed");
            #else
                ROS_ERROR("Open device failed");
            #endif
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
        
	const std::string package_name = "odin_ros_driver";
	std::string config_dir = "";
	#ifdef ROS2
	    char* ros_workspace = std::getenv("COLCON_PREFIX_PATH");
	    if (ros_workspace) {
		std::string workspace_path(ros_workspace);
		size_t pos = workspace_path.find("/install");
		if (pos != std::string::npos) {
		    config_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/config";
		} else {
		    config_dir = ament_index_cpp::get_package_share_directory(package_name) + "/config";
		}
	    } else {
		config_dir = ament_index_cpp::get_package_share_directory(package_name) + "/config";
	    }
	#else
	    config_dir = ros::package::getPath(package_name) + "/config";
	#endif
   		 std::cout << "config_dir"<< config_dir <<std::endl;
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Calibration files will be saved to: %s", config_dir.c_str());
        #else
            ROS_INFO("Calibration files will be saved to: %s", config_dir.c_str());
        #endif
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - software_connect_start);
        if (elapsed.count() >= 60) {
            #ifdef ROS2
                RCLCPP_FATAL(rclcpp::get_logger("device_cb"), 
                            "Software connection timed out after 60 seconds. Exiting program.");
            #else
                ROS_FATAL("Software connection timed out after 60 seconds. Exiting program.");
            #endif
            
            if (odinDevice) {
                lidar_close_device(odinDevice);
                lidar_destory_device(odinDevice);
                odinDevice = nullptr;
            }
            
            g_connection_timeout = true;
            return;
        }
        
        if(lidar_get_version(odinDevice)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to get device firmware version, potential incompatible, please upgrade device firmware and retry.");
            #else
                ROS_ERROR("Failed to get device firmware version, potential incompatible, please upgrade device firmware and retry.");
            #endif
            system("pkill -f rviz");
            exit(1);
        }
        else {
            printf("ros_driver_version:%s\n", ros_driver_version);
            printf("get version success.\n");
        }
        
        if (lidar_get_calib_file(odinDevice, config_dir.c_str())) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to get calibration file");
            #else
                ROS_ERROR("Failed to get calibration file");
            #endif
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
        
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Successfully retrieved calibration files");
        #else
            ROS_INFO("Successfully retrieved calibration files");
        #endif
        
        std::string calib_config = config_dir + "/calib.yaml";
        calib_file_ = calib_config;
        if (std::filesystem::exists(calib_config)) {
            g_renderer = std::make_shared<rawCloudRender>();
            if (g_renderer->init(calib_config)) {
            #ifdef ROS2
                    RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Point cloud renderer initialized");
            #else
                    ROS_INFO("Point cloud renderer initialized");
            #endif
            } else {
            #ifdef ROS2
                    RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to initialize point cloud renderer");
            #else
                    ROS_ERROR("Failed to initialize point cloud renderer");
            #endif
                }
        } else {
            #ifdef ROS2
                    RCLCPP_WARN(rclcpp::get_logger("device_cb"), "Renderer config file not found: %s", calib_config.c_str());
            #else
                    ROS_WARN("Renderer config file not found: %s", calib_config.c_str());
            #endif
        }
        
        if (lidar_set_mode(odinDevice, type)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Set mode failed");
            #else
                ROS_ERROR("Set mode failed");
            #endif
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
 
        lidar_data_callback_info_t data_callback_info;
        data_callback_info.data_callback = lidar_data_callback;
        data_callback_info.user_data = &odinDevice;

        if (lidar_register_stream_callback(odinDevice, data_callback_info)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device"), "Register callback failed");
            #else
                ROS_ERROR("Register callback failed");
            #endif
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }

        uint32_t dtof_subframe_odr = 0;
        if (lidar_start_stream(odinDevice, type, dtof_subframe_odr)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Start stream failed");
            #else
                ROS_ERROR("Start stream failed");
            #endif
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
        
        if (dtof_subframe_odr > 0) {
            g_rosNodeControlImpl.setDtofSubframeODR(dtof_subframe_odr);
        }
        
        if (g_sendrgb) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_RGB);
        }
        if (g_sendimu) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_IMU);
        }
        if (g_sendodom) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_SLAM_ODOMETRY);
        }
        if (g_senddtof) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_DTOF);
        }
        if (g_sendcloudslam) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_SLAM_CLOUD);
        }
        
        software_connect_timing = false;
        deviceConnected = true;
        deviceDisconnected = false; 
        
        if (g_sendrgb_undistort && g_ros_object->loadCameraParams(calib_config) == 0) {
            g_ros_object->buildUndistortMap();
        }

        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Software connection successful in %ld seconds", 
                       std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - software_connect_start).count());
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device ready and streams activated");
        #else
            ROS_INFO("Software connection successful in %ld seconds", 
                    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - software_connect_start).count());
            ROS_INFO("Device ready and streams activated");
        #endif
    } else {
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device detaching...");
        #else
            ROS_INFO("Device detaching...");
        #endif

        deviceConnected = false;
        deviceDisconnected = true;

        clear_all_queues();

        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Waiting for device reconnection...");
        #else
            ROS_INFO("Waiting for device reconnection...");
        #endif
    }
}

int main(int argc, char *argv[])
{
#ifdef ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lydros_node");
    g_ros_object = std::make_shared<MultiSensorPublisher>(node);
#else
    ros::init(argc, argv, "lydros_node");
    ros::NodeHandle nh;
    g_ros_object = new MultiSensorPublisher(nh);
#endif

    try {
    #ifdef ROS2
        std::string package_path = get_package_source_directory();
        std::cout << "package_path: " << package_path << std::endl;
    #else
    	std::string package_path = get_package_share_path("odin_ros_driver");
    #endif
        std::string config_file = package_path + "/config/control_command.yaml";   
      
        odin_ros_driver::YamlParser parser(config_file);
        if (!parser.loadConfig()) {
            #ifdef ROS2
                RCLCPP_ERROR(node->get_logger(), "Failed to load config file: %s", config_file.c_str());
            #else
                ROS_ERROR("Failed to load config file: %s", config_file.c_str());
            #endif
            return -1;
        }

        auto keys = parser.getRegisterKeys();
        parser.printConfig();

        auto get_key_value = [&](const std::string& key, int default_value) -> int {
            auto it = keys.find(key);
            return it != keys.end() ? it->second : default_value;
        };

        g_sendrgb       = get_key_value("sendrgb", 1);
        g_sendimu       = get_key_value("sendimu", 1);
        g_senddtof      = get_key_value("senddtof", 1);
        g_sendodom      = get_key_value("sendodom", 1);
        g_sendcloudslam = get_key_value("sendcloudslam", 0);
        g_sendcloudrender = get_key_value("sendcloudrender", 1);
        g_sendrgb_compressed = get_key_value("sendrgbcompressed", 1);
        g_sendrgb_undistort = get_key_value("sendrgbundistort", 0);
        g_record_data = get_key_value("recorddata", 0);
        g_show_fps = get_key_value("showfps", 0);
        g_devstatus_log = get_key_value("devstatuslog", 0);
        g_log_level = get_key_value("log_devel", LOG_LEVEL_INFO);
        lidar_log_set_level(LIDAR_LOG_INFO);

        const std::string package_name = "odin_ros_driver";
        std::string data_dir = "";
        std::string log_dir = "";
        #ifdef ROS2
            char* ros_workspace = std::getenv("COLCON_PREFIX_PATH");
            if (ros_workspace) {
            std::string workspace_path(ros_workspace);
            size_t pos = workspace_path.find("/install");
            if (pos != std::string::npos) {
                data_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/recorddata";
                log_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/log";
            } else {
                data_dir = ament_index_cpp::get_package_share_directory(package_name) + "/recorddata";
                log_dir = ament_index_cpp::get_package_share_directory(package_name) + "/log";
            }
            } else {
            data_dir = ament_index_cpp::get_package_share_directory(package_name) + "/recorddata";
            log_dir = ament_index_cpp::get_package_share_directory(package_name) + "/log";
            }
        #else
            data_dir = ros::package::getPath(package_name) + "/recorddata";
            log_dir = ros::package::getPath(package_name) + "/log";
        #endif

        if (g_record_data) {
            g_ros_object->initialize_data_logger(data_dir);
        }

        if (g_devstatus_log) {
            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm{};
            #ifdef _WIN32
                localtime_s(&tm, &t);
            #else
                localtime_r(&t, &tm);
            #endif
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);

            std::filesystem::path log_root_dir_ = std::filesystem::path(log_dir) / buf;
            std::filesystem::create_directories(log_root_dir_);
            std::string dev_status_csv_file_path_ = log_root_dir_ / "dev_status.csv";

            // Open the file in append mode
            dev_status_csv_file = fopen(dev_status_csv_file_path_.c_str(), "a");
            if (!dev_status_csv_file) {
                #ifdef ROS2
                    RCLCPP_ERROR(rclcpp::get_logger("init"), "Failed to open dev_status CSV file");
                #else
                    ROS_ERROR("Failed to open dev_status CSV file");
                #endif
            } else {
                const char* header =
                "uptime_seconds,package_temp,cpu_temp,center_temp,gpu_temp,npu_temp,dtof_tx_temp,dtof_rx_temp,"
                "cpu0,cpu1,cpu2,cpu3,cpu4,cpu5,cpu6,cpu7,ram_use,"
                "rgb_configured_odr,rgb_tx_odr,rgb_rx_odr,dtof_configured_odr,dtof_tx_odr,dtof_rx_odr,imu_configured_odr,imu_tx_odr,imu_rx_odr,"
                "slam_cloud_tx_odr,slam_cloud_rx_odr,slam_odom_tx_odr,slam_odom_rx_odr,slam_odom_highfreq_tx_odr,slam_odom_highfreq_rx_odr\n";
                fprintf(dev_status_csv_file, "%s", header);
                std::fflush(dev_status_csv_file);
            }
        }

        if (lidar_system_init(lidar_device_callback)) {
            #ifdef ROS2
                RCLCPP_ERROR(node->get_logger(), "Lidar system init failed");
            #else
                ROS_ERROR("Lidar system init failed");
            #endif
            return -1;
        }
        

        bool usbPresent = false;
        bool usbVersionChecked = false; 
        while (!deviceConnected) {
            #ifdef ROS2
            if (!rclcpp::ok()) {
                break;
            }
            #else
            if (!ros::ok())     // ROS1 shutdown check
            {
                break;
            }
            #endif

            usbPresent = isUsbDevicePresent(TARGET_VENDOR, TARGET_PRODUCT); 
            if (usbPresent) { 
                if (!usbVersionChecked) {
                    usbVersionChecked = true;
                    
                    if (!isUsb3OrHigher(TARGET_VENDOR, TARGET_PRODUCT)) {
                        #ifdef ROS2
                            RCLCPP_FATAL(node->get_logger(), 
                                        "Device connected to USB 2.0 port. This device requires USB 3.0 or higher. Exiting program.Please use USB 3.0 and restart the device.");
                        #else
                            ROS_FATAL("Device connected to USB 2.0 port. This device requires USB 3.0 or higher. Exiting program .Please use USB 3.0 and restart the device.");
                        #endif
                        
                        lidar_system_deinit();
                        return 1;
                    }
                }
            }
            
            #ifdef ROS2
                std::this_thread::sleep_for(std::chrono::seconds(1));
            #else
                ros::Duration(1.0).sleep();
            #endif
        }
    } catch (const std::exception& e) {
        #ifdef ROS2
                RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        #else
                ROS_ERROR("Exception: %s", e.what());
        #endif
                lidar_system_deinit();
                return -1;
    }

    if (!deviceConnected) {
        #ifdef ROS2
        if (g_ros_object) {
            g_ros_object.reset();   // destroys all publishers/subscribers
        }
        node.reset();              // destroy the node first
        rclcpp::shutdown();
        #else
        if (g_ros_object) {
            delete g_ros_object;
            g_ros_object = nullptr;
        }
        ros::shutdown();
        #endif
        return 1;
    }

    bool disconnect_msg_printed = false;
    #ifdef ROS2
        // Create 10Hz Rate object
        rclcpp::Rate rate(10);
        
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            // Check device disconnection status
            if (deviceDisconnected.load()) {
                if (!disconnect_msg_printed) {
                    RCLCPP_INFO(node->get_logger(), "Device disconnected, waiting for reconnection...");
                    disconnect_msg_printed = true;
                }
                
                // Wait 0.1 seconds
                rate.sleep();
                continue;  // Skip rest of this loop iteration
            }
            
            // Data processing when device is connected
            if (g_sendcloudrender) {
                g_ros_object->try_process_pair();  
            }
            disconnect_msg_printed = false;

            // Wait 0.1 seconds
            rate.sleep();
        }
        rclcpp::shutdown();
    #else
        // Create 10Hz Rate object
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();

            // Check device disconnection status
            if (deviceDisconnected.load()) {
                if (!disconnect_msg_printed) {
                    ROS_INFO("Device disconnected, waiting for reconnection...");
                    disconnect_msg_printed = true;
                }
                
                // Wait 0.1 seconds
                rate.sleep();
                continue;  // Skip rest of this loop iteration
            }
            
            // Data processing when device is connected
            if (g_sendcloudrender) {
                g_ros_object->try_process_pair();  
            }
            disconnect_msg_printed = false;

            // Wait 0.1 seconds
            rate.sleep();
        }
        ros::shutdown();
    #endif

    // Cleanup on normal program exit
    if (odinDevice) {
        // Convert calib.yaml to cam_in_ex.txt at program end
        if (g_ros_object) {
            const std::filesystem::path out_path = g_ros_object->get_root_dir() / "image" / "cam_in_ex.txt";
            (void)convert_calib_to_cam_in_ex(calib_file_, out_path);
        }
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "pose_index: %d", g_ros_object->get_pose_index());
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "cloud_index: %d", g_ros_object->get_cloud_index());
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "image_index: %d", g_ros_object->get_image_index());
        #else
            ROS_INFO("pose_index: %d", g_ros_object->get_pose_index());
            ROS_INFO("cloud_index: %d", g_ros_object->get_cloud_index());
            ROS_INFO("image_index: %d", g_ros_object->get_image_index());
        #endif
        // Perform cleanup on normal exit
        // if(lidar_stop_stream(odinDevice, LIDAR_MODE_SLAM))
        // {
        //     #ifdef ROS2
        //         RCLCPP_INFO(rclcpp::get_logger("device_cb"), "lidar_stop_stream failed");
        //     #else
        //         ROS_INFO("lidar_stop_stream failed");
        //     #endif
        // }
        
        if(lidar_unregister_stream_callback(odinDevice))
        {
            #ifdef ROS2
                RCLCPP_INFO(rclcpp::get_logger("device_cb"), "lidar_unregister_stream_callback failed");
            #else
                ROS_INFO("lidar_unregister_stream_callback failed");
            #endif
        }
        // lidar_close_device(odinDevice);
        // lidar_destory_device(odinDevice);

        std::fflush(dev_status_csv_file);
        fclose(dev_status_csv_file);
    }
    // lidar_system_deinit();



    return 0;
}
