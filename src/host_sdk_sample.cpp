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
#include <fstream>
#include <vector>
#include <cstdio>
#include <array>
#ifdef ROS2
    #include <ament_index_cpp/get_package_share_directory.hpp>
    #include <rclcpp/rclcpp.hpp>
#else
    #include <ros/package.h>
    #include <ros/ros.h> 
#endif
#define ros_driver_version "0.3.0"
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

// Get package path
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
    
    switch(data->type) {
        case LIDAR_DT_NONE:
            printf("empty lidar data type: %x\n", data->type);
            break;
        case LIDAR_DT_RAW_RGB:
            if (g_sendrgb) {
                g_ros_object->publishRgb((capture_Image_List_t *)&data->stream);
            }
            break;
        case LIDAR_DT_RAW_IMU:
            if (g_sendimu) {
                g_ros_object->publishImu((icm_6aixs_data_t *)data->stream.imageList[0].pAddr);
            }
            break;
        case LIDAR_DT_RAW_DTOF:
        if (g_senddtof ) {
            g_ros_object->publishIntensityCloud((capture_Image_List_t *)&data->stream, 1);
            }
          break;
        case LIDAR_DT_SLAM_CLOUD:
            if (g_sendcloudslam) {
                g_ros_object->publishPC2XYZRGBA((capture_Image_List_t *)&data->stream, 0);
            }
            break;
        case LIDAR_DT_SLAM_ODOMETRY:
            if (g_sendodom) {
                g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream);
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
            printf("get version failed.\n");
        }
        else {
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
        
        if (lidar_start_stream(odinDevice, type)) {
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
        std::string package_path = get_package_share_path("odin_ros_driver");
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
        g_log_level = get_key_value("log_devel", LOG_LEVEL_INFO);
        lidar_log_set_level(LIDAR_LOG_INFO);

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

    #ifdef ROS2
        // Create 10Hz Rate object
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);

            // Check device disconnection status
            if (deviceDisconnected.load()) {
                #ifdef ROS2
                    RCLCPP_INFO(node->get_logger(), "Device disconnected, waiting for reconnection...");
                #else
                    ROS_INFO("Device disconnected, waiting for reconnection...");
                #endif
                
                // Wait 0.1 seconds
                rate.sleep();
                continue;  // Skip rest of this loop iteration
            }
            
            // Data processing when device is connected
            if (g_sendcloudrender) {
                g_ros_object->try_process_pair();  
            }

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
                ROS_INFO("Device disconnected, waiting for reconnection...");
                
                // Wait 0.1 seconds
                rate.sleep();
                continue;  // Skip rest of this loop iteration
            }
            
            // Data processing when device is connected
            if (g_sendcloudrender) {
                g_ros_object->try_process_pair();  
            }

            // Wait 0.1 seconds
            rate.sleep();
        }
        ros::shutdown();
    #endif

    // Cleanup on normal program exit
    if (odinDevice) {
        // Perform cleanup on normal exit
        lidar_stop_stream(odinDevice, LIDAR_MODE_SLAM);
        lidar_unregister_stream_callback(odinDevice);
        lidar_close_device(odinDevice);
        lidar_destory_device(odinDevice);
    }
    lidar_system_deinit();

    return 0;
}
