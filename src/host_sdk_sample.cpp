#include "host_sdk_sample.h"
#include "yaml_parser.h"
#include <filesystem> 
#include <thread>
#include <string>
#include <stdexcept>
#include <atomic>

#ifdef ROS2
    #include <ament_index_cpp/get_package_share_directory.hpp>
#else
    #include <ros/package.h>
#endif

static device_handle odinDevice = nullptr;
static std::shared_ptr<MultiSensorPublisher> g_ros_object;
static std::atomic<bool> deviceConnected(false);

// Function to get package share path
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

// Global configuration variables
static int g_sendrgb = 1;
static int g_sendimu = 1;
static int g_senddtof = 1;
static int g_sendodom = 1;
static int g_sendcloudslam = 0;

static void lidar_data_callback(const lidar_data_t *data, void *user_data)
{
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
            if (g_senddtof) {
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
    if(attach == true) {
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device attaching...");
        #else
            ROS_INFO("Device attaching...");
        #endif
        
        // Clean up any existing device
        if (odinDevice) {
            lidar_stop_stream(odinDevice, type);
            lidar_unregister_stream_callback(odinDevice);
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
        }
        
        // Use const_cast to remove const qualifier
        if (lidar_create_device(const_cast<lidar_device_info_t*>(device), &odinDevice)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Create device failed");
            #else
                ROS_ERROR("Create device failed");
            #endif
            return;
        }
        
        // Open device
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
        
        // Set mode
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
        
        // Register callback
        lidar_data_callback_info_t data_callback_info;
        data_callback_info.data_callback = lidar_data_callback;
        data_callback_info.user_data = &odinDevice;

        if (lidar_register_stream_callback(odinDevice, data_callback_info)) {
            #ifdef ROS2
                RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Register callback failed");
            #else
                ROS_ERROR("Register callback failed");
            #endif
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
        
        // Start data stream
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
        
        // Activate stream types based on configuration
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
        
        deviceConnected = true;
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device ready and streams activated");
        #else
            ROS_INFO("Device ready and streams activated");
        #endif
    } else {
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device detaching...");
        #else
            ROS_INFO("Device detaching...");
        #endif
        
        deviceConnected = false;
        
        if (odinDevice) {
            lidar_stop_stream(odinDevice, LIDAR_MODE_SLAM);
            lidar_unregister_stream_callback(odinDevice);
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
        }
    }
}

int main(int argc, char *argv[])
{
    // ROS initialization
    #ifdef ROS2
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("lydros_node");
        g_ros_object = std::make_shared<MultiSensorPublisher>(node);
    #else
        ros::init(argc, argv, "lydros_node");
        ros::NodeHandle nh;
        g_ros_object = std::make_shared<MultiSensorPublisher>(nh);
    #endif

    try {
        std::string package_path = get_package_share_path("odin_ros_driver");
        std::string config_file = package_path + "/config/control_command.yaml";   
        
        // Create YAML parser
        odin_ros_driver::YamlParser parser(config_file);
        
        // Load configuration
        if (!parser.loadConfig()) {
        #ifdef ROS2
            RCLCPP_ERROR(node->get_logger(), "Failed to load config file: %s", config_file.c_str());
        #else
            ROS_ERROR("Failed to load config file: %s", config_file.c_str());
        #endif
            return -1;
        }
        
        // Get key-value
        auto keys = parser.getRegisterKeys();
        
        // Print configuration
        parser.printConfig();
        
        auto get_key_value = [&](const std::string& key_name, int default_value) -> int {  
            auto it = keys.find(key_name);
            if (it != keys.end()) {
                return it->second;
            }
            return default_value;
        };
        
        // Read configuration values into global variables
        g_sendrgb = get_key_value("sendrgb", 1);
        g_sendimu = get_key_value("sendimu", 1);
        g_senddtof = get_key_value("senddtof", 1);
        g_sendodom = get_key_value("sendodom", 1);
        g_sendcloudslam = get_key_value("sendcloudslam", 0);
        
        // Set log level
        lidar_log_set_level(LIDAR_LOG_INFO);

        // Initialize system and start USB monitoring
        if(lidar_system_init(lidar_device_callback)) {
        #ifdef ROS2
            RCLCPP_ERROR(node->get_logger(), "Lidar system init failed");
        #else
            ROS_ERROR("Lidar system init failed");
        #endif
            return -1;
        }
        
        // wait device connect
        #ifdef ROS2
            RCLCPP_INFO(node->get_logger(), "Waiting for device connection...");
        #else
            ROS_INFO("Waiting for device connection...");
        #endif
        
        auto start = std::chrono::steady_clock::now();
        while (!deviceConnected) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);
            
            if (elapsed.count() >= 30) {
            #ifdef ROS2
                RCLCPP_ERROR(node->get_logger(), "No device connected after 30 seconds");
            #else
                ROS_ERROR("No device connected after 30 seconds");
            #endif
                lidar_system_deinit();
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    // ROS loop
    #ifdef ROS2
        rclcpp::spin(node);
        rclcpp::shutdown();
    #else
        ros::spin();
        ros::shutdown();
    #endif

    // Cleanup
    if (odinDevice) {
        lidar_stop_stream(odinDevice, LIDAR_MODE_SLAM);
        lidar_unregister_stream_callback(odinDevice);
        lidar_close_device(odinDevice);
        lidar_destory_device(odinDevice);
    }
    lidar_system_deinit();
    
    return 0;
}
