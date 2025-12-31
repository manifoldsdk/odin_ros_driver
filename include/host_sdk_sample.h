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
#pragma once

#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <Eigen/Dense>
#include <atomic>
#include <unordered_map>
#include "data_logger.h"
#include "lidar_api.h"
#include "lidar_api_type.h"
#include "rawCloudRender.h"
#include <deque> 
#include <mutex>  
#include <vector> 
#include <queue>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include "polynomial_camera.hpp"
#include "camera_pose_visualization.h"

struct CameraParams {
    int width, height;
    double fx, fy, cx, cy, skew;
    double k2, k3, k4, k5, k6, k7;
    double p1, p2;
};

enum class OdometryType {
    STANDARD = LIDAR_DT_SLAM_ODOMETRY,
    HIGHFREQ = LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ,
    TRANSFORM = LIDAR_DT_SLAM_ODOMETRY_TF
};

#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DEBUG 4


extern int g_log_level;
extern int g_sendcloudrender;
#ifdef ROS2

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    #include <std_msgs/msg/header.hpp>
    #include <visualization_msgs/msg/marker_array.hpp>
    #include "sensor_msgs/msg/image.hpp"
    #include "sensor_msgs/msg/imu.hpp"
    #include "sensor_msgs/msg/point_cloud2.hpp"
    #include "sensor_msgs/point_cloud2_iterator.hpp"
    #include <sensor_msgs/msg/compressed_image.hpp>
    #include <builtin_interfaces/msg/time.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <nav_msgs/msg/path.hpp>
    #include <sensor_msgs/msg/point_field.hpp>
    #include "tf2/LinearMath/Quaternion.h"
    #include "tf2_ros/transform_broadcaster.h"
    namespace ros {
        using namespace rclcpp;
        using namespace std_msgs::msg;
        using namespace sensor_msgs::msg;
        using namespace nav_msgs::msg;
        using namespace visualization_msgs::msg;
        using Time = builtin_interfaces::msg::Time;
    }
    

    #define LOG_ERROR(...)
    #define LOG_WARN(...)
    #define LOG_INFO(...)
    #define LOG_DEBUG(...)
#else

    #include <ros/ros.h>
    #include <ros/package.h>
    #include <sensor_msgs/Image.h>
    #include <std_msgs/Header.h>
    #include <sensor_msgs/Imu.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/point_cloud2_iterator.h>
    #include <sensor_msgs/CompressedImage.h>
    #include <nav_msgs/Odometry.h>
    #include <nav_msgs/Path.h>
    #include <sensor_msgs/Image.h>
    #include <tf2_ros/transform_broadcaster.h>
    #include <tf2/LinearMath/Quaternion.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    namespace ros {
        using namespace ::ros;
        using namespace sensor_msgs;
        using namespace nav_msgs;
    }
    

    #define LOG_ERROR(...) \
        if (g_log_level >= LOG_LEVEL_ERROR) { \
            ROS_ERROR(__VA_ARGS__); \
        }
    #define LOG_WARN(...) \
        if (g_log_level >= LOG_LEVEL_WARN) { \
            ROS_WARN(__VA_ARGS__); \
        }
    #define LOG_INFO(...) \
        if (g_log_level >= LOG_LEVEL_INFO) { \
            ROS_INFO(__VA_ARGS__); \
        }
    #define LOG_DEBUG(...) \
        if (g_log_level >= LOG_LEVEL_DEBUG) { \
            ROS_DEBUG(__VA_ARGS__); \
        }
#endif


#ifdef ROS2
    namespace sensor_msgs {
        using PointField = msg::PointField;
    }
#else
    namespace sensor_msgs {
        using PointField = ::sensor_msgs::PointField;
    }
#endif

// Common definitions
#define PAI 3.14159265358979323846
#define DTOF_NUM_ROW_PER_GROUP 6
// Common functions
inline ros::Time ns_to_ros_time(uint64_t timestamp_ns) {
    ros::Time t;
    #ifdef ROS2
        t.sec = static_cast<int32_t>(timestamp_ns / 1000000000);
        t.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000);
    #else
        t.sec = static_cast<uint32_t>(timestamp_ns / 1000000000);
        t.nsec = static_cast<uint32_t>(timestamp_ns % 1000000000);
    #endif
    return t;
}

inline uint64_t ros_time_to_ns(const ros::Time &t) {
    #ifdef ROS2
        return static_cast<uint64_t>(t.sec) * 1000000000ULL + t.nanosec;
    #else
        return static_cast<uint64_t>(t.sec) * 1000000000ULL + t.nsec;
    #endif
}

class RosNodeControlInterface {
    public:
        virtual ~RosNodeControlInterface() = default;
        virtual void setDtofSubframeODR(int odr) = 0;
        virtual int getDtofSubframeODR() const = 0;
        virtual void setUseHostRosTime(bool use_host_ros_time) = 0;
        virtual bool useHostRosTime() const = 0;
        virtual void setSendOdomBaseLinkTF(bool send_odom_baselink_tf) = 0;
        virtual bool sendOdomBaseLinkTF() const = 0;
    };
    
RosNodeControlInterface* getRosNodeControl();

// Multi-sensor publisher class
class MultiSensorPublisher {
public:
    #ifdef ROS2
        MultiSensorPublisher(rclcpp::Node::SharedPtr node)
            : node_(node),cameraposevisual_ {1.0f, 0.0f, 0.0f, 1.0f} {
            initialize_publishers();
            // initialize_data_logger();
        }
    #else
        MultiSensorPublisher(ros::NodeHandle& nh)
            : cameraposevisual_(1.0f, 0.0f, 0.0f, 1.0f) {
            initialize_publishers(nh);
            // initialize_data_logger();
        }
    #endif
    
    std::filesystem::path get_root_dir() const { return root_dir_; }

    void set_log_level(int level) {
        g_log_level = level;
    }
    // Optional external logger setter
    void set_data_logger(std::shared_ptr<BinaryDataLogger> logger) {
        data_logger_ = std::move(logger);
    }

    int get_pose_index() {
        return pose_index_.load();
    }

    int get_cloud_index() {
        return cloud_index_.load();
    }

    int get_image_index() {
        return image_index_.load();
    }
 
    rawCloudRender render_;
    void publishImu(imu_convert_data_t *stream) {
        #ifdef ROS2
            sensor_msgs::msg::Imu imu_msg;
        #else
            ros::Imu imu_msg;
        #endif
        
        if (getRosNodeControl()->useHostRosTime()) {
            #ifdef ROS2
                imu_msg.header.stamp = node_->now();
            #else
                imu_msg.header.stamp = ros::Time::now();
            #endif
        } else {
            imu_msg.header.stamp = ns_to_ros_time(stream->stamp);
        }
        imu_msg.header.frame_id = "imu_link";

        imu_msg.linear_acceleration.y = -1 * stream->accel_x;
        imu_msg.linear_acceleration.x = stream->accel_y;
        imu_msg.linear_acceleration.z = stream->accel_z;

        imu_msg.angular_velocity.y = -1 * stream->gyro_x;
        imu_msg.angular_velocity.x = stream->gyro_y;
        imu_msg.angular_velocity.z = stream->gyro_z;

        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;
        
        #ifdef ROS2
            imu_pub_->publish(std::move(imu_msg));
        #else
            imu_pub_.publish(imu_msg);
        #endif
    }
#ifdef ROS2
    using ImageMsg = sensor_msgs::msg::Image;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using ImageConstPtr = ImageMsg::ConstSharedPtr;
    using PointCloud2ConstPtr = PointCloud2Msg::ConstSharedPtr;
#else
    using ImageMsg = sensor_msgs::Image;
    using PointCloud2Msg = sensor_msgs::PointCloud2;
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using PointCloud2ConstPtr = sensor_msgs::PointCloud2ConstPtr;
#endif
void try_process_pair() {
    // Record queue status
    size_t rgb_size, pcd_size;
    {
        std::lock_guard<std::mutex> lock1(rgb_queue_mutex_);
        std::lock_guard<std::mutex> lock2(pcd_queue_mutex_);
        rgb_size = rgb_image_queue_.size();
        pcd_size = pcd_queue_.size();
    }
    
    while (true) {
        ImageConstPtr rgb_msg = nullptr;
        PointCloud2ConstPtr pcd_msg = nullptr;
        
        // Get a pair of data from queues (with lock protection)
        {
            std::lock_guard<std::mutex> lock1(rgb_queue_mutex_);
            std::lock_guard<std::mutex> lock2(pcd_queue_mutex_);
            
            if (!rgb_image_queue_.empty() && !pcd_queue_.empty()) {
                rgb_msg = rgb_image_queue_.front();
                pcd_msg = pcd_queue_.front();
            }
        }
        
        if (!rgb_msg || !pcd_msg) {
            break;
        }
        
        // timestamp
        uint64_t rgb_stamp = ros_time_to_ns(rgb_msg->header.stamp);
        uint64_t pcd_stamp = ros_time_to_ns(pcd_msg->header.stamp);
        int64_t time_diff = static_cast<int64_t>(rgb_stamp) - static_cast<int64_t>(pcd_stamp);
        int64_t abs_time_diff = std::abs(time_diff);
        
        // Check if time difference is within allowed range (50ms)
        const int64_t MAX_TIME_DIFF = 50000000; // 50ms in nanoseconds
        if (abs_time_diff > MAX_TIME_DIFF) {
            // Remove older timestamped message
            {
                std::lock_guard<std::mutex> lock1(rgb_queue_mutex_);
                std::lock_guard<std::mutex> lock2(pcd_queue_mutex_);
                
                if (time_diff > 0) {
                    // RGB timestamp is newer, remove PCD
                    pcd_queue_.pop_front();
                } else {
                    // PCD timestamp is newer, remove RGB
                    rgb_image_queue_.pop_front();
                }
            }
            
            // Try next pair
            continue;
        }
        
        // Time difference within allowed range, process data pair
        {
            std::lock_guard<std::mutex> lock1(rgb_queue_mutex_);
            std::lock_guard<std::mutex> lock2(pcd_queue_mutex_);
            
            // Remove messages from queues
            rgb_image_queue_.pop_front();
            pcd_queue_.pop_front();
        }
        
        // Process data pair
        process_pair(rgb_msg, pcd_msg);
    }
}
bool validate_render_parameters(std::vector<std::vector<float>>& rgb_image, 
                               capture_Image_List_t* cloud_stream, 
                               int pcd_idx) 
{
    // 1. Check RGB image validity
    if (rgb_image.empty()) {
        #ifndef ROS2
            ROS_ERROR("Invalid RGB image: empty vector");
        #endif
        return false;
    }
    
    // Check RGB image dimension consistency
    const size_t height = rgb_image.size();
    const size_t width = (height > 0) ? rgb_image[0].size() : 0;
    
    if (height == 0 || width == 0) {
        #ifndef ROS2
            ROS_ERROR("Invalid RGB image dimensions: %zux%zu", 
                     height, width);
        #endif
        return false;
    }
    
    // 2. Check point cloud stream pointer validity
    if (!cloud_stream) {
        #ifndef ROS2
            ROS_ERROR("Invalid cloud stream: null pointer");
        #endif
        return false;
    }
    
    // 3. Check point cloud index validity
    if (pcd_idx < 0 || pcd_idx >= 10) {
        #ifndef ROS2
            ROS_ERROR("Invalid pcd index: %d (must be 0-9)", pcd_idx);
        #endif
        return false;
    }
    
    // 4. Check point cloud data validity
    buffer_List_t& cloud = cloud_stream->imageList[pcd_idx];
    if (!cloud.pAddr) {
        #ifndef ROS2
            ROS_ERROR("Invalid cloud data: null pointer");
        #endif
        return false;
    }
    
    if (cloud.width <= 0 || cloud.height <= 0) {
        #ifndef ROS2
            ROS_ERROR("Invalid cloud dimensions: %dx%d", 
                     cloud.width, cloud.height);
        #endif
        return false;
    }
    
    return true;
}

void process_pair(const ImageConstPtr &rgb_msg, const PointCloud2ConstPtr &pcd_msg)
{
    auto start_time = std::chrono::steady_clock::now();

    const int input_image_width = rgb_msg->width;
    const int input_image_height = rgb_msg->height;

    // Verify input image format
    if (rgb_msg->encoding != "bgr8") {
        #ifndef ROS2
            ROS_ERROR("Unsupported image format: %s. Only bgr8 is supported.", 
                      rgb_msg->encoding.c_str());
        #endif
        return;
    }

    // Prepare point cloud data stream
    capture_Image_List_t cloud_stream;
    const int pcd_idx = 2;
    cloud_stream.imageList[pcd_idx].height = pcd_msg->height;
    cloud_stream.imageList[pcd_idx].width = pcd_msg->width;

    const auto total_point_num = pcd_msg->width * pcd_msg->height;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pcd_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pcd_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pcd_msg, "z");
    std::vector<float> cloud_flat;
    cloud_flat.reserve(total_point_num * 4);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        cloud_flat.push_back(*iter_y * -1000.0f);
        cloud_flat.push_back(*iter_z * 1000.0f);
        cloud_flat.push_back(*iter_x * 1000.0f);
        cloud_flat.push_back(0.0f);
    }
    cloud_stream.imageList[pcd_idx].pAddr = cloud_flat.data();

    
    if (input_image_height > 0 && input_image_width > 0) {
        // Use BGR8 image data directly
        std::vector<std::vector<float>> rgb_image(input_image_height, std::vector<float>(input_image_width));
        
        // Convert BGR8 data to required format for rendering
        const uint8_t* bgr_data = rgb_msg->data.data();
        for (int y = 0; y < input_image_height; ++y) {
            for (int x = 0; x < input_image_width; ++x) {
                // Start position of each pixel (BGR format)
                int idx = y * rgb_msg->step + x * 3;
                
                // Extract RGB values and combine into 32-bit integer
                uint8_t b = bgr_data[idx];
                uint8_t g = bgr_data[idx + 1];
                uint8_t r = bgr_data[idx + 2];
                uint32_t rgb_int = (r << 16) | (g << 8) | b;
                
                // Convert to float
                float rgb_float;
                std::memcpy(&rgb_float, &rgb_int, sizeof(float));
                rgb_image[y][x] = rgb_float;
            }
        }
        // Render colored point cloud
        std::vector<float> rgbCloud_flat;
        render_.render(rgb_image, &cloud_stream, pcd_idx, rgbCloud_flat);
        const int valid_point_num = rgbCloud_flat.size() / 4;
         
        // Create and publish RGB point cloud
        PointCloud2Msg output_msg;
        output_msg.header.frame_id = "odin1_base_link";
        output_msg.header.stamp = rgb_msg->header.stamp; // Use original image timestamp
        output_msg.height = 1;
        output_msg.width = valid_point_num;

        sensor_msgs::PointCloud2Modifier modifier(output_msg);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::PointField::FLOAT32
        );
        modifier.resize(valid_point_num);

        sensor_msgs::PointCloud2Iterator<float> iter_res_x(output_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_res_y(output_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_res_z(output_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_res_rgb(output_msg, "rgb");

        for (int i = 0; i < valid_point_num; i++) {
            *iter_res_x = rgbCloud_flat[4*i]; ++iter_res_x;
            *iter_res_y = rgbCloud_flat[4*i+1]; ++iter_res_y;
            *iter_res_z = rgbCloud_flat[4*i+2]; ++iter_res_z;
            *iter_res_rgb = rgbCloud_flat[4*i+3]; ++iter_res_rgb;
        }

        #ifdef ROS2
            rgbcloud_pub_->publish(output_msg);
        #else
            rgbcloud_pub_.publish(output_msg);
        #endif
    } 
}

void publishIntensityCloud(capture_Image_List_t* stream, int idx)
{
    // Check index validity
    if (idx < 0 || idx >= 10) {
        #ifndef ROS2
            ROS_ERROR("Invalid index %d for intensity cloud", idx);
        #endif
        return;
    }

    // Check point cloud data validity
    buffer_List_t &cloud = stream->imageList[idx];
    if (!cloud.pAddr) {
        #ifndef ROS2
            ROS_ERROR("Invalid point cloud: null data pointer at index %d", idx);
        #endif
        return;
    }
 
    if (cloud.width <= 0 || cloud.height <= 0) {
        #ifndef ROS2
            ROS_ERROR("Invalid point cloud dimensions: %dx%d at index %d", 
                     cloud.width, cloud.height, idx);
        #endif
        return;
    }

    #ifdef ROS2
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    #else
        auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
    #endif

    // Set message header
    msg->header.frame_id = "odin1_base_link";
    if (getRosNodeControl()->useHostRosTime()) {
        #ifdef ROS2
            msg->header.stamp = node_->now();
        #else
            msg->header.stamp = ros::Time::now();
        #endif
    } else {
        msg->header.stamp = ns_to_ros_time(cloud.timestamp);
    }

    msg->height = cloud.height;
    msg->width = cloud.width;
    msg->is_dense = false;
    msg->is_bigendian = false;

    // Set point cloud fields
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2Fields(
        6,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::UINT8,
        "confidence", 1, sensor_msgs::PointField::UINT16,
        "offset_time", 1, sensor_msgs::PointField::FLOAT32
    );
    modifier.resize(msg->height * msg->width);


    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_confidence(*msg, "confidence");
    sensor_msgs::PointCloud2Iterator<float> iter_offsettime(*msg, "offset_time");

    float* xyz_data_f = static_cast<float*>(cloud.pAddr);
    int total_points = cloud.height * cloud.width;
	//std::cout << stream->imageCount << std::endl;
    float dtof_subframe_odr = getRosNodeControl()->getDtofSubframeODR() / 1000.0f;
    // printf("dtof_subframe_odr: %f\n", dtof_subframe_odr);

    if (stream->imageCount == 4) {

        uint8_t* intensity_data = static_cast<uint8_t*>(stream->imageList[2].pAddr);
        uint16_t* confidence_data = static_cast<uint16_t*>(stream->imageList[3].pAddr);
    
        for (int i = 0; i < total_points; ++i) {
            if (confidence_data[i] < 35) {
                *iter_x = 0.0f; ++iter_x;
                *iter_y = 0.0f; ++iter_y;
                *iter_z = 0.0f; ++iter_z;
                *iter_intensity = 0; ++iter_intensity;
                *iter_confidence = 0; ++iter_confidence;
                *iter_offsettime = 0.0f; ++iter_offsettime;
            } else {
                // XYZ point
                *iter_x = xyz_data_f[i * 3 + 2] / 1000.0f; ++iter_x;
                *iter_y = -xyz_data_f[i * 3 + 0] / 1000.0f; ++iter_y;
                *iter_z = xyz_data_f[i * 3 + 1] / 1000.0f; ++iter_z;
                
                *iter_intensity = intensity_data[i]; ++iter_intensity;
                *iter_confidence = confidence_data[i]; ++iter_confidence;
                
                if (dtof_subframe_odr > 0.0) {
                    int line_num = i / 256;
                    int group = line_num / DTOF_NUM_ROW_PER_GROUP;
                    float timestamp_offset = group * 1.0 / dtof_subframe_odr;

                    *iter_offsettime = timestamp_offset;
                    ++iter_offsettime;
                }
            }
    	}
    } else {
        uint16_t* intensity_data = static_cast<uint16_t*>(stream->imageList[2].pAddr);
        
        for (int i = 0; i < total_points; ++i) {
            *iter_x = xyz_data_f[i * 4 + 2] / 1000.0f; ++iter_x;
            *iter_y = -xyz_data_f[i * 4 + 0] / 1000.0f; ++iter_y;
            *iter_z = xyz_data_f[i * 4 + 1] / 1000.0f; ++iter_z;
            
            float intensity = (intensity_data[i] - 10) * 255.0f / (12500 - 10);
            if (intensity > 255) {
                *iter_intensity = 255;
            } else if (intensity < 0) {
                *iter_intensity = 0;
            } else {
                *iter_intensity = static_cast<uint8_t>(intensity);
            }
            ++iter_intensity;
            
            *iter_confidence = 0;
            ++iter_confidence;
        }
    }

    {
        std::lock_guard<std::mutex> lock(pcd_queue_mutex_);
        
        // Get actual point count
        const int real_point_count = cloud.width * cloud.height;  
        // Create deep copy of point cloud
        #ifdef ROS2
            auto msg_copy = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
        #else
            auto msg_copy = boost::make_shared<sensor_msgs::PointCloud2>();
            *msg_copy = *msg;  // Deep copy
        #endif
        
        // Queue management
        if (pcd_queue_.size() >= 10) {
            pcd_queue_.pop_front();
        }
        
        // Add to queue (using copy)
        pcd_queue_.push_back(msg_copy);
    }

    // Publish point cloud
    #ifdef ROS2
        cloud_pub_->publish(*msg);
    #else
        cloud_pub_.publish(msg);
    #endif
}

void publishGrayUInt8(capture_Image_List_t *stream, int idx) {
    ImageMsg msg;
    if (getRosNodeControl()->useHostRosTime()) {
        #ifdef ROS2
            msg.header.stamp = node_->now();
        #else
            msg.header.stamp = ros::Time::now();
        #endif
    } else {
        msg.header.stamp = ns_to_ros_time(stream->imageList[idx].timestamp);
    }
    msg.header.frame_id = "map";

    int width = stream->imageList[idx].width;
    int height = stream->imageList[idx].height;

    msg.height = height;
    msg.width = width;
    msg.encoding = "mono8";
    msg.is_bigendian = false;
    msg.step = width * sizeof(uint8_t);

    size_t image_size = msg.step * height;

    msg.data.resize(image_size);

    memcpy(msg.data.data(), stream->imageList[idx].pAddr, image_size);

    #ifdef ROS2
        intensity_gray_pub_->publish(msg);
    #else
        intensity_gray_pub_.publish(msg);
    #endif
}

void publishRgb(capture_Image_List_t *stream) {
    buffer_List_t &image = stream->imageList[0];

    // old version yuv data
    if (image.length == image.width * image.height * 3 / 2) {
        #ifdef ROS2
            RCLCPP_INFO(rclcpp::get_logger("publishRgb"), "old format rgb data, please upgrade device firmware");
        #else
            ROS_INFO("old format rgb data, please upgrade device firmware");
        #endif
    } else {// new version jpeg data

        std::vector<uint8_t> jpeg_data(static_cast<uint8_t*>(image.pAddr),
                                        static_cast<uint8_t*>(image.pAddr) + image.length);

        // convert back to bgr8                                                
        cv::Mat decoded_image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        cv_bridge::CvImage cv_image;
        if (getRosNodeControl()->useHostRosTime()) {
            #ifdef ROS2
                cv_image.header.stamp = node_->now();
            #else
                cv_image.header.stamp = ros::Time::now();
            #endif
        } else {
            cv_image.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
        }
        cv_image.encoding = "bgr8";
        cv_image.image = decoded_image;

        if (g_sendcloudrender) {
            std::lock_guard<std::mutex> lock(rgb_queue_mutex_);
            if (rgb_image_queue_.size() >= 10) {
                rgb_image_queue_.pop_front();
            }
            rgb_image_queue_.push_back(cv_image.toImageMsg());
            }        

        // Enqueue binary logging for image
        if (data_logger_) {
            const uint32_t idx_now = image_index_.fetch_add(1, std::memory_order_relaxed);
            const double ts_sec = static_cast<double>(stream->imageList[0].timestamp) / 1e9;
            const uint32_t jpeg_size = static_cast<uint32_t>(jpeg_data.size());
            std::vector<uint8_t> blob;
            blob.reserve(sizeof(uint32_t) + sizeof(double) + sizeof(uint32_t) + jpeg_size);
            auto append_pod = [&](const auto& v) {
                const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
                blob.insert(blob.end(), p, p + sizeof(v));
            };
            append_pod(idx_now);
            append_pod(ts_sec);
            append_pod(jpeg_size);
            blob.insert(blob.end(), jpeg_data.begin(), jpeg_data.end());
            data_logger_->enqueueImageFrame(std::move(blob));
        }

        // undistort image
        cv::Mat undistorted_image = cv::Mat::zeros(decoded_image.size(), decoded_image.type());
        cv_bridge::CvImage cv_undistorted_image;

        if (m_undistort_map_init_success) {
            cv::remap(decoded_image, undistorted_image, m_undistort_map_x, m_undistort_map_y, cv::INTER_LINEAR);
            if (getRosNodeControl()->useHostRosTime()) {
                #ifdef ROS2
                    cv_undistorted_image.header.stamp = node_->now();
                #else
                    cv_undistorted_image.header.stamp = ros::Time::now();
                #endif
            } else {
                cv_undistorted_image.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
            }
            cv_undistorted_image.encoding = "bgr8";
            cv_undistorted_image.image = undistorted_image;
        }

        #ifdef ROS2
        {
            rgb_pub_->publish(*cv_image.toImageMsg());
            if (m_undistort_map_init_success) {
                undistort_rgb_pub_->publish(*cv_undistorted_image.toImageMsg());
            }

            // original jpeg
            sensor_msgs::msg::CompressedImage jpeg_msg;
            if (getRosNodeControl()->useHostRosTime()) {
                jpeg_msg.header.stamp = node_->now();
            } else {
                jpeg_msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
            }
            jpeg_msg.format = "jpeg";
            jpeg_msg.data = jpeg_data;

            compressed_rgb_pub_->publish(jpeg_msg);
        }
        #else
        {
            rgb_pub_.publish(cv_image.toImageMsg());
            if (m_undistort_map_init_success) {
                undistort_rgb_pub_.publish(cv_undistorted_image.toImageMsg());
            }

            // original jpeg
            sensor_msgs::CompressedImagePtr jpeg_msg(new sensor_msgs::CompressedImage());
            if (getRosNodeControl()->useHostRosTime()) {
                jpeg_msg->header.stamp = ros::Time::now();
            } else {
                jpeg_msg->header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
            }
            jpeg_msg->format = "jpeg";
            jpeg_msg->data = jpeg_data;

            compressed_rgb_pub_.publish(jpeg_msg);
        }
        #endif
    }

}


    void publishPC2XYZRGBA(capture_Image_List_t* stream, int idx) 
    {
        #ifdef ROS2
                sensor_msgs::msg::PointCloud2 msg;
                msg.header.frame_id = "odom";
                if (getRosNodeControl()->useHostRosTime()) {
                    msg.header.stamp = node_->now();
                } else {
                    msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
                }

                //RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Point cloudrgba %ld",stream->imageList[0].timestamp);

                size_t pt_size = sizeof(int32_t) * 3 + sizeof(int32_t) * 4;
                uint32_t points = stream->imageList[idx].length / pt_size;

                msg.height = 1;
                msg.width = points;
                msg.is_dense = false;

                sensor_msgs::PointCloud2Modifier modifier(msg);
                modifier.setPointCloud2Fields(
                    4,
                    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
                );
                modifier.resize(msg.width * msg.height);

                sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
                sensor_msgs::PointCloud2Iterator<float> iter_rgb(msg, "rgb");
        #else
            sensor_msgs::PointCloud2 msg;
            msg.header.frame_id = "odom";
            if (getRosNodeControl()->useHostRosTime()) {
                msg.header.stamp = ros::Time::now();
            } else {
                msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
            }
            
            size_t pt_size = sizeof(int32_t) * 3 + sizeof(int32_t) * 4;
            uint32_t points = stream->imageList[idx].length / pt_size;
            
            msg.height = 1;
            msg.width = points;
            msg.is_dense = false;
            
            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::PointField::FLOAT32
            );
            modifier.resize(msg.width * msg.height);
            
            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_rgb(msg, "rgb");
        #endif
        
        // Shared data processing logic
        int32_t* xyz_data = static_cast<int32_t*>(stream->imageList[idx].pAddr);
        
        for(uint32_t i = 0; i < points; i++) {
            int32_t* ptr = xyz_data + 7*i;
            
#ifdef ROS2
                *iter_x = static_cast<float>(ptr[0]) / 10000.0f; ++iter_x;
                *iter_y = static_cast<float>(ptr[1]) / 10000.0f; ++iter_y;
                *iter_z = static_cast<float>(ptr[2]) / 10000.0f; ++iter_z;
#else
                *iter_x = (1.0 * ptr[0]) / 1e4; ++iter_x;
                *iter_y = (1.0 * ptr[1]) / 1e4; ++iter_y;
                *iter_z = (1.0 * ptr[2]) / 1e4; ++iter_z;
#endif
            
            uint8_t r = ptr[3] & 0xff;
            uint8_t g = ptr[4] & 0xff;
            uint8_t b = ptr[5] & 0xff;  
            
            uint32_t packed_rgb = (static_cast<uint32_t>(r) << 16) | 
                                (static_cast<uint32_t>(g) << 8)  | 
                                static_cast<uint32_t>(b);
            
            float rgb_float;
            std::memcpy(&rgb_float, &packed_rgb, sizeof(float));
            
            *iter_rgb = rgb_float; ++iter_rgb;
        }

        // Enqueue binary logging for point cloud (XYZRGB per point)
        if (data_logger_ && points > 0) {
            const double ts_sec = static_cast<double>(stream->imageList[0].timestamp) / 1e9;
            const uint32_t idx_now = cloud_index_.fetch_add(1, std::memory_order_relaxed);
            // Compute total blob size: header + per-point payload
            const size_t header_size = sizeof(uint32_t) + sizeof(double) + sizeof(uint32_t);
            const size_t point_size = sizeof(float) * 3 + sizeof(uint8_t) * 3;
            std::vector<uint8_t> blob;
            blob.reserve(header_size + static_cast<size_t>(points) * point_size);
            auto append_pod = [&](const auto& v) {
                const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
                blob.insert(blob.end(), p, p + sizeof(v));
            };
            append_pod(idx_now);
            append_pod(ts_sec);
            append_pod(points);

            for (uint32_t i = 0; i < points; ++i) {
                int32_t* ptr = xyz_data + 7 * i;
                float fx = static_cast<float>(ptr[0]) / 10000.0f;
                float fy = static_cast<float>(ptr[1]) / 10000.0f;
                float fz = static_cast<float>(ptr[2]) / 10000.0f;
                uint8_t r = static_cast<uint8_t>(ptr[3] & 0xff);
                uint8_t g = static_cast<uint8_t>(ptr[4] & 0xff);
                uint8_t b = static_cast<uint8_t>(ptr[5] & 0xff);
                append_pod(fx);
                append_pod(fy);
                append_pod(fz);
                blob.push_back(r);
                blob.push_back(g);
                blob.push_back(b);
            }
            data_logger_->enqueuePointCloudFrame(std::move(blob));
        }
        
#ifdef ROS2
            xyzrgbacloud_pub_->publish(std::move(msg));
#else
            xyzrgbacloud_pub_.publish(msg);
#endif
    }

    void publishOdometry(capture_Image_List_t* stream, OdometryType odom_type, bool show_path, bool show_camerapose) {
        
#ifdef ROS2
            auto msg = nav_msgs::msg::Odometry();
#else
            ros::Odometry msg;
#endif
        
            msg.header.frame_id = "odom";
            msg.child_frame_id = "odin1_base_link";

            //RCLCPP_INFO(rclcpp::get_logger("device_cb"), "odom %ld",odom_data->timestamp_ns);

            uint32_t data_len = stream->imageList[0].length;
            if (data_len == sizeof(ros_odom_convert_complete_t)) {

                ros_odom_convert_complete_t* odom_data = (ros_odom_convert_complete_t*)stream->imageList[0].pAddr;
                if (getRosNodeControl()->useHostRosTime()) {
                    #ifdef ROS2
                        msg.header.stamp = node_->now();
                    #else
                        msg.header.stamp = ros::Time::now();
                    #endif
                } else {
                    msg.header.stamp = ns_to_ros_time(odom_data->timestamp_ns);
                }

                msg.pose.pose.position.x = static_cast<double>(odom_data->pos[0]) / 1e6;
                msg.pose.pose.position.y = static_cast<double>(odom_data->pos[1]) / 1e6;
                msg.pose.pose.position.z = static_cast<double>(odom_data->pos[2]) / 1e6;

                msg.pose.pose.orientation.x = static_cast<double>(odom_data->orient[0]) / 1e6;
                msg.pose.pose.orientation.y = static_cast<double>(odom_data->orient[1]) / 1e6;
                msg.pose.pose.orientation.z = static_cast<double>(odom_data->orient[2]) / 1e6;
                msg.pose.pose.orientation.w = static_cast<double>(odom_data->orient[3]) / 1e6;

                // Enqueue binary logging for pose
                if ((odom_type == OdometryType::STANDARD) && data_logger_) {
                    const uint32_t idx_now = pose_index_.fetch_add(1, std::memory_order_relaxed);
                    const double ts_sec = static_cast<double>(odom_data->timestamp_ns) / 1e9;
                    float pose_arr[7];
                    pose_arr[0] = static_cast<float>(msg.pose.pose.position.x);
                    pose_arr[1] = static_cast<float>(msg.pose.pose.position.y);
                    pose_arr[2] = static_cast<float>(msg.pose.pose.position.z);
                    pose_arr[3] = static_cast<float>(msg.pose.pose.orientation.x);
                    pose_arr[4] = static_cast<float>(msg.pose.pose.orientation.y);
                    pose_arr[5] = static_cast<float>(msg.pose.pose.orientation.z);
                    pose_arr[6] = static_cast<float>(msg.pose.pose.orientation.w);
                    std::vector<uint8_t> blob;
                    blob.reserve(sizeof(uint32_t) + sizeof(double) + sizeof(float) * 7);
                    auto append_pod = [&](const auto& v) {
                        const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
                        blob.insert(blob.end(), p, p + sizeof(v));
                    };
                    append_pod(idx_now);
                    append_pod(ts_sec);
                    for (int i = 0; i < 7; ++i) append_pod(pose_arr[i]);
                    data_logger_->enqueuePoseFrame(std::move(blob));
                }
        
                msg.twist.twist.linear.x = static_cast<double>(odom_data->linear_velocity[0]) / 1e6;
                msg.twist.twist.linear.y = static_cast<double>(odom_data->linear_velocity[1]) / 1e6;
                msg.twist.twist.linear.z = static_cast<double>(odom_data->linear_velocity[2]) / 1e6;

                msg.twist.twist.angular.x = static_cast<double>(odom_data->angular_velocity[0]) / 1e6;
                msg.twist.twist.angular.y = static_cast<double>(odom_data->angular_velocity[1]) / 1e6;
                msg.twist.twist.angular.z = static_cast<double>(odom_data->angular_velocity[2]) / 1e6;

                msg.pose.covariance = {
                    static_cast<double>(odom_data->cov[0]) / 1e9, static_cast<double>(odom_data->cov[1]) / 1e9, static_cast<double>(odom_data->cov[2]) / 1e9, 0.0, 0.0, 0.0,
                    static_cast<double>(odom_data->cov[3]) / 1e9, static_cast<double>(odom_data->cov[4]) / 1e9, static_cast<double>(odom_data->cov[5]) / 1e9, 0.0, 0.0, 0.0,
                    static_cast<double>(odom_data->cov[6]) / 1e9, static_cast<double>(odom_data->cov[7]) / 1e9, static_cast<double>(odom_data->cov[8]) / 1e9, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, static_cast<double>(odom_data->cov[9]) / 1e9, static_cast<double>(odom_data->cov[10]) / 1e9, static_cast<double>(odom_data->cov[11]) / 1e9,
                    0.0, 0.0, 0.0, static_cast<double>(odom_data->cov[12]) / 1e9, static_cast<double>(odom_data->cov[13]) / 1e9, static_cast<double>(odom_data->cov[14]) / 1e9,
                    0.0, 0.0, 0.0, static_cast<double>(odom_data->cov[15]) / 1e9, static_cast<double>(odom_data->cov[16]) / 1e9, static_cast<double>(odom_data->cov[17]) / 1e9,
                };
            
            } else if (data_len == sizeof(ros2_odom_convert_t)) {

                ros2_odom_convert_t* odom_data = (ros2_odom_convert_t*)stream->imageList[0].pAddr;
                if (getRosNodeControl()->useHostRosTime()) {
                    #ifdef ROS2
                        msg.header.stamp = node_->now();
                    #else
                        msg.header.stamp = ros::Time::now();
                    #endif
                } else {
                    msg.header.stamp = ns_to_ros_time(odom_data->timestamp_ns);
                }

                msg.pose.pose.position.x = static_cast<double>(odom_data->pos[0]) / 1e6;
                msg.pose.pose.position.y = static_cast<double>(odom_data->pos[1]) / 1e6;
                msg.pose.pose.position.z = static_cast<double>(odom_data->pos[2]) / 1e6;

                msg.pose.pose.orientation.x = static_cast<double>(odom_data->orient[0]) / 1e6;
                msg.pose.pose.orientation.y = static_cast<double>(odom_data->orient[1]) / 1e6;
                msg.pose.pose.orientation.z = static_cast<double>(odom_data->orient[2]) / 1e6;
                msg.pose.pose.orientation.w = static_cast<double>(odom_data->orient[3]) / 1e6;
            }

#ifdef ROS2
            switch(odom_type) {
                case OdometryType::STANDARD:
                    {
                    if (getRosNodeControl()->sendOdomBaseLinkTF()) {
                        geometry_msgs::msg::TransformStamped transformStamped;
                        if (getRosNodeControl()->useHostRosTime()) {
                            transformStamped.header.stamp = node_->now();
                        } else {
                            transformStamped.header.stamp = msg.header.stamp;
                        }
                        transformStamped.header.frame_id = "odom";
                        transformStamped.child_frame_id = "odin1_base_link";
                        transformStamped.transform.translation.x = msg.pose.pose.position.x;
                        transformStamped.transform.translation.y = msg.pose.pose.position.y;
                        transformStamped.transform.translation.z = msg.pose.pose.position.z;
                        transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
                        transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
                        transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
                        transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
                        tf_broadcaster->sendTransform(transformStamped);
                    }
                    odom_publisher_->publish(msg);

                    // Publish odom trajectory as visualization markers (green lines connecting adjacent points)
                    static visualization_msgs::msg::Marker marker;
                    static std::vector<geometry_msgs::msg::Point> path_points;
                    
                    if (show_path) {
                        marker.header = msg.header;
                        marker.ns = "odom_trajectory";
                        marker.id = 0;
                        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                        marker.action = visualization_msgs::msg::Marker::ADD;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.02;  // Line width
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        marker.color.a = 1.0;
        
                        geometry_msgs::msg::Point pt;
                        pt.x = msg.pose.pose.position.x;
                        pt.y = msg.pose.pose.position.y;
                        pt.z = msg.pose.pose.position.z;
                        path_points.push_back(pt);
                        
                        // Keep only recent points to avoid memory issues (e.g., last 1000 points)
                        if (path_points.size() > 30000) {
                            path_points.erase(path_points.begin());
                        }
                        
                        marker.points = path_points;
        
                        // Publish marker array
                        static visualization_msgs::msg::MarkerArray marker_array;
                        marker_array.markers.clear();  // Clear previous markers
                        marker_array.markers.push_back(marker);
                        path_publisher_->publish(marker_array);
                    }

                    if (show_camerapose) {
                        // camera pose visualization (ROS2)
                        Eigen::Vector3d P(msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        msg.pose.pose.position.z);
                        Eigen::Quaterniond R(msg.pose.pose.orientation.w,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z);
                        if (extrinsic_ok_) {
                            P = P + R * t_ic_;
                            R = R * R_ic_;
                        }
                        cameraposevisual_.reset();
                        cameraposevisual_.add_pose(P, R);
                        cameraposevisual_.publish_by(*pub_camera_pose_visual_, msg.header);
                    }
                    }
                    break;
                case OdometryType::HIGHFREQ:
                    odom_highfreq_publisher_->publish(std::move(msg));
                    break;
                case OdometryType::TRANSFORM:
                    {
                    geometry_msgs::msg::TransformStamped transformStamped;
                    if (getRosNodeControl()->useHostRosTime()) {
                        transformStamped.header.stamp = node_->now();
                    } else {
                        transformStamped.header.stamp = msg.header.stamp;
                    }
                    transformStamped.header.frame_id = "odom";
                    transformStamped.child_frame_id = "map";
                    transformStamped.transform.translation.x = msg.pose.pose.position.x;
                    transformStamped.transform.translation.y = msg.pose.pose.position.y;
                    transformStamped.transform.translation.z = msg.pose.pose.position.z;
                    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
                    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
                    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
                    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
                    tf_broadcaster->sendTransform(transformStamped);
                    }
                    break;
            }
#else
            switch(odom_type) {
                case OdometryType::STANDARD:
                    {
                    if (getRosNodeControl()->sendOdomBaseLinkTF()) {
                        geometry_msgs::TransformStamped transformStamped;
                        if (getRosNodeControl()->useHostRosTime()) {
                            transformStamped.header.stamp = ros::Time::now();
                        } else {
                            transformStamped.header.stamp = msg.header.stamp;
                        }
                        transformStamped.header.frame_id = "odom";
                        transformStamped.child_frame_id = "odin1_base_link";
                        transformStamped.transform.translation.x = msg.pose.pose.position.x;
                        transformStamped.transform.translation.y = msg.pose.pose.position.y;
                        transformStamped.transform.translation.z = msg.pose.pose.position.z;
                        transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
                        transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
                        transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
                        transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
                        tf_broadcaster->sendTransform(transformStamped);
                    }
                    odom_publisher_.publish(msg);

                    if (show_path) {
                        // Publish odom trajectory as visualization markers (green lines connecting adjacent points)
                        static visualization_msgs::Marker marker;
                        static std::vector<geometry_msgs::Point> path_points;
                        
                        marker.header = msg.header;
                        marker.ns = "odom_trajectory";
                        marker.id = 0;
                        marker.type = visualization_msgs::Marker::LINE_STRIP;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.02;  // Line width
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        marker.color.a = 1.0;

                        geometry_msgs::Point pt;
                        pt.x = msg.pose.pose.position.x;
                        pt.y = msg.pose.pose.position.y;
                        pt.z = msg.pose.pose.position.z;
                        path_points.push_back(pt);
                        
                        // Keep only recent points to avoid memory issues (e.g., last 1000 points)
                        if (path_points.size() > 30000) {
                            path_points.erase(path_points.begin());
                        }
                        
                        marker.points = path_points;

                        // Publish marker array
                        static visualization_msgs::MarkerArray marker_array;
                        marker_array.markers.clear();  // Clear previous markers
                        marker_array.markers.push_back(marker);
                        path_publisher_.publish(marker_array);
                    }

                    if (show_camerapose) {
                        // camera pose visualization (ROS1)
                        Eigen::Vector3d P(msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        msg.pose.pose.position.z);
                        Eigen::Quaterniond R(msg.pose.pose.orientation.w,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z);
                            
                        if (extrinsic_ok_) {
                            P = P + R * t_ic_;
                            R = R * R_ic_;
                        }
                        cameraposevisual_.reset();
                        cameraposevisual_.add_pose(P, R);
                        cameraposevisual_.publish_by(pub_camera_pose_visual_, msg.header);
                    }
                    }
                    break;
                case OdometryType::HIGHFREQ:
                    odom_highfreq_publisher_.publish(msg);
                    break;
                case OdometryType::TRANSFORM:
                    {
                    geometry_msgs::TransformStamped transformStamped;
                    if (getRosNodeControl()->useHostRosTime()) {
                        transformStamped.header.stamp = ros::Time::now();
                    } else {
                        transformStamped.header.stamp = msg.header.stamp;
                    }
                    transformStamped.header.frame_id = "odom";
                    transformStamped.child_frame_id = "map";
                    transformStamped.transform.translation.x = msg.pose.pose.position.x;
                    transformStamped.transform.translation.y = msg.pose.pose.position.y;
                    transformStamped.transform.translation.z = msg.pose.pose.position.z;
                    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
                    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
                    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
                    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
                    tf_broadcaster->sendTransform(transformStamped);
                    }
                    break;
            }
#endif
    }

    void initialize_data_logger(std::string data_dir = "") {

        try {
            BinaryDataLogger::Options opts;
            opts.batch_size = 100;
            opts.base_dir = data_dir;
            data_logger_ = std::make_shared<BinaryDataLogger>(opts);
            root_dir_ = data_logger_->root_dir();
            #ifdef ROS2
                RCLCPP_INFO(node_->get_logger(), "Data logger initialized at %s", root_dir_.c_str());
            #endif
        } catch (...) {
            // Swallow logger initialization failures to avoid affecting runtime
            #ifdef ROS2
                RCLCPP_INFO(node_->get_logger(), "Failed to initialize data logger");
            #endif
            data_logger_.reset();
        }
    }

    int loadCameraParams(const std::string& yaml_file) {
        try {
            YAML::Node config = YAML::LoadFile(yaml_file);
    
            YAML::Node cam_node = config["cam_0"];
    
            m_camera_params.width = cam_node["image_width"].as<int>();
            m_camera_params.height = cam_node["image_height"].as<int>();
    
            double A11 = cam_node["A11"].as<double>();
            double A12 = cam_node["A12"].as<double>();
            double A22 = cam_node["A22"].as<double>();
            double u0 = cam_node["u0"].as<double>();
            double v0 = cam_node["v0"].as<double>();
    
            m_camera_params.fx = A11;
            m_camera_params.fy = A22;
            m_camera_params.cx = u0;
            m_camera_params.cy = v0;
            m_camera_params.skew = A12;
    
            m_camera_params.k2 = cam_node["k2"].as<double>();
            m_camera_params.k3 = cam_node["k3"].as<double>();
            m_camera_params.k4 = cam_node["k4"].as<double>();
            m_camera_params.k5 = cam_node["k5"].as<double>();
            m_camera_params.k6 = cam_node["k6"].as<double>();
            m_camera_params.k7 = cam_node["k7"].as<double>();
            m_camera_params.p1 = cam_node["p1"].as<double>();
            m_camera_params.p2 = cam_node["p2"].as<double>();
#if 0 
            std::cout << "成功读取相机参数:" << std::endl;
            std::cout << "图像尺寸: " << m_camera_params.width << "x" << m_camera_params.height << std::endl;
            std::cout << "焦距: fx=" << m_camera_params.fx << ", fy=" << m_camera_params.fy << std::endl;
            std::cout << "主点: cx=" << m_camera_params.cx << ", cy=" << m_camera_params.cy << std::endl;
            std::cout << "倾斜: " << m_camera_params.skew << std::endl;
            std::cout << "畸变系数: k2=" << m_camera_params.k2 << ", k3=" << m_camera_params.k3 
                      << ", k4=" << m_camera_params.k4 << ", k5=" << m_camera_params.k5 
                      << ", k6=" << m_camera_params.k6 << ", k7=" << m_camera_params.k7 << std::endl;
#endif            
            m_cam = std::make_unique<mini_vikit::PolynomialCamera>(m_camera_params.width, m_camera_params.height,
                m_camera_params.fx, m_camera_params.fy, m_camera_params.cx, m_camera_params.cy, m_camera_params.skew,
                m_camera_params.k2, m_camera_params.k3, m_camera_params.k4, m_camera_params.k5, m_camera_params.k6, m_camera_params.k7);

            // Load extrinsic Tcl_0 (camera to lidar)
            extrinsic_ok_ = false;
            if (config["Tcl_0"]) {
                auto T = config["Tcl_0"];
                if (T.IsSequence() && T.size() == 16) {
                    Eigen::Matrix4d Tcl;
                    for (int i = 0; i < 16; ++i) Tcl(i/4, i%4) = T[i].as<double>();
                    T_cl_ = Tcl;
                }
                std::cout << "Tcl: " << T_cl_ << std::endl;

                Eigen::Matrix4d Tic = T_il_ * T_cl_.inverse();
                Eigen::Matrix3d Ric = Tic.block<3,3>(0,0);
                Eigen::Vector3d tic = Tic.block<3,1>(0,3);
                R_ic_ = Eigen::Quaterniond(Ric);
                t_ic_ = tic;
                extrinsic_ok_ = true;
            }

            m_cam_init_success = true;
            return 0;
        } catch (const std::exception& e) {
            std::cerr << "读取YAML文件失败: " << e.what() << std::endl;
            m_cam_init_success = false;
            return -1;
        }
    }

    void buildUndistortMap()
    {
        m_undistort_map_x.create(m_camera_params.height, m_camera_params.width, CV_32F);
        m_undistort_map_y.create(m_camera_params.height, m_camera_params.width, CV_32F);

        for (int v_out = 0; v_out < m_camera_params.height; ++v_out) {
            for (int u_out = 0; u_out < m_camera_params.width; ++u_out) {
                double x_norm = (u_out - m_cam->cx()) / m_cam->fx();
                double y_norm = (v_out - m_cam->cy()) / m_cam->fy();

                // remove skew
                x_norm = x_norm - y_norm * m_cam->skew() / m_cam->fx();

                Eigen::Vector2d uv(x_norm, y_norm);
                Eigen::Vector2d distorted_pixel = m_cam->world2cam(uv);

                m_undistort_map_x.at<float>(v_out, u_out) = static_cast<float>(distorted_pixel[0]);
                m_undistort_map_y.at<float>(v_out, u_out) = static_cast<float>(distorted_pixel[1]);
            }
        }
        m_undistort_map_init_success = true;
    }

private:
    // Add the following member variables
    std::mutex rgb_queue_mutex_;
    std::deque<ImageConstPtr> rgb_image_queue_;
    const size_t max_rgb_queue_size_ = 10; // Cache up to 10 image frames
    
    std::mutex pcd_queue_mutex_;
    std::deque<PointCloud2ConstPtr> pcd_queue_;
    const size_t max_pcd_queue_size_ = 10; // Maximum cache frames

    // Binary logger and frame indices
    std::shared_ptr<BinaryDataLogger> data_logger_;
    std::atomic<uint32_t> pose_index_{0};
    std::atomic<uint32_t> cloud_index_{0};
    std::atomic<uint32_t> image_index_{0};

    std::filesystem::path root_dir_;

    // Updated helper functions
    ImageConstPtr getLatestRgbImage() {
        std::lock_guard<std::mutex> lock(rgb_queue_mutex_);
        return (!rgb_image_queue_.empty()) ? rgb_image_queue_.back() : nullptr;
    }

    PointCloud2ConstPtr getLatestIntensityCloud() {
        std::lock_guard<std::mutex> lock(pcd_queue_mutex_);
        return (!pcd_queue_.empty()) ? pcd_queue_.back() : nullptr;
    }
    std::vector<cv::Mat> getRgbImageQueueSnapshot() {
        std::lock_guard<std::mutex> lock(rgb_queue_mutex_);
        std::vector<cv::Mat> images;
        for (const auto& msg : rgb_image_queue_) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, "bgr8");
                images.push_back(cv_ptr->image.clone());
            } catch (cv_bridge::Exception& e) {
                #ifndef ROS2
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                #endif
            }
        }
        return images;
    }

    CameraParams m_camera_params;
    std::unique_ptr<mini_vikit::PolynomialCamera> m_cam;
    bool m_cam_init_success;
    bool m_undistort_map_init_success = false;
    cv::Mat m_undistort_map_x;
    cv::Mat m_undistort_map_y;
    Eigen::Quaterniond R_ic_ {Eigen::Quaterniond::Identity()};
    Eigen::Vector3d t_ic_ {Eigen::Vector3d::Zero()};
    bool extrinsic_ok_ {false};
    Eigen::Matrix4d T_il_ = (Eigen::Matrix4d() <<
        1.0, 0.0, 0.0, 0.00347,
        0.0, 1.0, 0.0, 0.03447,
        0.0, 0.0, 1.0, 0.02174,
        0.0, 0.0, 0.0, 1.0).finished();
    Eigen::Matrix4d T_cl_ = Eigen::Matrix4d::Identity(); // Camera->Lidar from YAML
#ifdef ROS2
    std::vector<sensor_msgs::msg::PointCloud2> getIntensityCloudQueueSnapshot() {
        std::lock_guard<std::mutex> lock(pcd_queue_mutex_);
        std::vector<sensor_msgs::msg::PointCloud2> clouds;
        
        for (const auto& msg_ptr : pcd_queue_) {
            clouds.push_back(*msg_ptr);
        }
        
        return clouds;
    }
#else
    std::vector<sensor_msgs::PointCloud2> getIntensityCloudQueueSnapshot() {
        std::lock_guard<std::mutex> lock(pcd_queue_mutex_);
        std::vector<sensor_msgs::PointCloud2> clouds;
        
        for (const auto& msg_ptr : pcd_queue_) {
            clouds.push_back(*msg_ptr);
        }
        
        return clouds;
    }
#endif

    void initialize_publishers() {
        #ifdef ROS2
            auto qos_profile = rclcpp::QoS(1)
                                    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

            imu_pub_ = node_->create_publisher<ros::Imu>("odin1/imu", qos_profile);
            rgb_pub_ = node_->create_publisher<ros::Image>("odin1/image", qos_profile);
            cloud_pub_ = node_->create_publisher<ros::PointCloud2>("odin1/cloud_raw", qos_profile);
            xyzrgbacloud_pub_ = node_->create_publisher<ros::PointCloud2>("odin1/cloud_slam", qos_profile);
            odom_publisher_ = node_->create_publisher<ros::Odometry>("odin1/odometry", qos_profile);
            odom_highfreq_publisher_ = node_->create_publisher<ros::Odometry>("odin1/odometry_highfreq", qos_profile);
            path_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("odin1/path", qos_profile);
            pub_camera_pose_visual_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("odin1/camera_pose_visual", qos_profile);
            rgbcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("odin1/cloud_render", qos_profile);
            compressed_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>("odin1/image/compressed", qos_profile);
            undistort_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("odin1/image/undistorted", qos_profile);
            intensity_gray_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("odin1/image/intensity_gray", qos_profile);
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        #endif
    }
    #ifdef ROS1
        void initialize_publishers(ros::NodeHandle& nh) {
            imu_pub_ = nh.advertise<ros::Imu>("odin1/imu", 4000);
            rgb_pub_ = nh.advertise<ros::Image>("odin1/image", 100);
            cloud_pub_ = nh.advertise<ros::PointCloud2>("odin1/cloud_raw", 100);
            xyzrgbacloud_pub_ = nh.advertise<ros::PointCloud2>("odin1/cloud_slam", 100);
            odom_publisher_ = nh.advertise<ros::Odometry>("odin1/odometry", 100);
            odom_highfreq_publisher_ = nh.advertise<ros::Odometry>("odin1/odometry_highfreq", 4000);
            path_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("odin1/path", 100);
            pub_camera_pose_visual_ = nh.advertise<visualization_msgs::MarkerArray>("odin1/camera_pose_visual", 100);
            rgbcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odin1/cloud_render", 100);
            compressed_rgb_pub_ = nh.advertise<sensor_msgs::CompressedImage>("odin1/image/compressed", 100);
            undistort_rgb_pub_ = nh.advertise<sensor_msgs::Image>("odin1/image/undistorted", 100);
            intensity_gray_pub_ = nh.advertise<sensor_msgs::Image>("odin1/image/intensity_gray", 100);
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
        }
    #endif

    #ifdef ROS2
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<ros::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<ros::Image>::SharedPtr rgb_pub_;
        rclcpp::Publisher<ros::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::Publisher<ros::PointCloud2>::SharedPtr xyzrgbacloud_pub_;
        rclcpp::Publisher<ros::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Publisher<ros::Odometry>::SharedPtr odom_highfreq_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rendered_cloud_pub_;
        rclcpp::Publisher<PointCloud2Msg>::SharedPtr rgbcloud_pub_;
        rclcpp::Publisher<ImageMsg>::SharedPtr rgbFromnv12_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_rgb_pub_; // New compressed image publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistort_rgb_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_gray_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual_;
        camera_pose_visualization cameraposevisual_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    #else
        ros::Publisher imu_pub_;
        ros::Publisher rgb_pub_;
        ros::Publisher cloud_pub_;
        ros::Publisher xyzrgbacloud_pub_;
        ros::Publisher odom_publisher_;
        ros::Publisher odom_highfreq_publisher_;
        ros::Publisher path_publisher_;
        ros::Publisher pub_camera_pose_visual_;
        camera_pose_visualization cameraposevisual_;
        ros::Publisher rendered_cloud_pub_;
        ros::Publisher rgbcloud_pub_;
        ros::Publisher rgbFromnv12_pub_;
        ros::Publisher compressed_rgb_pub_; // New compressed image publisher
        ros::Publisher undistort_rgb_pub_;
        ros::Publisher intensity_gray_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    #endif
};

class CommandLineControl {
public:
    using Callback = std::function<void(const std::string&, int)>;


    void register_key(const std::string& key, int default_value = 0) {
        std::lock_guard<std::mutex> lock(mtx);
        kv_map[key] = default_value;
    }

    void set(const std::string& key, int value) {
        Callback cb_to_invoke = nullptr;
        {
            std::lock_guard<std::mutex> lock(mtx);
            auto it = kv_map.find(key);
            if (it != kv_map.end()) {
                if (it->second != value) {
                    it->second = value;
                    cb_to_invoke = callback;
                } 
            } else {
                return;
            }
        }

        if (cb_to_invoke) {
            cb_to_invoke(key, value);
        }
    }

    int get(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mtx);
        auto it = kv_map.find(key);
        return (it != kv_map.end()) ? it->second : -1;
    }


    void register_callback(Callback cb) {
        std::lock_guard<std::mutex> lock(mtx);
        callback = cb;
    }

private:
    std::unordered_map<std::string, int> kv_map;
    std::thread input_thread;
    std::atomic<bool> running;
    mutable std::mutex mtx;
    Callback callback;

};

#define STREAMCTRL "streamctrl"        /* start/stop all streams */
#define SENDRGB    "sendrgb"           /* send RGB data */
#define SENDIMU    "sendimu"           /* send IMU data */
#define SENDODOM   "sendodom"          /* send odometry data */
#define SENDDTOF   "senddtof"          /* send raw cloud data */
#define SENDCLOUDSLAM  "sendcloudslam"      /* send rgb cloud data */
#define EXIT       "q"                 /* exit sample */
