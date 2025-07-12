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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <Eigen/Dense>
#include "lidar_api.h"
#include "lidar_api_type.h"


#ifdef ROS2

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include "sensor_msgs/msg/imu.hpp"
    #include "sensor_msgs/msg/point_cloud2.hpp"
    #include "sensor_msgs/point_cloud2_iterator.hpp"
    #include <sensor_msgs/msg/imu.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <sensor_msgs/msg/point_cloud2.hpp>
    #include <sensor_msgs/msg/point_field.hpp>
    namespace ros {
        using namespace rclcpp;
        using namespace std_msgs::msg;
        using namespace sensor_msgs::msg;
        using namespace nav_msgs::msg;
        using Time = builtin_interfaces::msg::Time;
    }
#else

    #include <ros/ros.h>
    #include <ros/package.h>
    #include <sensor_msgs/Image.h>
    #include <sensor_msgs/Imu.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/point_cloud2_iterator.h>
    #include <nav_msgs/Odometry.h>
    namespace ros {
        using namespace ::ros;
        using namespace sensor_msgs;
        using namespace nav_msgs;
    }
#endif

// Common definitions
#define GD_ACCL_G 9.7833f
#define ACC_1G_ms2 9.8
#define ACC_SEN_SCALE 16348
#define PAI 3.14159265358979323846
#define GYRO_SEN_SCALE 32.8f

// Common functions
inline float accel_convert(int16_t raw, int sen_scale) {
    return (raw * GD_ACCL_G / sen_scale);
}

inline float gyro_convert(int16_t raw, float sen_scale) {
    return (raw * PAI) / (sen_scale * 180); 
}

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

// Multi-sensor publisher class
class MultiSensorPublisher {
public:
    #ifdef ROS2
        MultiSensorPublisher(rclcpp::Node::SharedPtr node)
            : node_(node) {
            initialize_publishers();
        }
    #else
        MultiSensorPublisher(ros::NodeHandle& nh) {
            initialize_publishers(nh);
        }
    #endif

    void publishImu(icm_6aixs_data_t *stream) {
        #ifdef ROS2
            sensor_msgs::msg::Imu imu_msg;
        #else
            ros::Imu imu_msg;
        #endif
        
        imu_msg.header.stamp = ns_to_ros_time(stream->stamp);
        imu_msg.header.frame_id = "imu_link";
        
        imu_msg.linear_acceleration.y = -1 * static_cast<double>(accel_convert(stream->aacx, ACC_SEN_SCALE));
        imu_msg.linear_acceleration.x = static_cast<double>(accel_convert(stream->aacy, ACC_SEN_SCALE));
        imu_msg.linear_acceleration.z = static_cast<double>(accel_convert(stream->aacz, ACC_SEN_SCALE));
        
        imu_msg.angular_velocity.y = -1 * static_cast<double>(gyro_convert(stream->gyrox, GYRO_SEN_SCALE));
        imu_msg.angular_velocity.x = static_cast<double>(gyro_convert(stream->gyroy, GYRO_SEN_SCALE));
        imu_msg.angular_velocity.z = static_cast<double>(gyro_convert(stream->gyroz, GYRO_SEN_SCALE));
        
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

    void publishIntensityCloud(capture_Image_List_t* stream, int idx) 
    {
        #ifdef ROS2
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.frame_id = "map";
            msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
        
            msg.height = stream->imageList[idx].height;
            msg.width = stream->imageList[idx].width;
            msg.is_dense = false;

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::UINT8
            );
            modifier.resize(stream->imageList[idx].width * stream->imageList[idx].height);
        
            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(msg, "intensity");
        #else
            sensor_msgs::PointCloud2 msg;
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();  
            msg.height = stream->imageList[idx].height;
            msg.width = stream->imageList[idx].width;
            msg.is_dense = false;
            msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::UINT8
            );
            modifier.resize(msg.height * msg.width);

            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(msg, "intensity");
        #endif

        float* xyz_data = static_cast<float*>(stream->imageList[idx].pAddr);
        uint16_t* intensity_data = static_cast<uint16_t*>(stream->imageList[2].pAddr);

        int total_points = stream->imageList[idx].height * stream->imageList[idx].width;
        for (int i = 0; i < total_points; ++i) {
            float* pf = xyz_data + i * 4;

            #ifdef ROS2
                *iter_x = pf[2] / 1000.0f; ++iter_x;
                *iter_y = -pf[0] / 1000.0f; ++iter_y;
                *iter_z = pf[1] / 1000.0f; ++iter_z;
                *iter_intensity = static_cast<uint8_t>(intensity_data[i] >> 8); ++iter_intensity;
            #else
                *iter_x = pf[2] / 1000.0f; ++iter_x;
                *iter_y = -pf[0] / 1000.0f; ++iter_y;
                *iter_z = pf[1] / 1000.0f; ++iter_z;
                *iter_intensity = static_cast<uint8_t>(intensity_data[i] >> 8); ++iter_intensity;
            #endif
        }

        // Publish Intensity point cloud
        #ifdef ROS2
            cloud_pub_->publish(std::move(msg));
        #else
            cloud_pub_.publish(msg);
        #endif
    }

    void publishRgb(capture_Image_List_t *stream) {
        buffer_List_t &image = stream->imageList[0];
        
        // Validate image parameters
        if (!image.pAddr) {
            #ifdef ROS2
                RCLCPP_ERROR(node_->get_logger(), "Invalid RGB image: null data pointer");
            #else
                ROS_ERROR("Invalid RGB image: null data pointer");
            #endif
            return;
        }
        
        if (image.width <= 0 || image.height <= 0) {
            #ifdef ROS2
                RCLCPP_ERROR(node_->get_logger(), "Invalid RGB image dimensions: %dx%d", image.width, image.height);
            #else
                ROS_ERROR("Invalid RGB image dimensions: %dx%d", image.width, image.height);
            #endif
            return;
        }
        
        // Calculate NV12 image height
        const int height_nv12 = image.height * 3 / 2;
        
        // Validate NV12 image size
        const size_t expected_size = static_cast<size_t>(image.width) * height_nv12;
        if (image.length < expected_size) {
            #ifdef ROS2
                RCLCPP_ERROR(node_->get_logger(), "RGB buffer too small: expected %zu bytes, got %u bytes", 
                            expected_size, image.length);
            #else
                ROS_ERROR("RGB buffer too small: expected %zu bytes, got %u bytes", 
                         expected_size, image.length);
            #endif
            return;
        }
        
        try {
            cv::Mat nv12_mat(height_nv12, image.width, CV_8UC1, image.pAddr);
            cv::Mat bgr;
            cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);
            
            if (bgr.empty()) {
                #ifdef ROS2
                    RCLCPP_ERROR(node_->get_logger(), "Failed to convert NV12 to BGR");
                #else
                    ROS_ERROR("Failed to convert NV12 to BGR");
                #endif
                return;
            }
            
            #ifdef ROS2
                std_msgs::msg::Header header;
                sensor_msgs::msg::Image::SharedPtr msg;
            #else
                std_msgs::Header header;
                sensor_msgs::Image msg;
            #endif
            
            header.stamp = ns_to_ros_time(image.timestamp + 719060);
            header.frame_id = "camera_rgb_frame";
            
            #ifdef ROS2
                auto cv_image = std::make_shared<cv_bridge::CvImage>(header, "bgr8", bgr);
                msg = cv_image->toImageMsg();
                rgb_pub_->publish(*msg);
            #else
                cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg(msg);
                rgb_pub_.publish(msg);
            #endif
            
        } catch (const cv::Exception& e) {
            #ifdef ROS2
                RCLCPP_ERROR(node_->get_logger(), "OpenCV error in publishRgb: %s", e.what());
            #else
                ROS_ERROR("OpenCV error in publishRgb: %s", e.what());
            #endif
        } catch (const std::exception& e) {
            #ifdef ROS2
                RCLCPP_ERROR(node_->get_logger(), "Exception in publishRgb: %s", e.what());
            #else
                ROS_ERROR("Exception in publishRgb: %s", e.what());
            #endif
        }
    }

    void publishPC2XYZRGBA(capture_Image_List_t* stream, int idx) 
    {
        static int flag = 1;
        #ifdef ROS2
                sensor_msgs::msg::PointCloud2 msg;
                // msg.header.frame_id = "base_link";
                msg.header.frame_id = "map";
                msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
                // msg.header.stamp = this->now();
                size_t pt_size = sizeof(int32_t) * 3 + sizeof(int32_t) * 4;
                uint32_t points = stream->imageList[idx].length / pt_size;

                msg.height = 1;
                msg.width = points;
                msg.is_dense = false;
                // LOG_INFO("msg.height=%ld, msg.width=%ld.\n", msg.height, msg.width);

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
            msg.header.frame_id = "map";
            msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
            
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
        
        #ifdef ROS2
            flag = 0; // Preserve assignment if flag is used elsewhere
            xyzrgbacloud_pub_->publish(std::move(msg));
        #else
            flag = 0;
            xyzrgbacloud_pub_.publish(msg);
        #endif
    }

    void publishOdometry(capture_Image_List_t* stream) {
        ros2_odom_convert_t* odom_data = (ros2_odom_convert_t*)stream->imageList[0].pAddr;
        
        #ifdef ROS2
            auto msg = nav_msgs::msg::Odometry();
        #else
            ros::Odometry msg;
        #endif
        
            msg.header.stamp = ns_to_ros_time(odom_data->timestamp_ns);
            msg.header.frame_id = "map";
            msg.child_frame_id = "base_link";

            msg.pose.pose.position.x = static_cast<double>(odom_data->pos[0]) / 1e6;
            msg.pose.pose.position.y = static_cast<double>(odom_data->pos[1]) / 1e6;
            msg.pose.pose.position.z = static_cast<double>(odom_data->pos[2]) / 1e6;

            msg.pose.pose.orientation.x = static_cast<double>(odom_data->orient[0]) / 1e6;
            msg.pose.pose.orientation.y = static_cast<double>(odom_data->orient[1]) / 1e6;
            msg.pose.pose.orientation.z = static_cast<double>(odom_data->orient[2]) / 1e6;
            msg.pose.pose.orientation.w = static_cast<double>(odom_data->orient[3]) / 1e6;
        
        #ifdef ROS2
            odom_publisher_->publish(std::move(msg));
        #else
            odom_publisher_.publish(msg);
        #endif
    }

private:
    void initialize_publishers() {
        #ifdef ROS2
            imu_pub_ = node_->create_publisher<ros::Imu>("odin1/imu", 10);
            rgb_pub_ = node_->create_publisher<ros::Image>("odin1/image", 10);
            cloud_pub_ = node_->create_publisher<ros::PointCloud2>("odin1/cloud_raw", 10);
            xyzrgbacloud_pub_ = node_->create_publisher<ros::PointCloud2>("odin1/cloud_slam", 10);
            odom_publisher_ = node_->create_publisher<ros::Odometry>("odin1/odometry_map", 10);
        #endif
    }
    
    #ifdef ROS1
        void initialize_publishers(ros::NodeHandle& nh) {
            imu_pub_ = nh.advertise<ros::Imu>("odin1/imu", 10);
            rgb_pub_ = nh.advertise<ros::Image>("odin1/image", 10);
            cloud_pub_ = nh.advertise<ros::PointCloud2>("odin1/cloud_raw", 10);
            xyzrgbacloud_pub_ = nh.advertise<ros::PointCloud2>("odin1/cloud_slam", 10);
            odom_publisher_ = nh.advertise<ros::Odometry>("odin1/odometry_map", 10);
        }
    #endif

    #ifdef ROS2
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<ros::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<ros::Image>::SharedPtr rgb_pub_;
        rclcpp::Publisher<ros::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::Publisher<ros::PointCloud2>::SharedPtr xyzrgbacloud_pub_;
        rclcpp::Publisher<ros::Odometry>::SharedPtr odom_publisher_;
    #else
        ros::Publisher imu_pub_;
        ros::Publisher rgb_pub_;
        ros::Publisher cloud_pub_;
        ros::Publisher xyzrgbacloud_pub_;
        ros::Publisher odom_publisher_;
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
                    std::cout << "Set " << key << " = " << value << std::endl;
                    cb_to_invoke = callback;
                } else {
                    std::cout << "Set ignored: " << key << " is already " << value << std::endl;
                }
            } else {
                std::cout << "Unknown key: " << key << std::endl;
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

    void print_all() const {
        std::lock_guard<std::mutex> lock(mtx);
        std::cout << "Available keys and values:" << std::endl;
        for (const auto& [key, value] : kv_map) {
            std::cout << "  " << key << " = " << value << std::endl;
        }
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
