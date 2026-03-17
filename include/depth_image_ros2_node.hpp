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

/**
 * @file depth_image_ros2_node.hpp
 * @brief 深度图生成 ROS2 节点（ROS2 Humble）
 *
 * 订阅原始点云（cloud_raw）和 RGB 图像，同步后利用
 * PointCloudToDepthConverter 生成密集深度图并发布。
 * 同时输出带颜色的点云（depth_cloud）话题。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include "pointcloud_depth_converter.hpp"

#include <string>
#include <memory>

/**
 * @brief 深度图生成节点类（ROS2）
 *
 * 继承 rclcpp::Node，通过时间同步策略对齐点云与 RGB 图像，
 * 调用 PointCloudToDepthConverter 生成深度图并发布。
 */
class DepthImageRos2Node : public rclcpp::Node
{
public:
    /// 构造函数，初始化节点参数和订阅者
    explicit DepthImageRos2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /// 完成订阅者和发布者的初始化绑定
    void initialize();

private:
    std::string cloud_raw_topic_;         ///< 订阅的原始点云话题名
    std::string color_compressed_topic_;  ///< 订阅的压缩 RGB 图像话题名
    std::string color_raw_topic_;         ///< 订阅的解压 RGB 图像话题名
    std::string depth_image_topic_;       ///< 发布的深度图话题名
    std::string depth_cloud_topic_;       ///< 发布的彩色深度点云话题名

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> color_compressed_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;

    /// 时间近似同步策略（点云 + RGB 图像）
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher depth_image_pub_;                                  ///< 深度图发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_; ///< 深度点云发布者

    std::unique_ptr<PointCloudToDepthConverter> depth_converter_; ///< 深度转换器实例

    /// 从节点参数加载相机内参和外参
    PointCloudToDepthConverter::CameraParams loadCameraParams();

    /// 同步回调：接收对齐的点云和 RGB 图像，生成深度图
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr color_msg);

    /// 发布深度图（默认 32FC1 浮点格式）
    void publishDepthImage(const cv::Mat &img,
                           const std_msgs::msg::Header &header,
                           const std::string &encoding = "32FC1");

    /// 发布彩色深度点云
    void publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                           const std_msgs::msg::Header &header);
};
