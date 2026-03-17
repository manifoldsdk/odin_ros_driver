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
 * @file cloud_reprojection_ros_node.hpp
 * @brief 点云重投影 ROS2 节点（ROS2 Humble）
 *
 * 订阅 SLAM 点云（cloud_slam）和里程计（odom），
 * 时间精确同步后将三维点云重投影到相机图像平面，
 * 发布重投影结果图像（reprojected_image 话题）。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cloud_reprojector.hpp"

#include <string>
#include <memory>

/**
 * @brief 点云重投影节点类（ROS2）
 *
 * 使用精确时间同步策略对齐点云和里程计消息，
 * 利用 CloudReprojector 将全局点云投影到当前帧相机图像并发布。
 */
class CloudReprojectionRosNode : public rclcpp::Node
{
public:
    /// 构造函数，初始化节点并加载参数
    CloudReprojectionRosNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Odometry    = nav_msgs::msg::Odometry;
    using Image       = sensor_msgs::msg::Image;

    std::string cloud_slam_topic_;          ///< 订阅的 SLAM 点云话题名
    std::string odometry_topic_;            ///< 订阅的里程计话题名
    std::string reprojected_image_topic_;   ///< 发布的重投影图像话题名

    message_filters::Subscriber<PointCloud2> cloud_sub_; ///< 点云订阅者
    message_filters::Subscriber<Odometry> odom_sub_;     ///< 里程计订阅者

    /// 精确时间同步策略
    typedef message_filters::sync_policies::ExactTime<PointCloud2, Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    image_transport::Publisher reprojected_image_pub_; ///< 重投影图像发布者

    std::unique_ptr<CloudReprojector> reprojector_; ///< 点云重投影器实例

    /// 从节点参数加载相机内参和外参，初始化重投影器
    void loadParameters();

    /// 同步回调：接收对齐的点云和里程计，执行重投影并发布图像
    void syncCallback(const PointCloud2::ConstSharedPtr& cloud_msg,
                      const Odometry::ConstSharedPtr& odom_msg);
};
