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

#ifdef ROS2
    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/point_cloud2.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <image_transport/image_transport.hpp>
    // cv_bridge header: ROS2 humble+ provides cv_bridge.hpp; ROS2 jazzy removes the legacy .h.
    // Prefer .hpp when available, fall back to .h for older ROS distros (e.g. foxy/galactic).
    #if defined(__has_include)
    #  if __has_include(<cv_bridge/cv_bridge.hpp>)
    #    include <cv_bridge/cv_bridge.hpp>
    #  else
    #    include <cv_bridge/cv_bridge.h>
    #  endif
    #else
    #  include <cv_bridge/cv_bridge.h>
    #endif
    #include <message_filters/subscriber.h>
    #include <message_filters/time_synchronizer.h>
    #include <message_filters/sync_policies/exact_time.h>
    #include <message_filters/sync_policies/approximate_time.h>
#else
    #include <ros/ros.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/Image.h>
    #include <nav_msgs/Odometry.h>
    #include <cv_bridge/cv_bridge.h>
    #include <message_filters/subscriber.h>
    #include <message_filters/time_synchronizer.h>
    #include <message_filters/sync_policies/exact_time.h>
    #include <message_filters/sync_policies/approximate_time.h>
#endif

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cloud_reprojector.hpp"

#include <string>
#include <memory>

#ifdef ROS2
class CloudReprojectionRosNode : public rclcpp::Node
{
public:
    CloudReprojectionRosNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Odometry = nav_msgs::msg::Odometry;
    using Image = sensor_msgs::msg::Image;

    std::string cloud_slam_topic_;
    std::string odometry_topic_;
    std::string wiwc_topic_;
    std::string reprojected_image_topic_;

    message_filters::Subscriber<PointCloud2> cloud_sub_;
    message_filters::Subscriber<Odometry> odom_sub_;
    message_filters::Subscriber<Odometry> wiwc_sub_;

    typedef message_filters::sync_policies::ApproximateTime<PointCloud2, Odometry, Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    image_transport::Publisher reprojected_image_pub_;

    std::unique_ptr<CloudReprojector> reprojector_;

    void loadParameters();
    void syncCallback(const PointCloud2::ConstSharedPtr& cloud_msg,
                      const Odometry::ConstSharedPtr& odom_msg,
                      const Odometry::ConstSharedPtr& wiwc_msg);
};
#else
class CloudReprojectionRosNode
{
public:
    CloudReprojectionRosNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    ros::NodeHandle nh_, pnh_;

    std::string cloud_slam_topic_;
    std::string odometry_topic_;
    std::string wiwc_topic_;
    std::string reprojected_image_topic_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> wiwc_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    ros::Publisher reprojected_image_pub_;

    std::unique_ptr<CloudReprojector> reprojector_;

    void loadParameters();
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                      const nav_msgs::OdometryConstPtr& odom_msg,
                      const nav_msgs::OdometryConstPtr& wiwc_msg);
};
#endif
