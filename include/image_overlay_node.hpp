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
    #include <sensor_msgs/msg/image.hpp>
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
    #include <mutex>
#else
    #include <ros/ros.h>
    #include <sensor_msgs/Image.h>
    #include <cv_bridge/cv_bridge.h>
    #include <message_filters/subscriber.h>
    #include <message_filters/sync_policies/approximate_time.h>
    #include <message_filters/synchronizer.h>
#endif

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

#ifdef ROS2
class ImageOverlayNode : public rclcpp::Node
{
public:
    ImageOverlayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using Image = sensor_msgs::msg::Image;

    std::string reprojected_topic_;
    std::string camera_topic_;
    std::string overlay_topic_;
    double alpha_;  // blend alpha for overlay

    rclcpp::Subscription<Image>::SharedPtr reproj_sub_;
    rclcpp::Subscription<Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<Image>::SharedPtr overlay_pub_;

    // Cache latest images
    cv::Mat latest_reproj_img_;
    cv::Mat latest_camera_img_;
    std_msgs::msg::Header latest_header_;
    std::mutex mutex_;

    void reprojCallback(Image::ConstSharedPtr msg);
    void cameraCallback(Image::ConstSharedPtr msg);
    void publishOverlay();
};
#else
#include <mutex>
class ImageOverlayNode
{
public:
    ImageOverlayNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    ros::NodeHandle nh_, pnh_;

    std::string reprojected_topic_;
    std::string camera_topic_;
    std::string overlay_topic_;
    double alpha_;  // blend alpha for overlay

    ros::Subscriber reproj_sub_;
    ros::Subscriber camera_sub_;
    ros::Publisher overlay_pub_;

    // Cache latest images
    cv::Mat latest_reproj_img_;
    cv::Mat latest_camera_img_;
    std_msgs::Header latest_header_;
    std::mutex mutex_;

    void reprojCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void publishOverlay();
};
#endif
