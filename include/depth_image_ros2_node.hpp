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

class DepthImageRos2Node : public rclcpp::Node
{
public:
    explicit DepthImageRos2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void initialize();

private:
    std::string cloud_raw_topic_;
    std::string color_compressed_topic_;
    std::string color_raw_topic_;
    std::string depth_image_topic_;
    std::string depth_cloud_topic_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> color_compressed_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, 
        // sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;

    std::unique_ptr<PointCloudToDepthConverter> depth_converter_;


    PointCloudToDepthConverter::CameraParams loadCameraParams();

    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                      // const sensor_msgs::msg::CompressedImage::ConstSharedPtr image_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr color_msg);


    void publishDepthImage(const cv::Mat &img,
                           const std_msgs::msg::Header &header,
                           const std::string &encoding = "32FC1");

    void publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                           const std_msgs::msg::Header &header);
};
