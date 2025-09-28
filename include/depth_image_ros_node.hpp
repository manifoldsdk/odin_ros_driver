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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

#include "pointcloud_depth_converter.hpp"

#include <string>
#include <memory>


class DepthImageRosNode
{
public:
    DepthImageRosNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;


    std::string cloud_raw_topic_;
    std::string color_raw_topic_;
    std::string color_compressed_topic_;
    std::string depth_image_topic_;
    std::string depth_cloud_topic_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> color_compressed_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    image_transport::Publisher depth_image_pub_;
    ros::Publisher depth_cloud_pub_;

    std::unique_ptr<PointCloudToDepthConverter> depth_converter_;

    PointCloudToDepthConverter::CameraParams loadCameraParams();


    void syncCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                      const sensor_msgs::ImageConstPtr &image_msg);


    void publishDepthImage(const cv::Mat &img,
                           const std_msgs::Header &header,
                           const std::string &encoding = "32FC1");


    void publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                           const std_msgs::Header &header);
};
