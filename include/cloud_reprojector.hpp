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

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "polynomial_camera.hpp"

#include <memory>

class CloudReprojector
{
public:
    struct CameraParams
    {
        int image_width = 1600;
        int image_height = 1296;
        double A11 = 0.0, A12 = 0.0, A22 = 0.0;
        double u0 = 0.0, v0 = 0.0;
        double k2 = 0.0, k3 = 0.0, k4 = 0.0, k5 = 0.0, k6 = 0.0, k7 = 0.0;
    };

    struct ExtrinsicParams
    {
        Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity();  // camera to lidar
        Eigen::Matrix4d Til = Eigen::Matrix4d::Identity();  // lidar to imu (fixed)
        Eigen::Matrix4d Tic = Eigen::Matrix4d::Identity();  // camera to imu (calculated)
    };

    struct OdomPose
    {
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
    };

    CloudReprojector();
    ~CloudReprojector() = default;

    bool initialize(const CameraParams& cam_params, const ExtrinsicParams& ext_params);

    cv::Mat reprojectCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_odom,
                           const OdomPose& odom_pose);

    void setPointRadius(int radius) { point_radius_ = radius; }
    int getPointRadius() const { return point_radius_; }

    const CameraParams& getCameraParams() const { return camera_params_; }
    const ExtrinsicParams& getExtrinsicParams() const { return extrinsic_params_; }

    static Eigen::Matrix4d calculateTic(const Eigen::Matrix4d& Tcl, const Eigen::Matrix4d& Til);

private:
    Eigen::Matrix4d odomPoseToMatrix(const OdomPose& odom) const;

    cv::Mat projectCloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in_cam) const;

    CameraParams camera_params_;
    ExtrinsicParams extrinsic_params_;
    std::unique_ptr<mini_vikit::PolynomialCamera> camera_model_;

    int point_radius_ = 4;
    bool initialized_ = false;
};
