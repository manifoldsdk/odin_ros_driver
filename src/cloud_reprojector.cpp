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
 * @file cloud_reprojector.cpp
 * @brief 点云重投影核心算法，利用鱼眼相机模型将里程计坐标系下的彩色点云
 *        变换到相机坐标系并投影到图像平面，生成彩色重投影图像。
 */

#include "cloud_reprojector.hpp"
#include <cmath>

/// 默认构造函数，等待调用 initialize() 后方可使用
CloudReprojector::CloudReprojector()
{
}

/// 初始化相机内参和外参，创建鱼眼相机模型，验证参数合法性
bool CloudReprojector::initialize(const CameraParams& cam_params, const ExtrinsicParams& ext_params)
{
    camera_params_ = cam_params;
    extrinsic_params_ = ext_params;

    if (camera_params_.A11 < 1e-6 || camera_params_.A22 < 1e-6 ||
        camera_params_.u0 < 1e-6 || camera_params_.v0 < 1e-6)
    {
        return false;
    }

    camera_model_ = std::make_unique<mini_vikit::PolynomialCamera>(
        camera_params_.image_width, camera_params_.image_height,
        camera_params_.A11, camera_params_.A22,
        camera_params_.u0, camera_params_.v0,
        camera_params_.A12,
        camera_params_.k2, camera_params_.k3, camera_params_.k4,
        camera_params_.k5, camera_params_.k6, camera_params_.k7
    );

    initialized_ = true;
    return true;
}

/// 由相机-激光雷达变换矩阵 Tcl 和激光雷达-IMU 变换矩阵 Til 计算相机-IMU 变换矩阵 Tic
Eigen::Matrix4d CloudReprojector::calculateTic(const Eigen::Matrix4d& Tcl, const Eigen::Matrix4d& Til)
{
    // Tic = Til * Tlc = Til * Tcl.inverse()
    Eigen::Matrix4d Tlc = Tcl.inverse();
    return Til * Tlc;
}

/// 将里程计位姿（四元数 + 平移向量）转换为 4x4 齐次变换矩阵
Eigen::Matrix4d CloudReprojector::odomPoseToMatrix(const OdomPose& odom) const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = odom.orientation.toRotationMatrix();
    T(0, 3) = odom.position.x();
    T(1, 3) = odom.position.y();
    T(2, 3) = odom.position.z();
    return T;
}

/// 根据里程计位姿将里程计坐标系点云变换到相机坐标系，并调用投影函数生成图像
cv::Mat CloudReprojector::reprojectCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_odom,
                                          const OdomPose& odom_pose)
{
    if (!initialized_)
    {
        return cv::Mat();
    }

    // T_odom_imu: imu pose in odom frame
    Eigen::Matrix4d T_odom_imu = odomPoseToMatrix(odom_pose);

    // T_imu_odom: transforms points from odom frame to imu frame
    Eigen::Matrix4d T_imu_odom = T_odom_imu.inverse();

    // T_cam_imu = Tic.inverse(): transforms from imu to camera
    Eigen::Matrix4d T_cam_imu = extrinsic_params_.Tic.inverse();

    // T_cam_odom: transforms points from odom frame to camera frame
    Eigen::Matrix4d T_cam_odom = T_cam_imu * T_imu_odom;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_cam;
    pcl::transformPointCloud(cloud_odom, cloud_in_cam, T_cam_odom);

    return projectCloudToImage(cloud_in_cam);
}

/// 将相机坐标系下的彩色点云通过鱼眼相机模型投影到图像平面，以圆点绘制每个有效点
cv::Mat CloudReprojector::projectCloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in_cam) const
{
    cv::Mat img = cv::Mat::zeros(camera_params_.image_height, camera_params_.image_width, CV_8UC3);
    img.setTo(cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud_in_cam)
    {
        if (pt.z <= 0.01)
            continue;

        Eigen::Vector3d pt_cam(pt.x, pt.y, pt.z);
        Eigen::Vector2d uv = camera_model_->world2cam(pt_cam);

        int u_int = static_cast<int>(std::round(uv[0]));
        int v_int = static_cast<int>(std::round(uv[1]));

        if (u_int >= 0 && u_int < camera_params_.image_width &&
            v_int >= 0 && v_int < camera_params_.image_height)
        {
            cv::circle(img, cv::Point(u_int, v_int), point_radius_,
                       cv::Scalar(pt.b, pt.g, pt.r), -1);
        }
    }

    return img;
}
