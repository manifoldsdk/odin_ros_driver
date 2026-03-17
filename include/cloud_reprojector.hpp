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
 * @file cloud_reprojector.hpp
 * @brief 点云重投影核心算法模块
 *
 * 将全局坐标系下的 SLAM 点云（带颜色）根据当前里程计位姿
 * 变换到相机坐标系，再使用多项式相机模型投影到图像平面。
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

/**
 * @brief 点云重投影器
 *
 * 根据相机内参、相机-LiDAR 外参和当前位姿，
 * 将彩色点云重投影到二维图像平面并绘制结果图像。
 */
class CloudReprojector
{
public:
    /// 相机内参结构体
    struct CameraParams
    {
        int image_width  = 1600; ///< 图像宽度
        int image_height = 1296; ///< 图像高度
        double A11 = 0.0;        ///< fx（水平焦距）
        double A12 = 0.0;        ///< 倾斜系数 skew
        double A22 = 0.0;        ///< fy（垂直焦距）
        double u0  = 0.0;        ///< 主点 cx
        double v0  = 0.0;        ///< 主点 cy
        double k2 = 0.0, k3 = 0.0, k4 = 0.0; ///< 多项式畸变系数（低阶）
        double k5 = 0.0, k6 = 0.0, k7 = 0.0; ///< 多项式畸变系数（高阶）
    };

    /// 外参参数结构体（相机、LiDAR、IMU 之间的变换关系）
    struct ExtrinsicParams
    {
        Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity(); ///< 相机到 LiDAR 的变换矩阵
        Eigen::Matrix4d Til = Eigen::Matrix4d::Identity(); ///< LiDAR 到 IMU 的固定变换矩阵
        Eigen::Matrix4d Tic = Eigen::Matrix4d::Identity(); ///< 相机到 IMU 的变换矩阵（由 Tcl 和 Til 计算得到）
    };

    /// 里程计位姿结构体
    struct OdomPose
    {
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); ///< 方向四元数
        Eigen::Vector3d position = Eigen::Vector3d::Zero();              ///< 平移向量
    };

    CloudReprojector();
    ~CloudReprojector() = default;

    /**
     * @brief 初始化重投影器，加载相机模型和外参
     * @param cam_params 相机内参
     * @param ext_params 相机-LiDAR-IMU 外参
     * @return true 初始化成功
     */
    bool initialize(const CameraParams& cam_params, const ExtrinsicParams& ext_params);

    /**
     * @brief 将全局坐标系点云重投影到当前相机图像
     * @param cloud_odom 里程计坐标系下的彩色点云
     * @param odom_pose  当前里程计位姿
     * @return 重投影结果图像（BGR 格式）
     */
    cv::Mat reprojectCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_odom,
                           const OdomPose& odom_pose);

    /// 设置点的绘制半径（像素）
    void setPointRadius(int radius) { point_radius_ = radius; }

    /// 获取当前绘制半径
    int getPointRadius() const { return point_radius_; }

    /// 获取相机内参
    const CameraParams& getCameraParams() const { return camera_params_; }

    /// 获取外参
    const ExtrinsicParams& getExtrinsicParams() const { return extrinsic_params_; }

    /// 根据 Tcl 和 Til 计算相机到 IMU 的变换矩阵 Tic
    static Eigen::Matrix4d calculateTic(const Eigen::Matrix4d& Tcl, const Eigen::Matrix4d& Til);

private:
    /// 将里程计位姿转换为 4x4 齐次变换矩阵
    Eigen::Matrix4d odomPoseToMatrix(const OdomPose& odom) const;

    /// 将相机坐标系下的彩色点云投影到图像平面并绘制
    cv::Mat projectCloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in_cam) const;

    CameraParams camera_params_;        ///< 相机内参
    ExtrinsicParams extrinsic_params_;  ///< 外参
    std::unique_ptr<mini_vikit::PolynomialCamera> camera_model_; ///< 多项式相机模型

    int point_radius_  = 4;     ///< 点绘制半径（像素）
    bool initialized_  = false; ///< 是否已初始化
};
