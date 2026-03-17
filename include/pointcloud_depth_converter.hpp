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
 * @file pointcloud_depth_converter.hpp
 * @brief 点云转深度图核心算法模块
 *
 * 将稀疏三维点云投影到相机坐标系并生成密集深度图（depth completion），
 * 同时输出带颜色标注的点云（PointXYZRGB）。
 * 支持多项式畸变相机模型和可配置采样率。
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

/**
 * @brief 点云到深度图转换器
 *
 * 根据相机内参和外参，将输入点云投影到像素平面，
 * 生成密集深度图并进行后处理（填充、平滑）。
 */
class PointCloudToDepthConverter
{
public:
    /// 相机参数结构体（内参 + 外参 + 配置参数）
    struct CameraParams
    {
        int image_width;                       ///< 图像宽度（像素）
        int image_height;                      ///< 图像高度（像素）
        double A11, A12, A22;                  ///< 相机内参矩阵分量（fx, skew, fy）
        double u0, v0;                         ///< 主点坐标（cx, cy）
        double k2, k3, k4, k5, k6, k7;        ///< 多项式畸变系数
        double scale;                          ///< 深度值缩放系数
        int point_sampling_rate;               ///< 点云采样率（降低计算量）
        Eigen::Matrix4d Tcl;                   ///< 相机到 LiDAR 的外参变换矩阵
    };

    /// 处理结果结构体
    struct ProcessResult
    {
        cv::Mat depth_image;                            ///< 生成的深度图（32FC1）
        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud; ///< 带颜色的点云
        bool success;                                   ///< 是否处理成功
        std::string error_message;                      ///< 错误信息（失败时有效）
    };

    /// 构造函数，传入相机参数并初始化内部参数和畸变查找表
    explicit PointCloudToDepthConverter(const CameraParams &params);

    /**
     * @brief 处理点云和 RGB 图像，生成深度图和彩色点云
     * @param cloud 输入稀疏点云（相机坐标系）
     * @param image 输入 RGB 图像
     * @return ProcessResult 包含深度图和彩色点云的结果
     */
    ProcessResult processCloudAndImage(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                       const cv::Mat &image);

    /// 自定义图像缩放（保持深度精度）
    cv::Mat customResize(const cv::Mat& src, const cv::Size& size);

    /// 获取当前相机参数
    const CameraParams &getCameraParams() const { return params_; }

    /// 动态更新相机参数并重新初始化内部查找表
    void updateCameraParams(const CameraParams &params);

private:
    CameraParams params_;      ///< 相机参数

    Eigen::Matrix3d K_;        ///< 内参矩阵
    Eigen::Matrix3d Kl_;       ///< LiDAR 坐标系内参矩阵
    Eigen::Matrix4d K_4x4_;    ///< 齐次内参矩阵
    Eigen::Matrix4d Kcl_;      ///< 组合投影矩阵

    cv::Mat map_x_, map_y_;         ///< 畸变正向映射表
    cv::Mat inv_map_x_, inv_map_y_; ///< 畸变逆向映射表

    int scaled_width_, scaled_height_; ///< 缩放后的图像尺寸

    /// 初始化内部矩阵和缩放参数
    void initializeInternalParams();

    /// 创建畸变矫正查找表（map_x_, map_y_）
    void createDistortionMaps();

    /// 将点云投影到深度图像平面
    cv::Mat projectCloudToDepth(const pcl::PointCloud<pcl::PointXYZ> &cloud_in_cam);

    /// 对稀疏深度图进行后处理（膨胀/填充）
    cv::Mat postProcessDepthImage(const cv::Mat &depth_img);

    /// 根据深度图和 RGB 图生成带颜色点云
    pcl::PointCloud<pcl::PointXYZRGB> generateColoredCloud(const cv::Mat &depth_img,
                                                           const cv::Mat &color_img);

    /// 校验输入点云和图像的有效性
    std::pair<bool, std::string> validateInputs(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                const cv::Mat &image);
};
