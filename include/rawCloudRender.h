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
 * @file rawCloudRender.h
 * @brief 原始点云彩色渲染模块
 *
 * 使用标定文件（calib.yaml）将 RGB 图像颜色映射到 LiDAR 点云，
 * 生成彩色点云（cloud_render 话题）。
 */

#ifndef RGBCLOUD_H
#define RGBCLOUD_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "lidar_api.h"

/// 全局相机内参命名空间，供点云渲染时共享使用
namespace GlobalCameraParams {
    extern float g_fx;       ///< 相机水平焦距
    extern float g_fy;       ///< 相机垂直焦距
    extern float g_cx;       ///< 主点 u 坐标
    extern float g_cy;       ///< 主点 v 坐标
    extern float g_skew;     ///< 倾斜系数
    extern float g_k2;       ///< 畸变系数 k2
    extern float g_k3;       ///< 畸变系数 k3
    extern float g_k4;       ///< 畸变系数 k4
    extern float g_k5;       ///< 畸变系数 k5
    extern float g_k6;       ///< 畸变系数 k6
    extern float g_k7;       ///< 畸变系数 k7
    extern Eigen::Matrix4f g_T_camera_lidar; ///< 相机到 LiDAR 的外参变换矩阵
}

/**
 * @brief 原始点云彩色渲染类
 *
 * 负责将 NV12 格式图像转换为 RGB，并将颜色附加到点云上。
 */
class rawCloudRender {
public:

    /// 从 YAML 标定文件初始化相机参数和外参
    bool init(const std::string& yamlFilePath);

    /// 将 NV12 格式缓冲区转换为 RGB 图像（浮点二维数组）
    void nv12buffer_2_rgb(buffer_List_t &image, std::vector<std::vector<float>>& rgb_image);

    /// 将 RGB 颜色渲染到点云，输出扁平化的彩色点云数据
    void render(std::vector<std::vector<float>>& rgb_image, capture_Image_List_t* pcdStream, int pcdIdx, std::vector<float>& rgbCloud_flat);

    /// 打印当前相机标定参数（调试用）
    void print_camera_calib();

    /// 获取图像宽度
    int getImageWidth() const { return image_width_; }

    /// 获取图像高度
    int getImageHeight() const { return image_height_; }

private:
    std::string model_type_;
    std::string camera_name_;
    int image_width_;
    int image_height_;
    int frame_size_;
    bool opencv_available_;

    /// 相机到 LiDAR 的 4x4 外参变换矩阵
    Eigen::Matrix4f T_camera_lidar_;

    float k2_, k3_, k4_, k5_, k6_, k7_; ///< 多项式畸变系数
    float p1_, p2_;                       ///< 切向畸变系数
    float A11_fx_;                        ///< 内参 fx
    float A12_skew_;                      ///< 内参倾斜系数
    float A22_fy_;                        ///< 内参 fy
    float u0_cx_;                         ///< 主点 cx
    float v0_cy_;                         ///< 主点 cy
    bool isFast_;                         ///< 是否使用快速渲染模式
    int numDiff_;                         ///< 差分计算次数
    float maxIncidentAngle_;              ///< 最大入射角（用于过滤背面点）
};

#endif  // RGBCLOUD_H
