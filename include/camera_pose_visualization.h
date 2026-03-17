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
 * @file camera_pose_visualization.h
 * @brief 相机位姿可视化模块（ROS2）
 *
 * 使用 RViz MarkerArray 在三维空间中绘制相机位姿轨迹，
 * 包括相机视锥体边框、光心连接线和回环边。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @brief 相机位姿可视化类
 *
 * 将一系列相机位姿（平移 + 旋转）转化为 RViz Marker，
 * 支持设置颜色、尺寸和线宽，并发布到 ROS2 话题。
 */
class camera_pose_visualization {
  public:
    std::string m_marker_ns; ///< Marker 命名空间，用于 RViz 分组显示

    /// 构造函数，设置相机边框颜色（r, g, b, a）
    camera_pose_visualization(float r, float g, float b, float a);

    /// 设置图像边框颜色
    void setImageBoundaryColor(float r, float g, float b, float a = 1.0);

    /// 设置光心连接线颜色
    void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);

    /// 设置相机视锥体缩放比例
    void setScale(double s);

    /// 设置连接线宽度
    void setLineWidth(double width);

    /// 添加一个相机位姿（平移向量 + 旋转四元数）
    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);

    /// 清除所有已添加的 Marker
    void reset();

    using ColorRGBA  = std_msgs::msg::ColorRGBA;
    using Marker      = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Header      = std_msgs::msg::Header;
    using Publisher   = rclcpp::Publisher<MarkerArray>;

    /// 通过指定 Publisher 发布所有 Marker
    void publish_by(Publisher& pub, const Header& header);

    /// 添加普通位姿连接边（用于轨迹显示）
    void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

    /// 添加回环边（用于 SLAM 回环可视化）
    void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

  private:
    std::vector<Marker> m_markers;                    ///< 待发布的 Marker 列表
    ColorRGBA m_image_boundary_color;                 ///< 图像边框颜色
    ColorRGBA m_optical_center_connector_color;       ///< 光心连接线颜色
    double m_scale;                                   ///< 视锥体缩放比例
    double m_line_width;                              ///< 线宽

    /// 相机视锥体四个角点和光心的归一化坐标
    static const Eigen::Vector3d imlt; ///< 图像左上角
    static const Eigen::Vector3d imlb; ///< 图像左下角
    static const Eigen::Vector3d imrt; ///< 图像右上角
    static const Eigen::Vector3d imrb; ///< 图像右下角
    static const Eigen::Vector3d oc;   ///< 光心
    static const Eigen::Vector3d lt0;  ///< 三角形标记顶点 0
    static const Eigen::Vector3d lt1;  ///< 三角形标记顶点 1
    static const Eigen::Vector3d lt2;  ///< 三角形标记顶点 2
};
