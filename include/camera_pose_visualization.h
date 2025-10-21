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
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#else
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif
#include <Eigen/Dense>
#include <Eigen/Geometry>

class camera_pose_visualization {
  public:
    std::string m_marker_ns;

    camera_pose_visualization(float r, float g, float b, float a);

    void setImageBoundaryColor(float r, float g, float b, float a = 1.0);
    void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);
    void setScale(double s);
    void setLineWidth(double width);

    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    void reset();

    #ifdef ROS2
    using ColorRGBA = std_msgs::msg::ColorRGBA;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Header = std_msgs::msg::Header;
    using Publisher = rclcpp::Publisher<MarkerArray>;
    #else
    using ColorRGBA = std_msgs::ColorRGBA;
    using Marker = visualization_msgs::Marker;
    using MarkerArray = visualization_msgs::MarkerArray;
    using Header = std_msgs::Header;
    using Publisher = ros::Publisher;
    #endif

    void publish_by(Publisher& pub, const Header& header);
    void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
  private:
    std::vector<Marker> m_markers;
    ColorRGBA m_image_boundary_color;
    ColorRGBA m_optical_center_connector_color;
    double m_scale;
    double m_line_width;

    static const Eigen::Vector3d imlt;
    static const Eigen::Vector3d imlb;
    static const Eigen::Vector3d imrt;
    static const Eigen::Vector3d imrb;
    static const Eigen::Vector3d oc  ;
    static const Eigen::Vector3d lt0 ;
    static const Eigen::Vector3d lt1 ;
    static const Eigen::Vector3d lt2 ;
};
