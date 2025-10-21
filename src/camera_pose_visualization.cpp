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

#include "camera_pose_visualization.h"

const Eigen::Vector3d camera_pose_visualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d camera_pose_visualization::imrt = Eigen::Vector3d( 1.0, -0.5, 1.0);
const Eigen::Vector3d camera_pose_visualization::imlb = Eigen::Vector3d(-1.0,  0.5, 1.0);
const Eigen::Vector3d camera_pose_visualization::imrb = Eigen::Vector3d( 1.0,  0.5, 1.0);
const Eigen::Vector3d camera_pose_visualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d camera_pose_visualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d camera_pose_visualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d camera_pose_visualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);

#ifdef ROS2
using GeometryPoint = geometry_msgs::msg::Point;
#else
using GeometryPoint = geometry_msgs::Point;
#endif

void Eigen2Point(const Eigen::Vector3d& v, GeometryPoint& p) {
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

camera_pose_visualization::camera_pose_visualization(float r, float g, float b, float a)
    : m_marker_ns("camera_pose_visualization"), m_scale(0.3), m_line_width(0.03) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void camera_pose_visualization::setImageBoundaryColor(float r, float g, float b, float a) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
}

void camera_pose_visualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void camera_pose_visualization::setScale(double s) {
    m_scale = s;
}
void camera_pose_visualization::setLineWidth(double width) {
    m_line_width = width;
}
void camera_pose_visualization::add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
    Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
#ifdef ROS2
    marker.type = Marker::LINE_LIST;
    marker.action = Marker::ADD;
#else
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
#endif
    marker.scale.x = 0.005;

    marker.color.g = 1.0f;
    marker.color.a = 1.0;

    GeometryPoint point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}

void camera_pose_visualization::add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
    Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
#ifdef ROS2
    marker.type = Marker::LINE_LIST;
    marker.action = Marker::ADD;
#else
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
#endif
    marker.scale.x = 0.04;
    //marker.scale.x = 0.3;

    marker.color.r = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    GeometryPoint point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}


void camera_pose_visualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
#ifdef ROS2
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
#else
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
#endif
    marker.scale.x = m_line_width;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;


    GeometryPoint pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    Eigen2Point(q * (m_scale * imlt) + p, pt_lt);
    Eigen2Point(q * (m_scale * imlb) + p, pt_lb);
    Eigen2Point(q * (m_scale * imrt) + p, pt_rt);
    Eigen2Point(q * (m_scale * imrb) + p, pt_rb);
    Eigen2Point(q * (m_scale * lt0 ) + p, pt_lt0);
    Eigen2Point(q * (m_scale * lt1 ) + p, pt_lt1);
    Eigen2Point(q * (m_scale * lt2 ) + p, pt_lt2);
    Eigen2Point(q * (m_scale * oc  ) + p, pt_oc);

    // image boundaries
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // top-left indicator
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // optical center connector
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);


    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    m_markers.push_back(marker);
}

void camera_pose_visualization::reset() {
    m_markers.clear();
}

void camera_pose_visualization::publish_by( Publisher& pub, const Header& header ) {
    MarkerArray markerArray_msg;

    for (auto& marker : m_markers) {
        marker.header = header;
        markerArray_msg.markers.push_back(marker);
    }

    pub.publish(markerArray_msg);
}