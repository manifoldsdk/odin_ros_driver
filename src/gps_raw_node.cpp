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

#include "gps_raw_node.hpp"

#include <cstdint>
#include <functional>
#include <limits>

GpsRawNode::GpsRawNode()
    : Node("gps_raw_node")
{
    gpsTopic_ = this->declare_parameter<std::string>(
        "gps_topic", "/mavros/gpsstatus/gps1/raw");

    subscription_ = this->create_subscription<mavros_msgs::msg::GPSRAW>(
        gpsTopic_, rclcpp::SensorDataQoS(),
        std::bind(&GpsRawNode::OnGpsRaw, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "gps_raw_node started, subscribing GPSRAW on '%s'",
                gpsTopic_.c_str());
}

std::string GpsRawNode::FixTypeToString(uint8_t fixType)
{
    using Msg = mavros_msgs::msg::GPSRAW;
    switch (fixType) {
        case Msg::GPS_FIX_TYPE_NO_GPS:    return "NO_GPS";
        case Msg::GPS_FIX_TYPE_NO_FIX:    return "NO_FIX";
        case Msg::GPS_FIX_TYPE_2D_FIX:    return "2D_FIX";
        case Msg::GPS_FIX_TYPE_3D_FIX:    return "3D_FIX";
        case Msg::GPS_FIX_TYPE_DGPS:      return "DGPS";
        case Msg::GPS_FIX_TYPE_RTK_FLOAT: return "RTK_FLOAT";
        case Msg::GPS_FIX_TYPE_RTK_FIXED: return "RTK_FIXED";
        case Msg::GPS_FIX_TYPE_STATIC:    return "STATIC";
        case Msg::GPS_FIX_TYPE_PPP:       return "PPP";
        default:                          return "UNKNOWN";
    }
}

ParsedGpsRaw GpsRawNode::Parse(const mavros_msgs::msg::GPSRAW & msg)
{
    constexpr uint16_t kUint16Unknown = std::numeric_limits<uint16_t>::max();

    ParsedGpsRaw parsed;

    // Header.
    parsed.stampSeconds =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    parsed.frameId = msg.header.frame_id;

    // Fix type.
    parsed.fixType     = msg.fix_type;
    parsed.fixTypeName = FixTypeToString(msg.fix_type);

    // Position. lat/lon scaled by 1e-7, altitudes from mm to m.
    parsed.latDeg             = static_cast<double>(msg.lat) * 1e-7;
    parsed.lonDeg             = static_cast<double>(msg.lon) * 1e-7;
    parsed.altMslMeters       = static_cast<double>(msg.alt) * 1e-3;
    parsed.altEllipsoidMeters = static_cast<double>(msg.alt_ellipsoid) * 1e-3;

    // Dilution of precision (UINT16_MAX means unknown).
    parsed.hdopValid = (msg.eph != kUint16Unknown);
    parsed.hdop      = parsed.hdopValid ? static_cast<double>(msg.eph) : 0.0;
    parsed.vdopValid = (msg.epv != kUint16Unknown);
    parsed.vdop      = parsed.vdopValid ? static_cast<double>(msg.epv) : 0.0;

    // Ground speed (cm/s -> m/s) and course (cdeg -> deg).
    parsed.groundSpeedValid = (msg.vel != kUint16Unknown);
    parsed.groundSpeedMps   =
        parsed.groundSpeedValid ? static_cast<double>(msg.vel) * 0.01 : 0.0;
    parsed.courseValid = (msg.cog != kUint16Unknown);
    parsed.courseDeg   =
        parsed.courseValid ? static_cast<double>(msg.cog) * 0.01 : 0.0;

    // Yaw (cdeg -> deg). 0 is also used to mean "not available" by MAVLink.
    parsed.yawValid = (msg.yaw != 0 && msg.yaw != kUint16Unknown);
    parsed.yawDeg   = parsed.yawValid ? static_cast<double>(msg.yaw) * 0.01 : 0.0;

    // Accuracy estimates (mm -> m, degE5 -> deg).
    parsed.horizontalAccMeters = static_cast<double>(msg.h_acc) * 1e-3;
    parsed.verticalAccMeters   = static_cast<double>(msg.v_acc) * 1e-3;
    parsed.speedAccMeters      = static_cast<double>(msg.vel_acc) * 1e-3;
    parsed.headingAccDeg       = static_cast<double>(msg.hdg_acc) * 1e-5;

    // Satellite / DGPS statistics (ms -> s).
    parsed.satellitesVisible = msg.satellites_visible;
    parsed.dgpsNumChannels   = msg.dgps_numch;
    parsed.dgpsAgeSeconds    = static_cast<double>(msg.dgps_age) * 1e-3;

    return parsed;
}

void GpsRawNode::OnGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
{
    latest_ = Parse(*msg);

    RCLCPP_INFO(this->get_logger(),
                "GPSRAW [%s] sats=%u lat=%.7f lon=%.7f altMSL=%.3fm "
                "vel=%.2fm/s cog=%.2fdeg hAcc=%.3fm vAcc=%.3fm",
                latest_.fixTypeName.c_str(),
                latest_.satellitesVisible,
                latest_.latDeg,
                latest_.lonDeg,
                latest_.altMslMeters,
                latest_.groundSpeedMps,
                latest_.courseDeg,
                latest_.horizontalAccMeters,
                latest_.verticalAccMeters);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsRawNode>());
    rclcpp::shutdown();
    return 0;
}
