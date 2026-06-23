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

#ifndef ODIN_ROS_DRIVER_GPS_RAW_NODE_HPP
#define ODIN_ROS_DRIVER_GPS_RAW_NODE_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "mavros_msgs/msg/gpsraw.hpp"

// On-wire binary layout for a single GPSRAW sample. This is a fixed-size,
// tightly packed POD record (no std::string, fixed-width integers only) so it
// can be memcpy'd into a byte buffer and sent through lidar_send_user_data.
// All multi-byte fields are stored in little-endian (native x86/ARM layout).
// Raw integer units from mavros_msgs/msg/GPSRAW are kept as-is; the receiver
// re-scales them (lat/lon: 1e-7 deg, alt: mm, vel: cm/s, cog/yaw: cdeg, ...).
#pragma pack(push, 1)
struct GpsRawWire
{
    uint32_t magic;             // kGpsRawWireMagic, frame start marker
    uint16_t version;           // kGpsRawWireVersion, layout version
    uint16_t length;            // total byte size of this struct (== sizeof)

    int32_t  stampSec;          // header.stamp.sec
    uint32_t stampNanosec;      // header.stamp.nanosec

    uint8_t  fixType;           // GPS_FIX_TYPE enum value
    uint8_t  satellitesVisible; // satellites_visible
    uint8_t  dgpsNumChannels;   // dgps_numch
    uint8_t  reserved;          // explicit padding, kept zero

    int32_t  lat;               // lat          [degE7]
    int32_t  lon;               // lon          [degE7]
    int32_t  alt;               // alt          [mm] MSL
    int32_t  altEllipsoid;      // alt_ellipsoid[mm] above WGS84
    int32_t  hdgAcc;            // hdg_acc      [degE5]

    uint32_t hAcc;              // h_acc        [mm]
    uint32_t vAcc;              // v_acc        [mm]
    uint32_t velAcc;            // vel_acc      [mm]
    uint32_t dgpsAge;           // dgps_age     [ms]

    uint16_t eph;               // eph          (HDOP, UINT16_MAX = unknown)
    uint16_t epv;               // epv          (VDOP, UINT16_MAX = unknown)
    uint16_t vel;               // vel          [cm/s]
    uint16_t cog;               // cog          [cdeg]
    uint16_t yaw;               // yaw          [cdeg]
    uint16_t reserved2;         // explicit padding, kept zero

    char     frameId[16];       // header.frame_id, null-padded, truncated if longer

    uint32_t crc32;             // CRC-32 over all preceding bytes
};
#pragma pack(pop)

// Frame start marker, ASCII 'G''P''S''R'.
static constexpr uint32_t kGpsRawWireMagic   = 0x47505352u;
// Version 2 adds the fixed-size frameId field.
static constexpr uint16_t kGpsRawWireVersion = 2u;
// Capacity of the frameId field, including the trailing null terminator.
static constexpr size_t   kGpsRawFrameIdSize = 16u;

// Fill a GpsRawWire (including magic/version/length and crc32) from a message.
GpsRawWire BuildGpsRawWire(const mavros_msgs::msg::GPSRAW & msg);

// Serialize a message into a contiguous byte buffer ready for transmission.
std::vector<uint8_t> SerializeGpsRaw(const mavros_msgs::msg::GPSRAW & msg);

// Validate magic/version/length/crc and decode bytes back into a GpsRawWire.
// Returns true on success. Provided as a reference for the receiver side.
bool DeserializeGpsRaw(const uint8_t * data, size_t size, GpsRawWire & out);

// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320) over a byte range.
uint32_t GpsRawCrc32(const uint8_t * data, size_t size);

// Rebuild a GPSRAW message from a decoded wire record. header.frame_id is
// restored from the fixed-size frameId field (truncated to its capacity).
mavros_msgs::msg::GPSRAW GpsRawWireToMessage(const GpsRawWire & wire);

// Field-by-field equality of two GPSRAW messages, including header.frame_id.
// On mismatch, the name of the first differing field is written to
// mismatchField (if non-null).
bool GpsRawMessageEqual(const mavros_msgs::msg::GPSRAW & a,
                        const mavros_msgs::msg::GPSRAW & b,
                        std::string * mismatchField = nullptr);

// Parsed GPSRAW data: every raw integer field of mavros_msgs/msg/GPSRAW is
// converted into a physically meaningful value (SI units / degrees / strings).
// Fields whose source value equals the "unknown" sentinel are flagged via the
// matching *Valid boolean instead of carrying the raw sentinel number.
struct ParsedGpsRaw
{
    // Timestamp (seconds) and frame id copied from std_msgs/Header.
    double      stampSeconds = 0.0;
    std::string frameId;

    // GPS fix type: numeric code plus a human readable description.
    uint8_t     fixType = 0;
    std::string fixTypeName;

    // Geodetic position. lat/lon in degrees, altitudes in meters.
    double latDeg          = 0.0;   // from lat [degE7]
    double lonDeg          = 0.0;   // from lon [degE7]
    double altMslMeters    = 0.0;   // from alt [mm], mean sea level
    double altEllipsoidMeters = 0.0;// from alt_ellipsoid [mm], above WGS84

    // Dilution of precision (unitless). Valid flags guard the UINT16_MAX sentinel.
    double hdop      = 0.0;         // from eph
    bool   hdopValid = false;
    double vdop      = 0.0;         // from epv
    bool   vdopValid = false;

    // Ground speed and course over ground.
    double groundSpeedMps   = 0.0;  // from vel [cm/s]
    bool   groundSpeedValid = false;
    double courseDeg        = 0.0;  // from cog [cdeg]
    bool   courseValid      = false;

    // Heading (yaw) in earth frame from north.
    double yawDeg      = 0.0;       // from yaw [cdeg]
    bool   yawValid    = false;

    // Accuracy estimates (meters / degrees).
    double horizontalAccMeters = 0.0;// from h_acc [mm]
    double verticalAccMeters   = 0.0;// from v_acc [mm]
    double speedAccMeters      = 0.0;// from vel_acc [mm]
    double headingAccDeg       = 0.0;// from hdg_acc [degE5]

    // Satellite / DGPS statistics.
    uint8_t  satellitesVisible = 0; // from satellites_visible
    uint8_t  dgpsNumChannels   = 0; // from dgps_numch
    double   dgpsAgeSeconds    = 0.0;// from dgps_age [ms]
};

class GpsRawNode : public rclcpp::Node
{
public:
    GpsRawNode();

    // Convert a raw GPSRAW message into the human friendly ParsedGpsRaw struct.
    static ParsedGpsRaw Parse(const mavros_msgs::msg::GPSRAW & msg);

    // Map a GPS_FIX_TYPE enum value to its textual description.
    static std::string FixTypeToString(uint8_t fixType);

    // Access the most recently parsed sample.
    const ParsedGpsRaw & GetLatest() const { return latest_; }

private:
    void OnGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg);

    rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr subscription_;
    std::string  gpsTopic_;
    ParsedGpsRaw latest_;
};

#endif  // ODIN_ROS_DRIVER_GPS_RAW_NODE_HPP
