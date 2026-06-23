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

#include <algorithm>
#include <cstring>
#include <string>

uint32_t GpsRawCrc32(const uint8_t * data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

GpsRawWire BuildGpsRawWire(const mavros_msgs::msg::GPSRAW & msg)
{
    GpsRawWire wire;
    std::memset(&wire, 0, sizeof(wire));

    wire.magic   = kGpsRawWireMagic;
    wire.version = kGpsRawWireVersion;
    wire.length  = static_cast<uint16_t>(sizeof(GpsRawWire));

    wire.stampSec     = msg.header.stamp.sec;
    wire.stampNanosec = msg.header.stamp.nanosec;

    wire.fixType           = msg.fix_type;
    wire.satellitesVisible = msg.satellites_visible;
    wire.dgpsNumChannels   = msg.dgps_numch;
    wire.reserved          = 0;

    wire.lat          = msg.lat;
    wire.lon          = msg.lon;
    wire.alt          = msg.alt;
    wire.altEllipsoid = msg.alt_ellipsoid;
    wire.hdgAcc       = msg.hdg_acc;

    wire.hAcc    = msg.h_acc;
    wire.vAcc    = msg.v_acc;
    wire.velAcc  = msg.vel_acc;
    wire.dgpsAge = msg.dgps_age;

    wire.eph       = msg.eph;
    wire.epv       = msg.epv;
    wire.vel       = msg.vel;
    wire.cog       = msg.cog;
    wire.yaw       = msg.yaw;
    wire.reserved2 = 0;

    // frame_id is null-padded; truncate to fit and keep a trailing null.
    const std::string & frameId = msg.header.frame_id;
    const size_t copyLen = std::min(frameId.size(), sizeof(wire.frameId) - 1);
    std::memcpy(wire.frameId, frameId.data(), copyLen);

    // CRC covers every byte except the trailing crc32 field itself.
    wire.crc32 = GpsRawCrc32(reinterpret_cast<const uint8_t *>(&wire),
                             sizeof(GpsRawWire) - sizeof(wire.crc32));
    return wire;
}

std::vector<uint8_t> SerializeGpsRaw(const mavros_msgs::msg::GPSRAW & msg)
{
    const GpsRawWire wire = BuildGpsRawWire(msg);
    std::vector<uint8_t> buffer(sizeof(GpsRawWire));
    std::memcpy(buffer.data(), &wire, sizeof(GpsRawWire));
    return buffer;
}

bool DeserializeGpsRaw(const uint8_t * data, size_t size, GpsRawWire & out)
{
    if (data == nullptr || size < sizeof(GpsRawWire)) {
        return false;
    }

    GpsRawWire wire;
    std::memcpy(&wire, data, sizeof(GpsRawWire));

    if (wire.magic != kGpsRawWireMagic ||
        wire.version != kGpsRawWireVersion ||
        wire.length != sizeof(GpsRawWire)) {
        return false;
    }

    const uint32_t expected = GpsRawCrc32(
        reinterpret_cast<const uint8_t *>(&wire),
        sizeof(GpsRawWire) - sizeof(wire.crc32));
    if (expected != wire.crc32) {
        return false;
    }

    out = wire;
    return true;
}

mavros_msgs::msg::GPSRAW GpsRawWireToMessage(const GpsRawWire & wire)
{
    mavros_msgs::msg::GPSRAW msg;

    msg.header.stamp.sec     = wire.stampSec;
    msg.header.stamp.nanosec = wire.stampNanosec;

    // frame_id is null-padded inside a fixed buffer; stop at the first null.
    size_t frameIdLen = 0;
    while (frameIdLen < sizeof(wire.frameId) && wire.frameId[frameIdLen] != '\0') {
        ++frameIdLen;
    }
    msg.header.frame_id.assign(wire.frameId, frameIdLen);

    msg.fix_type           = wire.fixType;
    msg.satellites_visible = wire.satellitesVisible;
    msg.dgps_numch         = wire.dgpsNumChannels;

    msg.lat           = wire.lat;
    msg.lon           = wire.lon;
    msg.alt           = wire.alt;
    msg.alt_ellipsoid = wire.altEllipsoid;
    msg.hdg_acc       = wire.hdgAcc;

    msg.h_acc    = wire.hAcc;
    msg.v_acc    = wire.vAcc;
    msg.vel_acc  = wire.velAcc;
    msg.dgps_age = wire.dgpsAge;

    msg.eph = wire.eph;
    msg.epv = wire.epv;
    msg.vel = wire.vel;
    msg.cog = wire.cog;
    msg.yaw = wire.yaw;

    return msg;
}

bool GpsRawMessageEqual(const mavros_msgs::msg::GPSRAW & a,
                        const mavros_msgs::msg::GPSRAW & b,
                        std::string * mismatchField)
{
    auto fail = [mismatchField](const char * name) {
        if (mismatchField != nullptr) {
            *mismatchField = name;
        }
        return false;
    };

    if (a.header.frame_id != b.header.frame_id)           return fail("header.frame_id");
    if (a.header.stamp.sec != b.header.stamp.sec)         return fail("header.stamp.sec");
    if (a.header.stamp.nanosec != b.header.stamp.nanosec) return fail("header.stamp.nanosec");
    if (a.fix_type != b.fix_type)                         return fail("fix_type");
    if (a.satellites_visible != b.satellites_visible)     return fail("satellites_visible");
    if (a.dgps_numch != b.dgps_numch)                     return fail("dgps_numch");
    if (a.lat != b.lat)                                   return fail("lat");
    if (a.lon != b.lon)                                   return fail("lon");
    if (a.alt != b.alt)                                   return fail("alt");
    if (a.alt_ellipsoid != b.alt_ellipsoid)               return fail("alt_ellipsoid");
    if (a.hdg_acc != b.hdg_acc)                           return fail("hdg_acc");
    if (a.h_acc != b.h_acc)                               return fail("h_acc");
    if (a.v_acc != b.v_acc)                               return fail("v_acc");
    if (a.vel_acc != b.vel_acc)                           return fail("vel_acc");
    if (a.dgps_age != b.dgps_age)                         return fail("dgps_age");
    if (a.eph != b.eph)                                   return fail("eph");
    if (a.epv != b.epv)                                   return fail("epv");
    if (a.vel != b.vel)                                   return fail("vel");
    if (a.cog != b.cog)                                   return fail("cog");
    if (a.yaw != b.yaw)                                   return fail("yaw");

    if (mismatchField != nullptr) {
        mismatchField->clear();
    }
    return true;
}
