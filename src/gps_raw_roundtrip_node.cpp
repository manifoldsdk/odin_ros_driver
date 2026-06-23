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

// Offline round-trip verification tool.
//
// It subscribes to a source GPSRAW topic (e.g. while replaying a rosbag),
// and for every message it performs the exact transmit-side processing:
//   message -> SerializeGpsRaw (the bytes that would be sent to the device)
//           -> DeserializeGpsRaw -> GpsRawWireToMessage (rebuilt message).
// Both the original and the rebuilt message are written into a single output
// rosbag (two topics), and a field-by-field equality check is logged per
// frame. This lets you compare the "input source" and "output source" ROS
// messages entirely offline, without a device or any external party.

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "gps_raw_node.hpp"

class GpsRawRoundtripNode : public rclcpp::Node
{
public:
    GpsRawRoundtripNode()
        : Node("gps_raw_roundtrip")
    {
        inTopic_     = this->declare_parameter<std::string>("in_topic", "/mavros/gpsstatus/gps1/raw");
        bagUri_      = this->declare_parameter<std::string>("bag_uri", "gps_roundtrip_bag");
        inTopicName_ = this->declare_parameter<std::string>("in_topic_name", "/gps_in");
        outTopicName_ = this->declare_parameter<std::string>("out_topic_name", "/gps_out");

        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open(bagUri_);

        subscription_ = this->create_subscription<mavros_msgs::msg::GPSRAW>(
            inTopic_, rclcpp::SensorDataQoS(),
            std::bind(&GpsRawRoundtripNode::OnGpsRaw, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Round-trip verification started.\n"
                    "  in_topic       : %s\n"
                    "  bag_uri        : %s\n"
                    "  original topic : %s\n"
                    "  rebuilt topic  : %s",
                    inTopic_.c_str(), bagUri_.c_str(),
                    inTopicName_.c_str(), outTopicName_.c_str());
    }

    ~GpsRawRoundtripNode() override
    {
        RCLCPP_INFO(this->get_logger(),
                    "Round-trip summary: total=%lu pass=%lu fail=%lu",
                    static_cast<unsigned long>(total_),
                    static_cast<unsigned long>(pass_),
                    static_cast<unsigned long>(fail_));
    }

private:
    void OnGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
    {
        ++total_;

        // Transmit-side bytes, then decode them back exactly as the receiver would.
        const std::vector<uint8_t> blob = SerializeGpsRaw(*msg);

        GpsRawWire wire;
        if (!DeserializeGpsRaw(blob.data(), blob.size(), wire)) {
            ++fail_;
            RCLCPP_ERROR(this->get_logger(),
                         "frame #%lu: deserialize/CRC failed (%zu bytes)",
                         static_cast<unsigned long>(total_), blob.size());
            return;
        }

        const mavros_msgs::msg::GPSRAW rebuilt = GpsRawWireToMessage(wire);

        std::string mismatchField;
        const bool equal = GpsRawMessageEqual(*msg, rebuilt, &mismatchField);
        if (equal) {
            ++pass_;
        } else {
            ++fail_;
            RCLCPP_WARN(this->get_logger(),
                        "frame #%lu: MISMATCH at field '%s'",
                        static_cast<unsigned long>(total_), mismatchField.c_str());
        }

        // Use the message stamp so both topics line up on the bag timeline.
        const rclcpp::Time stamp(msg->header.stamp);
        writer_->write(*msg, inTopicName_, stamp);
        writer_->write(rebuilt, outTopicName_, stamp);

        RCLCPP_INFO(this->get_logger(),
                    "frame #%lu: %s",
                    static_cast<unsigned long>(total_),
                    equal ? "OK" : "FAIL");
    }

    rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr subscription_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    std::string inTopic_;
    std::string bagUri_;
    std::string inTopicName_;
    std::string outTopicName_;

    uint64_t total_ = 0;
    uint64_t pass_  = 0;
    uint64_t fail_  = 0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsRawRoundtripNode>());
    rclcpp::shutdown();
    return 0;
}
