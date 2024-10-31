#ifndef GROUND_STATION_HPP
#define GROUND_STATION_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "ns3_interfaces/msg/ns3_position_msg.hpp"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

namespace ground_station {

    class GroundStation {
    public:
        explicit GroundStation(rclcpp::Node::SharedPtr& _rclcpp_node,
            ns3::Ptr<ns3::Node> _ns3_node);
        GroundStation() = delete;

        std::string getName();

        void sendCommand(const std::string& command) const;

    protected:
        // The ROS node to use to create publishers and subscribers
        rclcpp::Node::SharedPtr rclcpp_node;

        // The NS-3 nodes
        ns3::Ptr<ns3::Node> ns3_node;

        // Location Subscriber
        rclcpp::Subscription<ns3_interfaces::msg::Ns3PositionMsg>::SharedPtr location_sub_;

        // Response Publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;

        // Command Publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

        // Subscription callback
        void onLocationReceived(const ns3_interfaces::msg::Ns3PositionMsg::SharedPtr msg);
    };

}  // namespace

#endif