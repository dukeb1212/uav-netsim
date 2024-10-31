#ifndef DRONE_HPP
#define DRONE_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "airsim_interfaces/msg/gps_yaw.hpp"
#include "ns3_interfaces/msg/ns3_position_msg.hpp"
#include "utils.hpp"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

namespace drone {

    class Drone {
    public:
        explicit Drone(rclcpp::Node::SharedPtr& _rclcpp_node,
            ns3::Ptr<ns3::Node> _ns3_node);
        Drone() = delete;

        std::string getName();

        void publishLocation();

        // Ping check
        /*void onPingReplyReceived(ns3::Ptr<ns3::Packet> packet);
        void checkConnection();
        void increasePingFailures();*/

    protected:
        // Ping check threshhold
        /*int ping_failures_;
        const int failure_threshold_ = 5;*/

        // Origin Geo Point
        PointLLA origin_;

        // Distance from ground control (origin Geo Point)
        double distance_;

        // The ROS node to use to create publishers and subscribers
        rclcpp::Node::SharedPtr rclcpp_node;

        // The NS-3 nodes
        ns3::Ptr<ns3::Node> ns3_node;

        // Location Publisher
        rclcpp::Publisher<ns3_interfaces::msg::Ns3PositionMsg>::SharedPtr location_pub_;

        // Command Subscriber
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

        // AirSim Position Subscriber
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

        // Origin Geo Point Subscriber
        rclcpp::Subscription<airsim_interfaces::msg::GPSYaw>::SharedPtr origin_geo_point_sub_;

        // Timer for periodic publisher
        rclcpp::TimerBase::SharedPtr timer_;

        // Execute command
        void onCommandReceived(const std_msgs::msg::String::SharedPtr msg);

        // Update NS3 Location
        void onGPSReceived(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

        // Update Origin Geo point
        void onOriginGeoPointReceived(const airsim_interfaces::msg::GPSYaw::SharedPtr origin_msg);

        // NS3 Mobility Model
        ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility_;

        // Takeoff
        void takeoff();

        // Landing
        void landing();

        
    };

}  // namespace

#endif