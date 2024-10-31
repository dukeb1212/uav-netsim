#include "ground_station.hpp"

#include <string>
#include <exception>

namespace ground_station {

    GroundStation::GroundStation(rclcpp::Node::SharedPtr& _rclcpp_node, ns3::Ptr<ns3::Node> _ns3_node)
        : rclcpp_node(_rclcpp_node), ns3_node(_ns3_node)
    {
        // Create location subscriber
        location_sub_ = rclcpp_node->create_subscription<ns3_interfaces::msg::Ns3PositionMsg>(
            "drone_location",
            rclcpp::QoS(10), // QoS with a queue size of 10
            std::bind(&GroundStation::onLocationReceived, this, std::placeholders::_1));

        // Create a response publisher
        response_pub_ = rclcpp_node->create_publisher<std_msgs::msg::String>(
            "ground_station_response", 
            rclcpp::QoS(10));

        // Create command publisher
        command_pub_ = rclcpp_node->create_publisher<std_msgs::msg::String>(
            "drone_command", 
            rclcpp::QoS(10));
    }

    std::string GroundStation::getName()
    {
        return "ground_station";
    }

    void GroundStation::sendCommand(const std::string& command) const
    {
        // Create a message
        std_msgs::msg::String msg;
        msg.data = command;

        // Publish the command to the drone_command topic
        command_pub_->publish(msg);

        RCLCPP_INFO(rclcpp_node->get_logger(), "Sent command: %s", msg.data.c_str());
    }

    void GroundStation::onLocationReceived(const ns3_interfaces::msg::Ns3PositionMsg::SharedPtr msg)
    {
        // Extract the drone location details from the received message
        float x = msg->x;
        float y = msg->y;
        float z = msg->z;
        float distance = msg->distance;

        // Format the drone location as a string
        std::string drone_location = "x: " + std::to_string(x) + ", y: " + std::to_string(y) +
            ", z: " + std::to_string(z) + ", distance: " + std::to_string(distance);

        // Create a response message
        std::string response_message = "Location received: Drone location is " + drone_location;

        // Create a message object to publish
        std_msgs::msg::String response_msg;
        response_msg.data = response_message;

        // Publish the response
        response_pub_->publish(response_msg);
    }

}  // namespace drone

