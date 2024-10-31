#include "drone.hpp"

#include <string>
#include <exception>
#include <chrono>

namespace drone {

    Drone::Drone(rclcpp::Node::SharedPtr& _rclcpp_node, ns3::Ptr<ns3::Node> _ns3_node)
        : rclcpp_node(_rclcpp_node), ns3_node(_ns3_node)
    {
        // Create location publisher
        location_pub_ = rclcpp_node->create_publisher<ns3_interfaces::msg::Ns3PositionMsg>("drone_location", 10);

        // Create the timer to publish location every second
        timer_ = rclcpp_node->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Drone::publishLocation, this) // Bind the publishLocation method to the timer
        );

        // Create command subscriber
        command_sub_ = rclcpp_node->create_subscription<std_msgs::msg::String>(
            "drone_command",
            rclcpp::QoS(10), // QoS with a queue size of 10
            std::bind(&Drone::onCommandReceived, this, std::placeholders::_1));

        // Create AirSim gps subscriber
        gps_sub_ = rclcpp_node->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/airsim_node/uav1/gps/gps",
            rclcpp::QoS(5), // QoS with a queue size of 10
            std::bind(&Drone::onGPSReceived, this, std::placeholders::_1));

        // Create AirSim origin geo point subscriber
        origin_geo_point_sub_ = rclcpp_node->create_subscription<airsim_interfaces::msg::GPSYaw>(
            "/airsim_node/origin_geo_point",
            rclcpp::QoS(5), // QoS with a queue size of 10
            std::bind(&Drone::onOriginGeoPointReceived, this, std::placeholders::_1));

        // Initiate mobility model ns3
        mobility_ = ns3_node->GetObject<ns3::ConstantPositionMobilityModel>();

        origin_ = PointLLA(0.0, 0.0, 0.0);

        distance_ = 0.0;
    }

    std::string Drone::getName()
    {
        return "drone";
    }

    void Drone::publishLocation()
    {
        ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility_model =
            ns3_node->GetObject<ns3::ConstantPositionMobilityModel>();

        ns3::Vector location = mobility_model->GetPosition();
        ns3_interfaces::msg::Ns3PositionMsg msg;
        msg.x = float(location.x);
        msg.y = float(location.y);
        msg.z = float(location.z);
        msg.distance = float(distance_);

        rclcpp::Time current_time = rclcpp_node->get_clock()->now();
        msg.timestamp.sec = current_time.seconds(); // Getting seconds part
        msg.timestamp.nanosec = current_time.nanoseconds() % 1000000000;

        location_pub_->publish(msg);

        // Logging
        // RCLCPP_INFO(rclcpp_node->get_logger(), "Published: '%s'", msg.data.c_str());
    }

    void Drone::onCommandReceived(const std_msgs::msg::String::SharedPtr msg)
    {
        // RCLCPP_INFO(rclcpp_node->get_logger(), "Received command: %s", msg->data.c_str());

        if (msg->data == "takeoff") {
            // Takeoff
            takeoff();
        }
        else if (msg->data == "landing") {
            // Landing
            landing();
        }
    }

    void Drone::onGPSReceived(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
    {
        PointLLA current_gps_lla = { gps_msg->latitude, gps_msg->longitude, gps_msg->altitude };
        PointECEF origin_ecef = llaToEcef(origin_);
        PointECEF current_gps_ecef = llaToEcef(current_gps_lla);

        PointENU current_gps_enu = ecefToEnu(current_gps_ecef, origin_ecef, origin_);
        
        auto ns3_position = ns3::Vector(current_gps_enu.east, current_gps_enu.north, current_gps_enu.up);
        mobility_->SetPosition(ns3_position);
        distance_ = calculateDistance(origin_, current_gps_lla);
    }

    void Drone::onOriginGeoPointReceived(const airsim_interfaces::msg::GPSYaw::SharedPtr origin_msg)
    {
        origin_.latitude = origin_msg->latitude;
        origin_.longitude = origin_msg->longitude;
        origin_.altitude = origin_msg->altitude;
    }

    void Drone::takeoff()
    {
        auto vector = mobility_->GetPosition();
        if (vector.z > 0) 
        {
            RCLCPP_INFO(rclcpp_node->get_logger(), "Cannot takeoff! Already in the air");
        }
        else 
        {
            // Takeoff 10m
            vector.z = 10.0;
            mobility_->SetPosition(vector);
        }
    }

    void Drone::landing()
    {
        auto vector = mobility_->GetPosition();
        if (vector.z > 0) 
        {
            vector.z = 0.0;
            mobility_->SetPosition(vector);
        }
        else 
        {
            RCLCPP_INFO(rclcpp_node->get_logger(), "Already on the ground!");
        }
    }
    //void Drone::onPingReplyReceived(ns3::Ptr<ns3::Packet> packet)
    //{
    //    ping_failures_ = 0; // Reset failures on successful ping
    //    RCLCPP_INFO(rclcpp_node->get_logger(), "Ping reply received from ground station");
    //}
    //void Drone::checkConnection()
    //{
    //    if (ping_failures_ >= failure_threshold_) {
    //        RCLCPP_WARN(rclcpp_node->get_logger(), "Connection lost with ground station.");
    //        // Implement additional logic here if needed
    //    }
    //}

    //void Drone::increasePingFailures() {
    //    ping_failures_++; // Increment the ping failure count
    //    RCLCPP_WARN(rclcpp_node->get_logger(), "Ping failure count increased: %d", ping_failures_);
    //}
}  // namespace drone

