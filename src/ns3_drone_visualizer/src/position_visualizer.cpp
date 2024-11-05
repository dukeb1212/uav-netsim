#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <ns3_interfaces/msg/ns3_position_msg.hpp>
#include "airsim_interfaces/msg/gps_yaw.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "utils.hpp"

class DroneVisualizer : public rclcpp::Node
{
public:
    DroneVisualizer() : Node("drone_visualizer"), point_count_(0)
    {
        // Subscription to the drone location topic
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/airsim_node/uav1/gps/gps",
            10,
            std::bind(&DroneVisualizer::onLocationReceived, this, std::placeholders::_1)
        );

        origin_subscription_ = this->create_subscription<airsim_interfaces::msg::GPSYaw>(
            "/airsim_node/origin_geo_point",
            10,
            std::bind(&DroneVisualizer::onOriginReceived, this, std::placeholders::_1)
        );

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("drone_markers", 10);

        // Initialize line strip marker
        line_strip_.header.frame_id = "uav1";  // Set the correct frame
        line_strip_.header.stamp = this->get_clock()->now();
        line_strip_.ns = "drone_path";
        line_strip_.id = 0;
        line_strip_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip_.action = visualization_msgs::msg::Marker::ADD;
        line_strip_.scale.x = 0.05;  // Increased line width
        line_strip_.color.r = 0.0f;  // Line color: Green
        line_strip_.color.g = 1.0f;
        line_strip_.color.b = 0.0f;
        line_strip_.color.a = 1.0;    // Opaque
        line_strip_.lifetime = rclcpp::Duration(0, 0);  // No lifetime
    }

private:
    void onLocationReceived(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
    {
        PointLLA current_gps_lla = { gps_msg->latitude, gps_msg->longitude, gps_msg->altitude };
        PointECEF origin_ecef = llaToEcef(origin_);
        PointECEF current_gps_ecef = llaToEcef(current_gps_lla);

        PointENU current_gps_enu = ecefToEnu(current_gps_ecef, origin_ecef, origin_);

        // Create a marker for the new point
        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = "uav1";  // Change to the appropriate frame
        point_marker.header.stamp = this->get_clock()->now();
        point_marker.ns = "drone_points";
        point_marker.id = point_count_; // Unique ID for each point
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.pose.position.x = current_gps_enu.east;
        point_marker.pose.position.y = current_gps_enu.north;
        point_marker.pose.position.z = current_gps_enu.up;

        // Increased scale for the point marker
        point_marker.scale.x = 0.2;  // Increased sphere diameter
        point_marker.scale.y = 0.2;
        point_marker.scale.z = 0.2;
        point_marker.color.r = 1.0f;  // Red color
        point_marker.color.g = 0.0f;
        point_marker.color.b = 0.0f;
        point_marker.color.a = 1.0;  // Opaque
        point_marker.lifetime = rclcpp::Duration(0, 0);  // No lifetime

        // Add the current position to the line strip
        geometry_msgs::msg::Point p;
        p.x = current_gps_enu.east;
        p.y = current_gps_enu.north;
        p.z = current_gps_enu.up;
        line_strip_.points.push_back(p);

        // Publish the point marker
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(point_marker);

        // Only publish the line strip if there are points
        if (!line_strip_.points.empty())
        {
            line_strip_.header.stamp = this->get_clock()->now(); // Update timestamp
            marker_array.markers.push_back(line_strip_); // Add line strip marker to array
        }

        // Publish the markers
        marker_pub_->publish(marker_array);

        // Increment point count
        point_count_++;
    }

    void onOriginReceived(const airsim_interfaces::msg::GPSYaw::SharedPtr msg) {
        origin_.latitude = msg->latitude;
        origin_.longitude = msg->longitude;
        origin_.altitude = msg->altitude;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<airsim_interfaces::msg::GPSYaw>::SharedPtr origin_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int point_count_;  // Counter for the number of points
    PointLLA origin_;

    visualization_msgs::msg::Marker line_strip_; // Line strip marker for the path
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneVisualizer>());
    rclcpp::shutdown();
    return 0;
}
