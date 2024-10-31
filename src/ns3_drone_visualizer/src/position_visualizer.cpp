#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ns3_interfaces/msg/ns3_position_msg.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class DroneVisualizer : public rclcpp::Node
{
public:
    DroneVisualizer() : Node("drone_visualizer"), point_count_(0)
    {
        // Subscription to the drone location topic
        subscription_ = this->create_subscription<ns3_interfaces::msg::Ns3PositionMsg>(
            "drone_location",
            10,
            std::bind(&DroneVisualizer::onLocationReceived, this, std::placeholders::_1)
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
    void onLocationReceived(const ns3_interfaces::msg::Ns3PositionMsg::SharedPtr msg)
    {
        // Create a marker for the new point
        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = "uav1";  // Change to the appropriate frame
        point_marker.header.stamp = this->get_clock()->now();
        point_marker.ns = "drone_points";
        point_marker.id = point_count_; // Unique ID for each point
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.pose.position.x = msg->x;
        point_marker.pose.position.y = msg->y;
        point_marker.pose.position.z = msg->z;

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
        p.x = msg->x;
        p.y = msg->y;
        p.z = msg->z;
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

    rclcpp::Subscription<ns3_interfaces::msg::Ns3PositionMsg>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int point_count_;  // Counter for the number of points

    visualization_msgs::msg::Marker line_strip_; // Line strip marker for the path
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneVisualizer>());
    rclcpp::shutdown();
    return 0;
}
