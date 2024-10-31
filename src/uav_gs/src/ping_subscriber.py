#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ns3_interfaces.msg import IcmpMsg  # Ensure this matches the publisher message type

class PingSubNode(Node):
    def __init__(self):
        super().__init__('ping_sub_node')

        # Subscription to the 'ping_response' topic
        self.subscription = self.create_subscription(
            IcmpMsg,               # Corrected message type
            'ping_response',       # Topic name matching the publisher
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received data in a formatted output
        timestamp_sec = msg.timestamp.sec
        timestamp_nsec = msg.timestamp.nanosec
        self.get_logger().info(
            f"Received ICMP Data | "
            f"Timestamp: {timestamp_sec}.{timestamp_nsec:09d} | "
            f"Seq: {msg.seq} | TTL: {msg.ttl} | "
            f"Size: {msg.size} bytes | Latency: {msg.latency:.3f} ms"
        )

def main(args=None):
    rclpy.init(args=args)
    ping_sub_node = PingSubNode()
    rclpy.spin(ping_sub_node)
    ping_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
