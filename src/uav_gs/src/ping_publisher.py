#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ns3_interfaces.msg import IcmpMsg  # Ensure this matches your package structure
from ping3 import ping  # Import ping from ping3
from datetime import datetime

class PingPublisher(Node):
    def __init__(self):
        super().__init__('ping_pub_node')

        # Parameters
        self.target_ip = self.declare_parameter('target_ip', '10.0.0.1').value
        self.timeout = self.declare_parameter('timeout', 1.0).value

        # Publisher
        self.publisher_ = self.create_publisher(IcmpMsg, 'ping_response', 10)

        # Set timer to call ping_target every second
        self.timer = self.create_timer(1.0, self.ping_target)

        # Initialize sequence number
        self.seq = 1

        self.get_logger().info(f'Start pinging host: {self.target_ip}')

    def ping_target(self):
        start_time = datetime.now()
        self.get_logger().info('Pinging target...')

        # Initialize variables for ping response
        latency = -1.0  # Initialize latency to -1.0 for failed pings
        ttl, size = 0, 0  # Initialize size to 0

        try:
            # Send one ping packet and wait for the response
            latency = ping(self.target_ip, timeout=self.timeout, unit='ms')

            if latency is None or latency <= 0:
                 # Ping failed or invalid latency
                 latency = -1.0  # Set latency to -1 for failure
                 ttl, size = 0, 0  # Set TTL and size to 0
                 self.get_logger().warn(f'Ping to {self.target_ip} failed.')
            else:
                 # Successful ping
                 ttl = 64  # Common default TTL; adjust if necessary
                 size = 64  # Default ICMP packet size; adjust as needed
                 self.get_logger().info(f'Ping successful: latency={latency:.3f}ms, seq={self.seq}, ttl={ttl}, size={size}')

        except Exception as e:
            self.get_logger().error(f'An error occurred while pinging: {str(e)}')

        # Create and publish the ICMP message
        msg = IcmpMsg()

        # Convert datetime to seconds and nanoseconds
        msg.timestamp.sec = int(start_time.timestamp())  # Get total seconds
        msg.timestamp.nanosec = start_time.microsecond * 1000  # Convert microseconds to nanoseconds

        msg.seq = self.seq  # Use the class attribute seq
        msg.ttl = ttl
        msg.size = size
        msg.latency = float(latency)

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Published ICMP message.')

        # Increment the seq for the next ping
        self.seq += 1  # Increment seq after publishing

def main(args=None):
    rclpy.init(args=args)
    ping_node = PingPublisher()
    rclpy.spin(ping_node)
    ping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
