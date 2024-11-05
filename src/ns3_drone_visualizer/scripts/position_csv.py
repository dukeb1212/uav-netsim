#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from airsim_interfaces.msg import GPSYaw
import csv
from datetime import datetime
from pyproj import Transformer
import numpy as np
import os
import argparse

class DroneVisualizer(Node):
    def __init__(self, file_name):
        super().__init__('drone_visualizer')
        
        # Subscriptions
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/airsim_node/uav1/gps/gps',
            self.on_location_received,
            10
        )
        self.origin_subscription = self.create_subscription(
            GPSYaw,
            '/airsim_node/origin_geo_point',
            self.on_origin_received,
            10
        )
        
        # Variables
        self.origin = None

        # Create folder structure
        today_date = datetime.now().strftime("%Y-%m-%d")
        current_time = datetime.now().strftime("%H-%M-%S")
        result_folder = os.path.join("result", today_date)
        os.makedirs(result_folder, exist_ok=True)

        # Final file path with time appended to user-provided file name
        self.csv_file_path = os.path.join(result_folder, f"{file_name}_{current_time}.csv")
        
        # Open CSV file
        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "east", "north", "up"])  # CSV headers
        
        # Initialize pyproj Transformer for LLA to ECEF conversion
        self.transformer_lla_to_ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978", always_xy=True)

    def on_origin_received(self, msg):
        self.origin = (msg.latitude, msg.longitude, msg.altitude)

    def on_location_received(self, gps_msg):
        if self.origin is None:
            self.get_logger().warn("Origin not set, ignoring GPS message.")
            return
        
        # Convert GPS coordinates to ENU
        current_gps = (gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)
        enu_point = self.lla_to_enu(current_gps, self.origin)

        # Convert numpy.float64 to Python float for CSV writing
        enu_point = [float(coord) for coord in enu_point]
        
        # Write to CSV
        timestamp = datetime.now().isoformat()
        self.csv_writer.writerow([timestamp, *enu_point])
        self.get_logger().info(f"GPS data saved: {timestamp}, ENU: {enu_point}")

    def lla_to_enu(self, current_lla, origin_lla):
        # Convert origin LLA to ECEF
        origin_ecef_x, origin_ecef_y, origin_ecef_z = self.transformer_lla_to_ecef.transform(
            origin_lla[1], origin_lla[0], origin_lla[2]
        )
        
        # Convert current LLA to ECEF
        current_ecef_x, current_ecef_y, current_ecef_z = self.transformer_lla_to_ecef.transform(
            current_lla[1], current_lla[0], current_lla[2]
        )
        
        # Calculate ECEF coordinate differences
        dx = current_ecef_x - origin_ecef_x
        dy = current_ecef_y - origin_ecef_y
        dz = current_ecef_z - origin_ecef_z
        
        # Compute the ENU conversion matrix based on the origin's latitude and longitude
        lat, lon = np.radians(origin_lla[0]), np.radians(origin_lla[1])
        enu_matrix = np.array([
            [-np.sin(lon), np.cos(lon), 0],
            [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
            [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]
        ])
        
        # Transform the ECEF differences to ENU coordinates
        enu_vector = enu_matrix @ np.array([dx, dy, dz])
        enu_east, enu_north, enu_up = enu_vector
        
        return (enu_east, enu_north, enu_up)

    def __del__(self):
        self.csv_file.close()
        self.get_logger().info(f"CSV file saved at: {self.csv_file_path}")

def main():
    parser = argparse.ArgumentParser(description="Drone Visualizer with CSV output")
    parser.add_argument('-f', '--file', type=str, required=True, help="Base name for the CSV file")
    args = parser.parse_args()

    # Initialize ROS 2 without passing the argparse namespace
    rclpy.init()
    
    # Pass the parsed file name to the node
    node = DroneVisualizer(args.file)
    
    # Spin node
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
