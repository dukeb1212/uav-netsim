#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import matplotlib.animation as animation
import argparse
import os
from datetime import datetime

def update_plot(frame_num, file_path, plot):
    # Read the CSV data at each frame update
    data = pd.read_csv(file_path)
    plot._offsets3d = (data["east"], data["north"], data["up"])

def save_snapshots(ax, output_folder, base_file_name):
    # Define angles for snapshots (azimuth, elevation)
    angles = [
        (0, 90),     # Top-down view
        (45, 45),    # Top-right corner view
        (135, 45),   # Top-left corner view
        (225, 45),   # Bottom-left corner view
        (315, 45),   # Bottom-right corner view
        (0, 0),      # Positive X-axis side view
        (90, 0)     # Negative X-axis side view
    ]
    for i, (azim, elev) in enumerate(angles):
        ax.view_init(elev=elev, azim=azim)
        plt.draw()  # Update the view
        # Save each view with the base file name at the beginning
        image_path = os.path.join(output_folder, f"{base_file_name}_view_{i+1}.png")
        plt.savefig(image_path)
        print(f"Saved view {i+1} to {image_path}")

def main():
    # Argument parsing to accept a CSV file path
    parser = argparse.ArgumentParser(description="3D plot of GPS ENU data from a CSV file")
    parser.add_argument("-f", "--file", type=str, required=True, help="Path to the CSV file containing GPS data")
    args = parser.parse_args()

    # Load initial data from the CSV file
    data = pd.read_csv(args.file)

    # Extract base file name (without path and extension)
    base_file_name = os.path.splitext(os.path.basename(args.file))[0]
    
    # Setup the figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")

    # Initial empty scatter plot
    plot = ax.scatter(data["east"], data["north"], data["up"], c='red', marker='o')

    # Real-time update using FuncAnimation
    ani = animation.FuncAnimation(fig, update_plot, fargs=(args.file, plot), interval=100)  # Update every second
    
    # Output folder for snapshots
    output_folder = os.path.join("result", datetime.now().strftime("%Y-%m-%d"))
    os.makedirs(output_folder, exist_ok=True)

    # Save snapshots from different angles
    save_snapshots(ax, output_folder, base_file_name)

    plt.show()

if __name__ == '__main__':
    main()
