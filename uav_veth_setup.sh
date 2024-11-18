#!/bin/bash

# Wait for IP setup on host to finish
sleep 10

echo "Assigning IP address to veth_uav interface"

airsim=$2

# IP addresses calculate function. The IP address range will be from 10.0.0.1 to 10.127.254.254
calculate_ips() {
  local i="$1"
  if (( i < 1 || i > (2**20 / 3) )); then
    echo "Index out of range"
    exit 1
  fi

  # Calculate IPs
  local j=$(( (i-1) * 3 + 1 ))
  veth_uav="10.$(( j / 65536 )).$(( (j / 256) % 256 )).$(( j % 256 ))/9"
}

calculate_ips "$1"

# Add IP address to interface and bring up
ip addr add "$veth_uav" dev "veth_uav"$1
ip link set "veth_uav"$1 up

echo "New IP addr of veth_uav is $veth_uav"

sleep 5
if [ "$airsim" = "true" ]; then
  cd /AirSim/ros2
  #. /opt/ros/$ROS_DISTRO/setup.sh
  #colcon build
  . install/setup.sh
  ros2 launch airsim_ros_pkgs airsim_node.launch.py output:=screen host:=$WSL_HOST_IP
fi

# Source the ROS2 setup for future use
cd /uav
. install/setup.sh


# Keep the container alive
sleep infinity