#!/bin/bash

# Wait for IP setup on host to finish
sleep 10

echo "Assigning IP address to veth_host and tap_uav interface"

# Total number of simulation devices
count="$1"

# IP addresses calculate function. The IP address range will be from 10.0.0.1 to 10.127.254.254
calculate_ips() {
  local i="$1"
  if (( i < 1 || i > (2**20 / 3) )); then
    echo "Index out of range"
    exit 1
  fi

  # Calculate IPs
  local j=$(( (i-1) * 3 + 2 ))
  local k=$(( (i-1) * 3 + 3 ))
  veth_host="10.$(( j / 65536 )).$(( (j / 256) % 256 )).$(( j % 256 ))/9"
  tap_uav="10.$(( k / 65536 )).$(( (k / 256) % 256 )).$(( k % 256 ))/9"
}

# Add IP address to interface and bring up
setup_network_link() {
  local i="$1"
  calculate_ips "$i"
  
  ip addr add "$veth_host" dev "veth_host$i"
  
  ip link add "uav_br$i" type bridge
  ip link set "uav_br$i" up
  ip link set "veth_host$i" up
  
  ip link set "veth_host$i" master "uav_br$i"
  
  ip addr add "$tap_uav" dev "tap_uav$i"
  ip link set "tap_uav$i" up
  
  ip link set "tap_uav$i" master "uav_br$i"

  echo "New IP addr of veth_host$i is $veth_host"
  echo "New IP addr of tap_uav$i is $tap_uav"
}

for (( i=1; i<=$count; i++)); do
    setup_network_link "$i"
done

# Source the ROS2 setup
. install/setup.sh

# Run the ROS2-NS3 simulation
ros2 run uav_gs uav_gs_cpp

# Keep the container alive
sleep infinity