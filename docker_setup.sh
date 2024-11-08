#!/bin/bash

# Set up network link
setup_network_link() {
  local i="$1"
  
  # Create veth pair
  ip link add "veth_host$i" type veth peer name "veth_uav$i"
  
  # Bring up veth interfaces
  ip link set "veth_host$i" up
  ip link set "veth_uav$i" up
  
  # Create tap interface
  ip tuntap add "tap_uav$i" mode tap
  ip addr flush dev "tap_uav$i"
  ip link set "tap_uav$i" up
}

# Set up network link inside docker container of UAV
set_up_docker_uav() {
  local i="$1"
  
  # Get PID of container
  pid=$(docker inspect -f '{{.State.Pid}}' "uav$i") # Container names must be in the format of uav1, uav2, etc.
  
  # Check if PID retrieval was successful
  if [ -z "$pid" ]; then
    echo "Error: Failed to retrieve PID for container uav$i"
    exit 1
  fi
  
  # Move the interface from host to docker container
  ip link set "veth_uav$i" netns "$pid"
}

# Set up network link inside docker container of ns3 simulation
set_up_docker_ns3() {
  local num="$1"
  
  # Get PID of the ns3 container
  pid_ns3=$(docker inspect -f '{{.State.Pid}}' ns3_sim)
  
  # Check if PID retrieval was successful
  if [ -z "$pid_ns3" ]; then
    echo "Error: Failed to retrieve PID for container ns3_sim"
    exit 1
  fi

  # Move the interfaces into the ns3 container's network namespace
  for ((i=1; i<= $num; i++)); do
    ip link set "veth_host$i" netns "$pid_ns3"
    ip link set "tap_uav$i" netns "$pid_ns3"
  done
}

count="${1:-2}"

echo "Setting up for $count UAV networks..."

# Start all the containers
docker compose up -d

# Set up network
for (( i=1; i<=count; i++ )); do
    setup_network_link "$i"
    set_up_docker_uav "$i"
done
set_up_docker_ns3 $count

echo "All done!"

# Usage:
# ./docker_setup.sh --> Default 2 devices
# ./docker_setup.sh 4 --> use 4 devices
