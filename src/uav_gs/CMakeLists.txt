cmake_minimum_required(VERSION 3.13)
project(uav_gs)

# use C++ 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(ns3_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)


# add ns3 libraries
include_directories(src /mnt/d/ros_ue5_ns3/ns-3-allinone/ns-3.42/build/include)
link_directories(/mnt/d/ros_ue5_ns3/ns-3-allinone/ns-3.42/build/lib)

add_executable(uav_gs_cpp src/uav_gs.cpp src/drone.cpp src/ground_station.cpp src/utils.cpp)

ament_target_dependencies(uav_gs_cpp
  rclcpp
  urdf
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  airsim_interfaces
  ns3_interfaces
)

# add link libraries
target_link_libraries(uav_gs_cpp
  ns3.42-core-default
  ns3.42-network-default
  ns3.42-internet-default
  ns3.42-wifi-default
  ns3.42-mobility-default
  ns3.42-tap-bridge-default
  ns3.42-internet-apps-default
  ns3.42-aodv-default
)

install(TARGETS
        uav_gs_cpp
        RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install Python nodes
install(
  PROGRAMS src/ping_publisher.py src/ping_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
