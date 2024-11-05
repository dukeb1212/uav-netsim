cmake_minimum_required(VERSION 3.8)
project(ns3_drone_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ns3_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(airsim_interfaces REQUIRED)

add_executable(position src/position_visualizer.cpp src/utils.cpp)

ament_target_dependencies(position
  rclcpp
  std_msgs
  ns3_interfaces
  visualization_msgs
  sensor_msgs
  airsim_interfaces
)

install(TARGETS
        position
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/position_csv.py  # Path to your Python script
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
