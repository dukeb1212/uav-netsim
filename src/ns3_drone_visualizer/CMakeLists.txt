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

add_executable(ns3_drone_visualizer src/position_visualizer.cpp)

ament_target_dependencies(visualizer
  rclcpp
  std_msgs
  ns3_interfaces
  visualization_msgs
)

install(TARGETS
        ns3_drone_visualizer
        RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
