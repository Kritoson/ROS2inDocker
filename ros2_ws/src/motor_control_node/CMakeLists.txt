cmake_minimum_required(VERSION 3.5)
project(motor_control_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motor_control_node src/motor_control_node.cpp)

ament_target_dependencies(
  motor_control_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(
  TARGETS motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
