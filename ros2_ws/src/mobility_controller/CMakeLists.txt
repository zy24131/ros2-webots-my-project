cmake_minimum_required(VERSION 3.8)
project(mobility_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(mobility_controller_node src/mobility_controller_node.cpp)
ament_target_dependencies(
  mobility_controller_node
  my_robot_msgs
  rclcpp
  std_msgs
  rosidl_default_runtime)

install(TARGETS mobility_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
