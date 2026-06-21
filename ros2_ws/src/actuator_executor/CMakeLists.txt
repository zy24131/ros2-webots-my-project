cmake_minimum_required(VERSION 3.8)
project(actuator_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(actuator_executor_node src/actuator_executor_node.cpp)
ament_target_dependencies(
  actuator_executor_node
  my_robot_msgs
  rclcpp
  sensor_msgs
  std_msgs
  rosidl_default_runtime)

install(TARGETS actuator_executor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
