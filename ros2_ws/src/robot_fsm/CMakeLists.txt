cmake_minimum_required(VERSION 3.8)
project(robot_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_fsm src/robot_fsm.cpp)
ament_target_dependencies(
  robot_fsm
  my_robot_msgs
  rclcpp
  rclcpp_action
  std_msgs)

install(TARGETS robot_fsm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
