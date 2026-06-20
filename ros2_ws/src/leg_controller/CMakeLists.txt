cmake_minimum_required(VERSION 3.8)
project(leg_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(my_robot_maps REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(leg_controller src/leg_controller.cpp)
ament_target_dependencies(
  leg_controller
  my_robot_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs)
target_include_directories(leg_controller PUBLIC ${my_robot_maps_INCLUDE_DIRS})
target_link_libraries(leg_controller ${my_robot_maps_LIBRARIES})

install(TARGETS leg_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
