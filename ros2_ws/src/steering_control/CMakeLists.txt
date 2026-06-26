cmake_minimum_required(VERSION 3.8)
project(steering_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(robot_kinematics REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(steering_control_node src/steering_control_node.cpp)
ament_target_dependencies(
  steering_control_node
  fsm
  my_robot_msgs
  rclcpp
  std_msgs
  rosidl_default_runtime)
target_include_directories(steering_control_node PUBLIC ${robot_kinematics_INCLUDE_DIRS})
target_link_libraries(steering_control_node ${robot_kinematics_LIBRARIES})

install(TARGETS steering_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
