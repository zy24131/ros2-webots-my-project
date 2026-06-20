cmake_minimum_required(VERSION 3.8)
project(mobility_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mobility_node src/mobility_node.cpp)
ament_target_dependencies(mobility_node rclcpp std_msgs)

install(TARGETS mobility_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
