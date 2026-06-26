cmake_minimum_required(VERSION 3.8)
project(hardware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

if(DEFINED ENV{WEBOTS_HOME})
  set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
  set(WEBOTS_HOME "/usr/local/webots")
endif()

get_filename_component(WEBOTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../webots" ABSOLUTE)
set(CONTROLLER_DIR "${WEBOTS_ROOT}/controllers/ros2_bridge")

include_directories(${WEBOTS_HOME}/include/controller/cpp)

add_executable(ros2_bridge_bin src/ros2_bridge.cpp)
ament_target_dependencies(
  ros2_bridge_bin
  fsm
  my_robot_msgs
  rclcpp
  sensor_msgs
  std_msgs
  rosidl_default_runtime)
target_link_libraries(ros2_bridge_bin ${WEBOTS_HOME}/lib/controller/libCppController.so)
target_compile_features(ros2_bridge_bin PUBLIC cxx_std_17)

install(TARGETS ros2_bridge_bin
  RUNTIME DESTINATION ${CONTROLLER_DIR})

install(PROGRAMS scripts/ros2_bridge_wrapper.sh
  DESTINATION ${CONTROLLER_DIR}
  RENAME ros2_bridge)

ament_package()
