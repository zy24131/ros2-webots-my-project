cmake_minimum_required(VERSION 3.8)
project(steering_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(my_robot_maps REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(steering_input_node src/steering_input_node.cpp)
ament_target_dependencies(steering_input_node rclcpp std_msgs)
target_include_directories(steering_input_node PUBLIC ${my_robot_maps_INCLUDE_DIRS})
target_link_libraries(steering_input_node ${my_robot_maps_LIBRARIES})

install(TARGETS steering_input_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
