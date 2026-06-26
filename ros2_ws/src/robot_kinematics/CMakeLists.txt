cmake_minimum_required(VERSION 3.8)
project(robot_kinematics)

find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME} SHARED
  src/narrow_track_maps.cpp
  src/wide_track_maps.cpp
  src/steering_angle_to_curvature.cpp
  src/curvature_to_joints.cpp
  src/spin_to_joints.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(${PROJECT_NAME} PUBLIC fsm)

install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(fsm)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
