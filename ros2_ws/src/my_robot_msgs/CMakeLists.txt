cmake_minimum_required(VERSION 3.8)
project(my_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SteeringCommand.msg"
  "msg/WheelSpeeds.msg"
  "action/SetTrackAction.action"
  "srv/SetMotion.srv"
  "srv/SetTrack.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
