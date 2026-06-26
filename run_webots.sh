#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/ros2_ws"
WEBOTS="${ROOT}/webots"

source /opt/ros/humble/setup.bash
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"

cd "${WS}"
colcon build --packages-select \
  fsm my_robot_msgs robot_kinematics hardware_bridge \
  steering_control actuator_control wheel_speed_control
source install/setup.bash

exec webots --stdout --stderr --mode=realtime "${WEBOTS}/worlds/my_project.wbt" "$@"
