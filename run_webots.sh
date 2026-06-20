#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/ros2_ws"
WEBOTS="${ROOT}/webots"

source /opt/ros/humble/setup.bash
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"

cd "${WS}"
colcon build --symlink-install --packages-select \
  my_robot_msgs my_robot_maps hardware_bridge \
  steering_controller actuator_executor mobility_controller robot_fsm
source install/setup.bash

exec webots --stdout --stderr --mode=realtime "${WEBOTS}/worlds/my_project.wbt" "$@"
