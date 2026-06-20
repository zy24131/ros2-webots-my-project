#!/usr/bin/env bash
# Webots 启动子进程时不带 ROS 库路径，需在此 source 后再 exec 真实二进制。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../../.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
else
  echo "ros2_bridge_wrapper: no ROS setup.bash found" >&2
  exit 1
fi

if [ -f "${ROOT}/ros2_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${ROOT}/ros2_ws/install/setup.bash"
fi

exec "${HERE}/ros2_bridge_bin" "$@"
