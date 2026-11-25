#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${WS_DIR}/scripts/workspace_env.sh"
require_ros_packages robot_localization nav2_bringup gazebo_ros gazebo_ros2_control
cd "${WS_DIR}"
exec colcon build --symlink-install --base-paths "${WS_DIR}/src" "$@"
