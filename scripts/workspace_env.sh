#!/usr/bin/env bash
# Source from the entry scripts, without inheriting another ROS workspace.
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '需要 Ubuntu 22.04 和 ROS 2 Humble，请先按 README 安装 ROS。' >&2
  return 1
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
unset GAZEBO_HOSTNAME IGN_RELAY
source /opt/ros/humble/setup.bash
if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  LOCAL_ROS_PREFIX="${WS_DIR}/.deps/root/opt/ros/humble"
  if [[ -f "${LOCAL_ROS_PREFIX}/local_setup.bash" ]]; then
    source "${LOCAL_ROS_PREFIX}/local_setup.bash"
    export LD_LIBRARY_PATH="${WS_DIR}/.deps/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  fi
fi
export ROS_DOMAIN_ID=03 ROS_LOCALHOST_ONLY=1
export GAZEBO_IP=127.0.0.1 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export RMW_IMPLEMENTATION="${DASHBOARD_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

require_ros_packages() {
  local missing=0 package
  for package in "$@"; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
      echo "缺少 ROS 包：${package}" >&2
      missing=1
    fi
  done
  if (( missing )); then
    echo '请执行 ./setup_dependencies.sh --system，或按 README 安装系统依赖。' >&2
    return 1
  fi
}

load_workspace() {
  if [[ ! -f "${WS_DIR}/install/local_setup.bash" ]]; then
    echo '尚未编译当前工作空间，请先执行 ./build_workspace.sh。启动不会自动编译。' >&2
    return 1
  fi
  source "${WS_DIR}/install/local_setup.bash"
  local prefix
  prefix="$(ros2 pkg prefix roslander_navigation 2>/dev/null)" || {
    echo '当前工作空间缺少已安装的导航包，请执行 ./build_workspace.sh。' >&2
    return 1
  }
  if [[ "$(realpath "$prefix")" != "$(realpath "${WS_DIR}/install/roslander_navigation")" ]]; then
    echo '导航包没有来自当前工作空间，请重新执行 ./build_workspace.sh。' >&2
    return 1
  fi
}
