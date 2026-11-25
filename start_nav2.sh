#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
  cat <<'HELP'
Usage: ./start_nav2.sh [ROS launch arguments]
Starts Gazebo + EKF + AMCL + Nav2 + RViz, using this workspace's map_01.
This command never builds. Run ./build_workspace.sh after deployment or changes.
  ./start_nav2.sh gui:=false       Hide native Gazebo; browser GUI is independent
  ./start_nav2.sh rviz:=false      Hide native RViz
  ./start_nav2.sh --check          Check dependencies and installed workspace only
  ./start_nav2.sh --show-args      List ROS launch arguments
HELP
  exit 0
fi
source "${WS_DIR}/scripts/workspace_env.sh"
require_ros_packages nav2_bringup nav2_amcl nav2_dwb_controller nav2_navfn_planner nav2_rviz_plugins rviz2 robot_localization
load_workspace
MAP_FILE="${WS_DIR}/maps/map_01.yaml"
[[ -f "$MAP_FILE" ]] || { echo "地图不存在：${MAP_FILE}" >&2; exit 1; }
if [[ "${1:-}" == '--check' ]]; then
  echo "环境及当前工作空间编译产物就绪：${WS_DIR}"
  exit 0
fi
LAUNCH_ARGS=()
START_SIM=true
for arg in "$@"; do
  [[ "$arg" == start_sim:=false ]] && START_SIM=false
  [[ "$arg" == map:=* ]] || LAUNCH_ARGS+=("$arg")
done
if [[ "$START_SIM" == true && "$*" != *--show-args* ]] && pgrep -x gzserver >/dev/null; then
  echo '已有 gzserver 正在运行，请核对来源，避免重复启动仿真。' >&2
  exit 1
fi
cd "${WS_DIR}"
exec ros2 launch roslander_navigation navigation.launch.py "${LAUNCH_ARGS[@]}" "map:=${MAP_FILE}"
