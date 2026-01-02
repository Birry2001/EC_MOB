#!/usr/bin/env bash
set -euo pipefail

WS="/home/nochi/NOCHI/M2_PAR/Projet_de_synthese/project_ws"
BAG="/home/nochi/NOCHI/M2_PAR/Projet_de_synthese/Ros_bags/Event_datas/rosbag2_2025_12_19-13_54_18"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

LOG_DIR="${WS}/event_mask_logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/event_mask_diag_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "${LOG_FILE}") 2>&1
echo "Log file: ${LOG_FILE}"

ros2 launch datasync_2_0 rosbag_pipeline.launch.py use_bag:=true use_rviz:=false bag_path:="${BAG}" &
LAUNCH_PID=$!
trap 'kill ${LAUNCH_PID} >/dev/null 2>&1 || true' EXIT

sleep 5

echo "== Topics =="
ros2 topic list

echo "== /count_image =="
ros2 topic echo --once /count_image || true

echo "== /time_image =="
ros2 topic echo --once /time_image || true

echo "== /event_mask =="
ros2 topic echo --once /event_mask || true

echo "== /event_mask rate =="
ros2 topic hz /event_mask -w 50 || true

echo "== Switching to count-only mask (use_time_image=false) =="
ros2 param set /event_segmentation use_time_image false || true
sleep 2
ros2 topic echo --once /event_mask || true
