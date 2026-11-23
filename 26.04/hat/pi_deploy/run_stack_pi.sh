#!/usr/bin/env bash
# 실기체(Pi) 스택. sim_tools/mission_stack.sh 의 실기 버전: gz bridge 대신 USB 카메라(usb_cam).
# 사용(컨테이너 안): run_stack_pi.sh [mission|landing|all]   (landing 은 호스트 hailo_server 필요)
set -u
MODE=${1:-mission}
ROS_DISTRO=${ROS_DISTRO:-jazzy}; WS=${WS:-$HOME/kasa_ws}
REPO=${REPO:-$HOME/NGD26.04}; MP=$REPO/26.04/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg
source /opt/ros/$ROS_DISTRO/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}            # 시뮬과 동일 격리 규칙 (ros-domain-isolation)
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
CFG=$(ros2 pkg prefix kasa_hailo)/share/kasa_hailo/config
LOGD=${LOGD:-$HOME/logs/$(date +%F_%H%M)}; mkdir -p $LOGD
trap 'kill $(jobs -p) 2>/dev/null' EXIT

# usb_cam 파라미터: 캘리브레이션 경로 치환
sed "s#@CALIB@#$CFG/c270_calibration.yaml#" $CFG/c270_usb_cam.yaml.in > /tmp/c270_usb_cam.yaml
if [ "$MODE" != landing ]; then
  ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera -r __node:=usb_cam_node --params-file /tmp/c270_usb_cam.yaml &
  # aruco: CPU(OpenCV) 알고리즘 — NPU 대상 아님. 전방 카메라는 별도 usb_cam 이 /camera_front 로 있을 때만.
  ros2 run aruco_opencv aruco_tracker --ros-args --params-file $CFG/aruco_kasa.yaml &
  for i in $(seq 20); do ros2 lifecycle get /aruco_tracker >/dev/null 2>&1 && break; sleep 1; done
  ros2 lifecycle set /aruco_tracker configure; ros2 lifecycle set /aruco_tracker activate
  (cd $MP && python3 -u line_odometry_node.py --ros-args -p csv:=$LOGD/line_odom.csv > $LOGD/line_odom.log 2>&1 &)
  (cd $MP && python3 -u search_phase_node.py --ros-args -p n_markers_expected:=4 -p use_front_cam:=false > $LOGD/mission_node.log 2>&1 &)
fi
if [ "$MODE" != mission ]; then
  ros2 launch kasa_hailo hailo_landing.launch.py &
fi
wait
