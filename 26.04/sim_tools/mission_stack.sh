#!/usr/bin/env bash
source ~/바탕화면/sim_tools/kasa_ros_env.sh   # ROS 도메인 42 분리 (2026-11-12)
# gz 하향+전방 카메라 -> ROS2 브리지 + aruco_tracker 2개 (하향: /aruco_detections, 전방: /aruco_front/detections)
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_IP=127.0.0.1
D=$(dirname "$(readlink -f "$0")")
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$D/gz_bridge.yaml &
ros2 run aruco_opencv aruco_tracker --ros-args --params-file $D/aruco_kasa.yaml &
ros2 run aruco_opencv aruco_tracker --ros-args -r __node:=aruco_front -r /aruco_detections:=/aruco_front/detections -r aruco_detections:=/aruco_front/detections --params-file $D/aruco_front.yaml &
for n in /aruco_tracker /aruco_front; do
  for i in $(seq 20); do ros2 lifecycle get $n >/dev/null 2>&1 && break; sleep 1; done
  ros2 lifecycle set $n configure; ros2 lifecycle set $n activate
done
wait
