#!/usr/bin/env bash
source ~/바탕화면/sim_tools/kasa_ros_env.sh   # ROS 도메인 42 분리 (2026-11-12)
# 영상처리(라인트레이싱) + 컴퓨터비전(ArUco) + 전방카메라 화면 띄우기. mission_stack.sh가 먼저 돌고 있어야 함.
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
export QT_QPA_PLATFORM=xcb GZ_IP=127.0.0.1
D=$(dirname "$(readlink -f "$0")")
ros2 run ros_gz_bridge parameter_bridge --ros-args -r __node:=front_cam_bridge -p config_file:=$D/gz_bridge_front.yaml &
cd ~/바탕화면/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg
python3 line_tracker_viz_node.py &
ros2 run rqt_image_view rqt_image_view /line_tracker/debug &
ros2 run rqt_image_view rqt_image_view /aruco_tracker/debug &
ros2 run rqt_image_view rqt_image_view /camera_front/image_raw &
wait
