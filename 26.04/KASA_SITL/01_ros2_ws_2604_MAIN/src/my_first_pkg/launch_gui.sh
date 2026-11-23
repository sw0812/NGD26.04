#!/bin/bash
# main.py(KASA 미션 제어 GUI)는 rclpy/px4_msgs가 필요해서 ROS2 환경을 먼저
# source해야 함 — 바탕화면 아이콘으로 더블클릭해서 켤 땐 이게 안 되어 있으므로
# 여기서 대신 해주는 래퍼.
#
# ROS_DOMAIN_ID를 명시적으로 0으로 고정 — 안 그러면 이 GUI가 켜는
# search_phase_node.py(및 터미널 1~7 버튼들)가 그 순간 셸 환경의 기본값을
# 따라가는데, SITL/브릿지 쪽을 다른 셸(다른 기본 도메인)에서 띄우면
# 서로 도메인이 어긋나서 /fmu/out/* 토픽이 하나도 안 보임 — 노드는
# 멀쩡히 떠서 실행 중인데 PX4 위치/ARM 상태를 영영 못 받아 미션이
# 멈춰있는 것처럼 보이는 버그로 실측됨(2026-08-24). 고정해두면 이 GUI로
# 켜는 모든 것이 항상 서로 같은 도메인이라 이 클래스 버그 자체가 없어짐.
export ROS_DOMAIN_ID=0
source /opt/ros/foxy/setup.bash
source /home/sasllab/ros2_ws/install/setup.bash
exec python3 /home/sasllab/ros2_ws/src/my_first_pkg/main.py
