#!/usr/bin/env bash
source ~/바탕화면/sim_tools/kasa_ros_env.sh   # ROS 도메인 42 분리 (2026-11-12)
# legacy(10회차 방식) / grid(12회차 방식) 각각 1회 비행 -> ulog 저장 -> 채점
D=~/바탕화면/sim_tools; B=~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default
PY=~/바탕화면/KASA_SITL/.venv-px4/bin/python
source /opt/ros/lyrical/setup.bash; source ~/ros2_ws/install/setup.bash
cd ~/바탕화면/KASA_SITL/01_ros2_ws_2604_MAIN/src/my_first_pkg
for pair in "13 legacy" "14 grid"; do
  set -- $pair; R=$1; DET=$2
  until $B/bin/px4-listener vehicle_status -n 1 | grep -q " arming_state: 1"; do sleep 2; done
  python3 -u search_phase_node.py --ros-args -p n_markers_expected:=4 -p intersection_detector:=$DET > $D/run${R}_mission_node.log 2>&1
  until $B/bin/px4-listener vehicle_status -n 1 | grep -q " arming_state: 1"; do sleep 2; done; sleep 3
  cp $(ls -t $B/rootfs/log/*/*.ulg | head -1) $D/run${R}.ulg
  echo "run$R ($DET): markers $(grep -c 'ArUco ID.* 확정' $D/run${R}_mission_node.log)/4, $(grep -c DONE $D/run${R}_mission_node.log) done"
  $PY $D/score_arrivals.py $D/run${R}_mission_node.log $D/run${R}.ulg "run$R $DET"
done
