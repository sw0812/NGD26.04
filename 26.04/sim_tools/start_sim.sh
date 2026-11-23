#!/usr/bin/env bash
source ~/바탕화면/sim_tools/kasa_ros_env.sh   # ROS 도메인 42 분리 (2026-11-12)
# PX4 SITL(gz, tarot680pro, kasa_indoor_search) 데몬으로 기동 + Ready 대기. 기존 인스턴스는 종료.
P=$(pgrep -x px4; pgrep -x gz-sim-main; pgrep -x gz-sim-gui-clie); [ -n "$P" ] && kill $P 2>/dev/null; sleep 3
P=$(pgrep -x px4; pgrep -x gz-sim-main; pgrep -x gz-sim-gui-clie); [ -n "$P" ] && kill -9 $P 2>/dev/null; sleep 1
cd ~/바탕화면/KASA_SITL && source /opt/ros/lyrical/setup.bash && source .venv-px4/bin/activate
cd PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
export PX4_SIM_MODEL=gz_tarot680pro GZ_IP=127.0.0.1 PX4_GZ_WORLD=kasa_indoor_search PX4_GZ_MODEL_POSE="0,0,0.95,0,0,0"
LOG=$(dirname "$(readlink -f "$0")")/px4_sitl.log
setsid nohup ~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default/bin/px4 -d > $LOG 2>&1 < /dev/null &
# 2026-11-11 사용자 지시: 시뮬 시작할 때마다 지자기 보정값(비행 중 학습돼 저장됨) 0으로 초기화
mag0() { for a in X Y Z; do ~/바탕화면/KASA_SITL/PX4-Autopilot/build/px4_sitl_default/bin/px4-param set CAL_MAG0_${a}OFF 0 >/dev/null 2>&1; done; }
for i in $(seq 1 60); do grep -q "Ready for takeoff" $LOG && { mag0; echo READY; exit 0; }; sleep 2; done
echo "NOT READY"; tail -20 $LOG; exit 1
