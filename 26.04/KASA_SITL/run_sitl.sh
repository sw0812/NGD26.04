#!/usr/bin/env bash
# tarot680pro + KASA 실내 월드 SITL 실행 (QGroundControl은 별도로 실행)
cd "$(dirname "$0")"
source /opt/ros/lyrical/setup.bash
source .venv-px4/bin/activate
cd PX4-Autopilot
export PX4_GZ_WORLD=kasa_indoor_search
export PX4_GZ_MODEL_POSE="0,0,0.75,0,0,0"   # 버티포트 상판 위
make px4_sitl gz_tarot680pro
