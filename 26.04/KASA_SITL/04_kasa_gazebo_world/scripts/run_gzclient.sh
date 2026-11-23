#!/usr/bin/env bash
# gzserver가 이미 떠 있는 상태에서 3D 뷰(gzclient)만 붙일 때 사용.
# GAZEBO_RESOURCE_PATH를 덮어쓰면 셰이더 라이브러리를 못 찾아 크래시하므로
# 반드시 기본 gazebo setup.sh를 먼저 source한 뒤 커스텀 경로를 "추가"한다.
set -e

source /usr/share/gazebo/setup.sh 2>/dev/null || source /usr/share/gazebo-11/setup.sh

export GAZEBO_MODEL_PATH=/home/sasllab/Downloads/kasa_gazebo_world/models:/home/sasllab/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/home/sasllab/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$GAZEBO_RESOURCE_PATH

exec gzclient --verbose
