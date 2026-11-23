#!/usr/bin/env bash
# PX4 SITL 빌드 (ROS 2 Lyrical에 들어있는 신버전 Gazebo(gz-sim 10 / gz-transport15)를 사용)
cd "$(dirname "$0")"
source /opt/ros/lyrical/setup.bash
source .venv-px4/bin/activate
cd PX4-Autopilot
make px4_sitl_default
