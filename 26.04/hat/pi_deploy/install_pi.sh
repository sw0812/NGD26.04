#!/usr/bin/env bash
# Raspberry Pi 5 + AI HAT(Hailo-10H) 에서 KASA 비전/미션 스택 설치. 미검증 초안 — 각 단계 출력 확인하며 실행.
# 가정: Ubuntu 24.04 + ROS 2 Jazzy (Pi 용 Ubuntu). 다른 배포판이면 ROS_DISTRO 변경.
# 사용: PX4_MSGS_BRANCH=<펌웨어와 맞는 브랜치> ./install_pi.sh
set -euo pipefail
ROS_DISTRO=${ROS_DISTRO:-jazzy}
WS=${WS:-$HOME/kasa_ws}
REPO=${REPO:-$HOME/NGD26.04}                 # git clone git@github.com:sw0812/NGD26.04.git
SRC=$REPO/26.04/KASA_SITL/01_ros2_ws_2604_MAIN/src

sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep git python3-opencv python3-numpy \
  ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-ros-base

# --- Hailo-10H 런타임 (HailoRT + PCIe 드라이버 + python 바인딩) ---
# Raspberry Pi OS 에서는: sudo apt install dkms hailo-h10-all   (패키지명은 Raspberry Pi 문서로 확인)
# Ubuntu 에서는 Hailo Developer Zone 의 hailort deb/whl(PCIe driver, hailort, python hailo_platform) 설치.
echo ">> Hailo 런타임이 설치되어 있어야 합니다. 확인: hailortcli fw-control identify"
command -v hailortcli >/dev/null && hailortcli fw-control identify || echo "!! hailortcli 없음 — 위 안내대로 설치 후 재실행"
python3 -c "import hailo_platform" 2>/dev/null || echo "!! python hailo_platform 없음 (venv 쓰면 --system-site-packages)"

mkdir -p $WS/src && cd $WS/src
[ -d px4_msgs ] || git clone -b "${PX4_MSGS_BRANCH:?PX4_MSGS_BRANCH 필요(예: release/1.16)}" https://github.com/PX4/px4_msgs.git
[ -d aruco_opencv ] || git clone https://github.com/fictionlab/ros2_aruco_opencv.git aruco_opencv   # aruco_opencv + aruco_opencv_msgs
for p in lidar_depth_pkg; do [ -e $p ] || cp -r $SRC/$p .; done
[ -e kasa_hailo ] || cp -r $REPO/26.04/hat/kasa_hailo .

cd $WS
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init 2>/dev/null || true; rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys "px4_msgs gazebo_msgs"
colcon build --symlink-install
echo "완료. 다음: source $WS/install/setup.bash && ./run_stack_pi.sh"
