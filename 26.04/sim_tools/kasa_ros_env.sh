# 2026-11-12: 연구실 같은 와이파이에서 다른 사람 PX4 SITL(ROS 도메인 0)이 돌아 setpoint/위치가 섞임 -> 우리 시뮬 분리.
# PX4 SITL rcS가 ROS_DOMAIN_ID를 보고 UXRCE_DDS_DOM_ID를 자동으로 맞춤 (ROMFS/px4fmu_common/init.d-posix/rcS:312).
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
