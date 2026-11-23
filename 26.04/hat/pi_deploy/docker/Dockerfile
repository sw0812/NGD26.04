# 빌드 컨텍스트 = 저장소 루트(NGD26.04):  docker build -f 26.04/hat/pi_deploy/docker/Dockerfile \
#   --build-arg PX4_MSGS_BRANCH=release/1.16 -t kasa-ros .
FROM ros:jazzy-ros-base
ARG PX4_MSGS_BRANCH
RUN test -n "$PX4_MSGS_BRANCH" || (echo "PX4_MSGS_BRANCH build-arg 필요" && false)
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-colcon-common-extensions python3-rosdep python3-opencv python3-numpy \
      ros-jazzy-cv-bridge ros-jazzy-usb-cam ros-jazzy-tf2-ros ros-jazzy-visualization-msgs v4l-utils \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /kasa_ws/src
RUN git clone --depth 1 -b "$PX4_MSGS_BRANCH" https://github.com/PX4/px4_msgs.git \
 && git clone --depth 1 https://github.com/fictionlab/ros2_aruco_opencv.git aruco_opencv
COPY 26.04/KASA_SITL/01_ros2_ws_2604_MAIN/src/lidar_depth_pkg lidar_depth_pkg
COPY 26.04/hat/kasa_hailo kasa_hailo
WORKDIR /kasa_ws
RUN apt-get update && rosdep update --include-eol-distros 2>/dev/null; \
    . /opt/ros/jazzy/setup.sh && rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "px4_msgs gazebo_msgs" ; \
    . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
COPY 26.04/hat/pi_deploy/run_stack_pi.sh /usr/local/bin/run_stack_pi.sh
ENV WS=/kasa_ws REPO=/NGD26.04 ROS_DISTRO=jazzy
CMD ["/usr/local/bin/run_stack_pi.sh", "mission"]
