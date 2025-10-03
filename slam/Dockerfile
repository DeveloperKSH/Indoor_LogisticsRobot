FROM z8015odroid-jazzy
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 모든 패키지를 한 번에 설치
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-cartographer-ros \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-localization && \
    rm -rf /var/lib/apt/lists/*

# 워크스페이스 설정
WORKDIR /root/ws/src

# Cartographer 설정 복사
COPY cartographer ./cartographer
COPY slambox ./slambox

# 의존성 설치 및 빌드
WORKDIR /root/ws
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# 빌드 후 정리 (선택사항)
# RUN rm -rf /root/ws/build /root/ws/log  # 공간 절약이 필요한 경우만

# 환경 설정
RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> ~/.bashrc && \
    echo "source /root/ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
