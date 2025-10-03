# Dockerfile
FROM nav2-base

RUN apt-get update && \
apt-get install -y python3-pip iputils-ping ros-$ROS_DISTRO-tf-transformations && \
pip install --break-system-packages requests psutil pytz geographiclib supabase fastapi uvicorn websocket-client flask-socketio pyproj



WORKDIR /root/3rdparty
RUN git clone https://github.com/gabrielsr/standalone-smach.git standalone-smach && \
    sed -i '247s/isAlive()/is_alive()/g' standalone-smach/smach/concurrence.py && \
    cd standalone-smach && \
    python3 setup.py install

# source 복사
WORKDIR /root/ws/src
COPY zeta_navigation2 /root/ws/src/zeta_navigation2
COPY fsm_waypoint fsm_waypoint
COPY accessory_msgs accessory_msgs
COPY accessory_client accessory_client
COPY image_action_msgs image_action_msgs
COPY navigation2_tutorials/nav2_straightline_planner nav2_straightline_planner

WORKDIR /root/ws/src
RUN git clone https://github.com/SteveMacenski/nonpersistent_voxel_layer.git -b jazzy

WORKDIR /root/ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select \
    fsm_waypoint \
    zeta_navigation2 \
    accessory_msgs \
    accessory_client \
    capture_image_msgs \
    nav2_straightline_planner \
    nonpersistent_voxel_layer

CMD source /opt/ros/$ROS_DISTRO/setup.sh && /bin/bash

# 환경 설정
RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> ~/.bashrc && \
    echo "source /root/ws/install/setup.bash" >> ~/.bashrc
