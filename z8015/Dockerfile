FROM z8015odroid-jazzy
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# COPY로 인한 시간 단축
RUN apt-get update && apt-get install -y  \
    ros-$ROS_DISTRO-robot-localization  \
    ros-$ROS_DISTRO-joy  \
    ros-$ROS_DISTRO-teleop-twist-joy  \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-joint-state-publisher  \
    ros-$ROS_DISTRO-rviz2  \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    nlohmann-json3-dev \
    libqt5serialport5-dev \
    ros-$ROS_DISTRO-tf2-geometry-msgs

# install CPPLinuxSerial

WORKDIR /root/3rdparty
RUN git clone https://github.com/gbmhunter/CppLinuxSerial.git \
    && cd CppLinuxSerial && mkdir build && cd build && cmake .. && make && make install

WORKDIR /root/ws/src
# wt905
RUN git clone -b ros2 --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git witmotion_ros \
    && sed -i "3s|ttyUSB3|wt905|g" witmotion_ros/config/wt905.yml \
    && sed -i "9s|topic_name: /imu|topic_name: /imu/data_raw|g" witmotion_ros/config/wt905.yml \
    && sed -i "10s|frame_id: imu|frame_id: imu_link|g" witmotion_ros/config/wt905.yml

# mwahrs 100hz 믿을수가 없음
#RUN git clone https://github.com/roasinc/mw_ahrs_ros2.git \
#    && sed -i '30s/imu\/data/imu\/data_raw/' mw_ahrs_ros2/src/mw_ahrs_driver.cpp \
#    && sed -i '114s/int num = 0;/int num = 12;/' mw_ahrs_ros2/src/mw_ahrs_driver.cpp \
#    && sed -i '32s/imu\/mag/magnetometer/' mw_ahrs_ros2/src/mw_ahrs_driver.cpp \
#    && sed -i 's|"/dev/ttyUSB0"|"/dev/mwahrs"|' mw_ahrs_ros2/launch/mw_ahrs_launch.py \
#    && sed -i 's/"version": "v2"/"version": "v0"/' mw_ahrs_ros2/launch/mw_ahrs_launch.py

# sllidar
RUN git clone https://github.com/Slamtec/sllidar_ros2.git \
    && sed -i '15s|/dev/ttyUSB0|/dev/rplidar|' sllidar_ros2/launch/sllidar_s3_launch.py

## package
COPY src /root/ws/src
RUN cd /root/ws \
    && source /opt/ros/$ROS_DISTRO/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --packages-select serial && colcon build # serial is not used

WORKDIR /root/ws
RUN echo "source /root/ws/install/setup.bash" >> ~/.bashrc
