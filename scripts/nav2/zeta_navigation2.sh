#!/usr/bin/env bash

# ROS 2 환경 설정
source /root/ws/install/setup.bash

# 토픽의 메시지 발행을 체크하는 함수
check_topic() {
    local topic=$1
    local timeout=$2
    local end=$((SECONDS+timeout))

    while [ $SECONDS -lt $end ]; do
        # 토픽의 메시지 발행 빈도 체크 (1초 동안 체크)
        if ros2 topic hz $topic --window 1 2>&1 | grep -q 'average rate'; then
            return 0
        fi
        sleep 1
    done
    return 1
}

echo "Checking for /filtered_scan topic messages..."

# /filtered_scan 토픽 체크
if ! check_topic "/filtered_scan" 30; then
    echo "No messages on /filtered_scan topic within the timeout period."
    exit 1
fi

echo "Both /filtered_scan topic is active. Launching zeta_navigation2..."
source /opt/ros/jazzy/setup.bash
ros2 launch zeta_navigation2 zeta_navigation2.launch.py