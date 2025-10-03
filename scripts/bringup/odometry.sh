#!/usr/bin/env bash
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

echo "Checking for /imu/data_raw topic messages..."

# /odometry 토픽 체크
if ! check_topic "/imu/data_raw" 30; then
    echo "No messages on /imu/data_raw topic within the timeout period."
    exit 1
fi

echo "/imu/data_raw topic is active. Launching z8015_launch..."

ros2 launch z8015_mobile_bringup z8015_launch.py