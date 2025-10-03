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

echo "Checking for and /scan topic messages..."

# /scan 토픽 체크
if ! check_topic "/scan" 30; then
    echo "No messages on /scan topic within the timeout period."
    exit 1
fi

echo "/scan topics is active. Launching lidar_filter..."

ros2 run lidar_filter lidar_filter
