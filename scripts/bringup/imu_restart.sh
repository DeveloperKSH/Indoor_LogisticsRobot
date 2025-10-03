#!/bin/bash
source /root/ws/install/setup.bash

# /witmotion 노드가 실행 중인지 확인하는 함수
check_witmotion_node() {
    local node_name="/witmotion"
    local node_list=$(ros2 node list)  # 노드 목록을 변수에 저장
    echo "$node_list" | grep -q $node_name  # 변수 내용을 grep
    if [ $? -eq 0 ]; then
        # echo "$node_name node is running."/rmf_patrol_request
        return 1
    else
        echo "$node_name node is not running."
        # 여기에 /witmotion 노드를 재시작하는 명령을 추가할 수 있습니다.
        # 예: pm2 restart witmotion
        return 0
    fi
}

# 토픽의 데이터 빈도를 체크하는 함수
check_topic() {
    local topic=$1
    local timeout=$2
    local end=$((SECONDS+timeout))

    while [ $SECONDS -lt $end ]; do
        # ros2 topic hz 명령을 timeout과 함께 실행
        if ! timeout $timeout ros2 topic hz $topic --window $timeout 2>&1 | grep -q 'average rate'; then
            echo "No data received on $topic for $timeout seconds, restarting imu..."
            return 1
        fi
        sleep 1

    done
    echo "Data received on $topic for $timeout seconds."
    return 0
}

# /imu/data_raw 토픽 체크 후 조치 실행
check_and_restart() {
    local topic="/imu/data_raw"
    local timeout=10

    echo "Checking the $topic topic..."
    if ! check_topic $topic $timeout; then
        echo "Restarting imu process..."
        pm2 restart imu
    fi
}

# 메인 루프
while true; do
    check_witmotion_node
    if [ $? -eq 0 ]; then
        check_and_restart
    fi
    sleep 30  # 30sec 대기

done
