#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class MappingLidarMultiAngleRangeFilter(Node):
    def __init__(self):
        super().__init__('mapping_lidar_multi_angle_range_filter')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/filtered_scan', 10)

        # 필터 설정 (각도 범위별 서로 다른 거리 제한)
        self.angle_ranges = [
            # (-2.11, -1.57, 10.0, 40.0),   # 60~90도: 10.0m 이내 제거, 200.0m 초과 유지
            (-1.57, -1.03, 0.50, 40.0),   # 90~150도: 0.35m 이내 제거, 200.0m 초과 유지
            (-1.03, 1.03, 0.80, 40.0),   # 150~210도: 0.45m 이내 제거, 200.0m 초과 유지
            (1.03, 1.57, 0.55, 40.0)    # 210~270도: 0.35m 이내 제거, 200.0m 초과 유지
        ]

    def scan_callback(self, msg):
        filtered_ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment

        # angles 배열의 크기를 msg.ranges의 길이에 맞춤
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        for lower_angle, upper_angle, min_range, max_range in self.angle_ranges:
            mask = (angles >= lower_angle) & (angles <= upper_angle)

            # mask 크기를 filtered_ranges와 일치시키기 위해 인덱싱 사용
            range_mask = (filtered_ranges[mask] < min_range) | (filtered_ranges[mask] > max_range)
            filtered_ranges[mask] = np.where(range_mask, float('inf'), filtered_ranges[mask])

        msg.ranges = filtered_ranges.tolist()
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MappingLidarMultiAngleRangeFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
