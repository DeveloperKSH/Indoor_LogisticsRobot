# my_robot_launch.py
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration
import math


def generate_launch_description():
    base2ld06 = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2ld06',
        arguments=['--x', '0.25', '--y', '0', '--z', '0.60', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'ld06_link'],
        output='screen'
    )
    base2sick = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2sick',
        arguments=['--x', '0.35', '--y', '0', '--z', '0.20', '--roll', str(math.pi), '--pitch', '0', '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'sick_link'],
        output='screen'
    )
    base2s2 = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2s2',
        arguments=['--x', '0.25', '--y', '0', '--z', '0.10', '--roll', '0', '--pitch', str(math.pi), '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 's2_link'],
        output='screen'
    )
    base2c1 = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2c1',
        arguments=['--x', '0.25', '--y', '0', '--z', '0.10', '--roll', '0', '--pitch', str(math.pi), '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'c1_link'],
        output='screen'
    )
    base2s2e = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2s2e',
        arguments=['--x', '0.25', '--y', '0', '--z', '0.30', '--roll', '0', '--pitch', str(math.pi), '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 's2e_link'],
        output='screen'
    )
    base2imu = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2imu',
        arguments=['--x', '0.0', '--y', '0', '--z', '0.28', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'imu_link'],
        output='screen'
    )

    base2gps = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2gps',
        arguments=['--x', '0.28', '--y', '0', '--z', '0.70', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'gps_link'],
        output='screen'
    )

    base2camera = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2camera',
        arguments=['--x', '0.20', '--y', '0', '--z', '0.32', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'camera_link'],
        output='screen'
    )

    map2odom = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map2odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'map', '--child-frame-id', 'odom'],
        output='screen'
    )

    odom2base = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom2base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'odom', '--child-frame-id', 'base_link'],
        output='screen'
    )
    base2s3 = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base2s3',
        arguments=['--x', '0.17', '--y', '0', '--z', '0.25', '--roll', str(math.pi), '--pitch', str(math.pi), '--yaw', '0', '--frame-id',
                   'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )
    map2odom = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map2odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id',
                   'map', '--child-frame-id', 'odom'],
        output='screen'
    )

    return launch.LaunchDescription([
        # base2sick,
        # base2s2,
        # base2c1,
        # base2s2e,
        # base2ld06,
        base2imu,
        # base2gps,
        # map2odom,  # <= slam으로 대체  <= 다시 dual_ekf_navsat에서 map -> odom
        # odom2base   # <= 삭제 이유는: dual_ekf_navsat에서 odom -> base_link
        base2camera,
        base2s3,
        # map2odom
    ])
