import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM')
]

def generate_launch_description():
    pkg_slambox = get_package_share_directory('z8015_slambox')

    sync = LaunchConfiguration('sync')

    slam_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_slambox, 'config', 'slam.yaml']),
        description='SLAM config file path')

    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration('params'),
        param_rewrites={},
        convert_types=True
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/filtered_scan', 'filtered_scan'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
    ]

    slam = GroupAction([
        Node(package='slam_toolbox',
             executable='sync_slam_toolbox_node',
             name='sync_slam_toolbox',
             output='screen',
             parameters=[
                 slam_params,
                 {'use_sim_time': LaunchConfiguration('use_sim_time')}
             ],
             remappings=remappings,
             condition=IfCondition(sync)),

        Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='async_slam_toolbox',
             output='screen',
             parameters=[
                 slam_params,
                 {'use_sim_time': LaunchConfiguration('use_sim_time')}
             ],
             remappings=remappings,
             condition=UnlessCondition(sync))
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam_params_arg)
    ld.add_action(slam)
    return ld
