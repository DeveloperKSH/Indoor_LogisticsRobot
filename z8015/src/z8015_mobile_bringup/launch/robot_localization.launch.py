# my_robot_launch.py
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    a012_share_dir = get_package_share_directory('a012_mobile_bringup')
    config_robot_localization_path = os.path.join(a012_share_dir, 'config', 'robot_localization.yaml')
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters = [config_robot_localization_path, {"use_sim_time": False}],
    )
    return launch.LaunchDescription([
        robot_localization_node,
    ])
