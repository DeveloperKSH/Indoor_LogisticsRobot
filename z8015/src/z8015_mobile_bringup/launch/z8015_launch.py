# my_robot_launch.py
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # pkg_share = launch_ros.substitutions.FindPackageShare(package='simple_description').find('simple_description')
    # default_model_path = os.path.join(pkg_share, 'src/description/simple_description.urdf')

    z8015_share_dir = get_package_share_directory('z8015_mobile_bringup')
    config_locks_path = os.path.join(z8015_share_dir, 'config', 'twist_mux_locks.yaml')
    config_topics_path = os.path.join(z8015_share_dir, 'config', 'twist_mux_topics.yaml')
    config_ps3_path = os.path.join(z8015_share_dir, 'config', 'ps3.config.yaml')

    twist_mux_node = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('cmd_vel_out', '/twist_mux/cmd_vel')},
        parameters=[
            {'use_sim_time': False},
            {'use_stamped': False},
            config_locks_path,
            config_topics_path],
    )

    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )

    z8015_driver_node = launch_ros.actions.Node(
        package='z8015_mobile_bringup',
        executable='z8015_driver',
        name='z8015_driver',
        remappings=[('cmd_vel', '/twist_mux/cmd_vel')],  # 리스트 형식으로 변경
    )
    teleop_tiwst_joy = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_ps3_path],
        remappings=[('cmd_vel', '/teleop_node/cmd_vel')],
    )

    teleop_twist_keyboard = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node',
        # prefix='xterm -e',
        remappings=[
            ('cmd_vel', '/teleop_node/cmd_vel'),
        ],
    )
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard -r cmd_vel:=/teleop_node/cmd_vel

    return launch.LaunchDescription([
        joy_node,
        teleop_tiwst_joy,
        z8015_driver_node,
        twist_mux_node
    ])
