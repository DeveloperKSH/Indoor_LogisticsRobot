# my_robot_launch.py
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    uron_node = launch_ros.actions.Node(
        package='uron',
        executable='uron_node',
        name='uron',
        remappings=[('cmd_vel', '/uron/cmd_vel')],
    )
    return launch.LaunchDescription([
        uron_node
    ])
