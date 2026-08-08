import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Canonical parameter values live in config/manager_params.yaml; launch
    # arguments (same names, same defaults) override them.
    params_file = os.path.join(get_package_share_directory('manager'), 'config', 'manager_params.yaml')

    recorder_sample_time_arg = DeclareLaunchArgument('recorder_sample_time', default_value='.025')
    recorder_sample_time = LaunchConfiguration('recorder_sample_time')

    master_node = Node(
        package='manager',
        executable='master',
        name='master_node',
        output='screen',
        prefix=['taskset -c 2'],
        parameters=[params_file],
    )

    recorder_node = Node(
        package='manager',
        executable='record',
        name='recorder_node',
        output='screen',
        prefix=['taskset -c 2'],
        parameters=[params_file, {
            'sample_time': recorder_sample_time,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(recorder_sample_time_arg)

    ld.add_action(recorder_node)
    ld.add_action(master_node)

    return ld
