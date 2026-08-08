import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Canonical parameter values live in config/emtracker_params.yaml; the
    # launch arguments below override them.
    params_file = os.path.join(get_package_share_directory('emtracker'), 'emtracker_params.yaml')

    host_name_arg = DeclareLaunchArgument('host_name', default_value='/dev/ttyUSB1')
    send_on_igtl_arg = DeclareLaunchArgument('send_on_igtl', default_value='false')
    enable_position_logging_arg = DeclareLaunchArgument('enable_position_logging', default_value='false')

    host_name = LaunchConfiguration('host_name')
    send_on_igtl = LaunchConfiguration('send_on_igtl')
    enable_position_logging = LaunchConfiguration('enable_position_logging')

    emt_node = Node(
        package='emtracker',
        executable='track',
        name='emt_node',
        output='screen',
        prefix=['taskset -c 10'],
        # arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[params_file, {
            'host_name': host_name,
            'send_on_igtl': send_on_igtl,
            'enable_position_logging': enable_position_logging,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(host_name_arg)
    ld.add_action(send_on_igtl_arg)
    ld.add_action(enable_position_logging_arg)
    ld.add_action(emt_node)

    return ld
