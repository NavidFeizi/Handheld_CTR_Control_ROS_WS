import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Canonical parameter values live in config/igtlink_params.yaml; the
    # launch arguments below override them.
    params_file = os.path.join(get_package_share_directory('igtlink_bridge'), 'config', 'igtlink_params.yaml')

    # Launch args
    hostname_module_arg = DeclareLaunchArgument('hostname_module', default_value='10.15.232.114')
    port_module_arg = DeclareLaunchArgument('port_module', default_value='18975')
    hostname_slicer_arg = DeclareLaunchArgument('hostname_slicer', default_value='localhost')
    port_slicer_arg = DeclareLaunchArgument('port_slicer', default_value='18944')
    auto_connect_arg = DeclareLaunchArgument('auto_connect', default_value='false')

    hostname_module = LaunchConfiguration('hostname_module')
    port_module = LaunchConfiguration('port_module')
    hostname_slicer = LaunchConfiguration('hostname_slicer')
    port_slicer = LaunchConfiguration('port_slicer')
    auto_connect = LaunchConfiguration('auto_connect')

    module_node = Node(
        package='igtlink_bridge',
        executable='bridge',
        name='igtlink_bridge_module_node',
        output='screen',
        prefix=['taskset -c 8'],
        parameters=[params_file, {
            'hostname': hostname_module,
            'port': port_module,
            'auto_connect': auto_connect,
        }],
    )

    slicer_node = Node(
        package='igtlink_bridge',
        executable='bridge',
        name='igtlink_bridge_slicer_node',
        # output='screen',
        prefix=['taskset -c 8'],
        parameters=[params_file, {
            'hostname': hostname_slicer,
            'port': port_slicer,
            'auto_connect': auto_connect,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(hostname_module_arg)
    ld.add_action(port_module_arg)
    ld.add_action(hostname_slicer_arg)
    ld.add_action(port_slicer_arg)
    ld.add_action(auto_connect_arg)

    # ld.add_action(module_node)
    ld.add_action(slicer_node)

    return ld
