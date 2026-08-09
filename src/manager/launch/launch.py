import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_YAML = 'config/manager_params.yaml'


def _parameters(context, params_file, mapping):
    """YAML first; only launch arguments actually typed on the CLI override it.

    mapping: {parameter_name: launch_argument_name}. An argument left at its
    empty default is dropped, so the YAML value stands.

    The value stays a ParameterValue substitution rather than the performed
    string: launch_ros infers the parameter type from the substitution result,
    so `recorder_sample_time:=0.01` reaches rclcpp as a double. Storing the plain
    string writes `sample_time: '0.01'` instead and rclcpp rejects it.
    """
    overrides = {}
    for param, arg in mapping.items():
        config = LaunchConfiguration(arg)
        if config.perform(context) != '':
            overrides[param] = ParameterValue(config)
    return [params_file, overrides] if overrides else [params_file]


def _launch_setup(context, *args, **kwargs):
    params_file = os.path.join(get_package_share_directory('manager'), 'config', 'manager_params.yaml')

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
        parameters=_parameters(context, params_file, {
            'sample_time': 'recorder_sample_time',
        }),
    )

    return [recorder_node, master_node]


def generate_launch_description():
    # Canonical parameter values live in config/manager_params.yaml. The launch
    # argument below defaults to empty and only overrides the YAML when passed
    # explicitly on the command line.
    return LaunchDescription([
        DeclareLaunchArgument(
            'recorder_sample_time', default_value='',
            description=f'recorder_node.sample_time [s]; unset -> {_YAML}'),
        OpaqueFunction(function=_launch_setup),
    ])
