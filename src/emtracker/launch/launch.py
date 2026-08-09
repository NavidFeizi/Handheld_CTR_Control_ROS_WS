import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_YAML = 'emtracker_params.yaml'


def _parameters(context, params_file, mapping):
    """YAML first; only launch arguments actually typed on the CLI override it.

    mapping: {parameter_name: launch_argument_name}. An argument left at its
    empty default is dropped, so the YAML value stands.

    The value stays a ParameterValue substitution rather than the performed
    string: launch_ros infers the parameter type from the substitution result,
    so `Kp:=99.0` reaches rclcpp as a double. Storing the plain string instead
    writes `Kp: '99.0'` into the generated params file and rclcpp rejects it.
    """
    overrides = {}
    for param, arg in mapping.items():
        config = LaunchConfiguration(arg)
        if config.perform(context) != '':
            overrides[param] = ParameterValue(config)
    return [params_file, overrides] if overrides else [params_file]


def _launch_setup(context, *args, **kwargs):
    params_file = os.path.join(get_package_share_directory('emtracker'), _YAML)

    emt_node = Node(
        package='emtracker',
        executable='track',
        name='emt_node',
        output='screen',
        prefix=['taskset -c 10'],
        # arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=_parameters(context, params_file, {
            'host_name': 'host_name',
            'send_on_igtl': 'send_on_igtl',
            'enable_position_logging': 'enable_position_logging',
        }),
    )

    return [emt_node]


def generate_launch_description():
    # Canonical parameter values live in config/emtracker_params.yaml. The
    # launch arguments below default to empty and only override the YAML when
    # passed explicitly on the command line.
    return LaunchDescription([
        DeclareLaunchArgument(
            'host_name', default_value='',
            description=f'NDI Aurora serial device; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'send_on_igtl', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'enable_position_logging', default_value='',
            description=f'unset -> {_YAML}'),
        OpaqueFunction(function=_launch_setup),
    ])
