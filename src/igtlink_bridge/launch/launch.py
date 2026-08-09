import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_YAML = 'config/igtlink_params.yaml'


def _parameters(context, params_file, mapping):
    """YAML first; only launch arguments actually typed on the CLI override it.

    mapping: {parameter_name: launch_argument_name}. An argument left at its
    empty default is dropped, so the YAML value stands.

    The value stays a ParameterValue substitution rather than the performed
    string: launch_ros infers the parameter type from the substitution result,
    so `port_slicer:=18950` reaches rclcpp as an integer. Storing the plain
    string writes `port: '18950'` instead and rclcpp rejects it.
    """
    overrides = {}
    for param, arg in mapping.items():
        config = LaunchConfiguration(arg)
        if config.perform(context) != '':
            overrides[param] = ParameterValue(config)
    return [params_file, overrides] if overrides else [params_file]


def _launch_setup(context, *args, **kwargs):
    params_file = os.path.join(get_package_share_directory('igtlink_bridge'), 'config', 'igtlink_params.yaml')

    module_node = Node(
        package='igtlink_bridge',
        executable='bridge',
        name='igtlink_bridge_module_node',
        output='screen',
        prefix=['taskset -c 8'],
        parameters=_parameters(context, params_file, {
            'hostname': 'hostname_module',
            'port': 'port_module',
            'auto_connect': 'auto_connect',
        }),
    )

    slicer_node = Node(
        package='igtlink_bridge',
        executable='bridge',
        name='igtlink_bridge_slicer_node',
        # output='screen',
        prefix=['taskset -c 8'],
        parameters=_parameters(context, params_file, {
            'hostname': 'hostname_slicer',
            'port': 'port_slicer',
            'auto_connect': 'auto_connect',
        }),
    )

    return [
        # module_node,
        slicer_node,
    ]


def generate_launch_description():
    # Canonical parameter values live in config/igtlink_params.yaml, one block
    # per node. The launch arguments below default to empty and only override
    # the YAML when passed explicitly on the command line.
    return LaunchDescription([
        DeclareLaunchArgument(
            'hostname_module', default_value='',
            description=f'igtlink_bridge_module_node.hostname; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'port_module', default_value='',
            description=f'igtlink_bridge_module_node.port; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'hostname_slicer', default_value='',
            description=f'igtlink_bridge_slicer_node.hostname; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'port_slicer', default_value='',
            description=f'igtlink_bridge_slicer_node.port; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'auto_connect', default_value='',
            description=f'applies to both nodes; unset -> {_YAML}'),
        OpaqueFunction(function=_launch_setup),
    ])
