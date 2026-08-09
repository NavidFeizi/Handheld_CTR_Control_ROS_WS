import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_YAML = 'config/mpc_params.yaml'


def _parameters(context, params_file, mapping):
    """YAML first; only launch arguments actually typed on the CLI override it.

    mapping: {parameter_name: launch_argument_name}. An argument left at its
    empty default is dropped, so the YAML value stands.

    The value stays a ParameterValue substitution rather than the performed
    string: launch_ros infers the parameter type from the substitution result,
    so `Q:=1234.0` reaches rclcpp as a double and `q0:='[...]'` as a double
    array. Storing the plain string writes `Q: '1234.0'` and rclcpp rejects it.
    """
    overrides = {}
    for param, arg in mapping.items():
        config = LaunchConfiguration(arg)
        if config.perform(context) != '':
            overrides[param] = ParameterValue(config)
    return [params_file, overrides] if overrides else [params_file]


def _launch_setup(context, *args, **kwargs):
    params_file = os.path.join(get_package_share_directory('mpc'), 'config', 'mpc_params.yaml')

    mpc_node = Node(
        package='mpc',
        executable='mpc',
        name='mpc',
        output='screen',
        prefix=['taskset -c 4'],
        parameters=_parameters(context, params_file, {
            'sample_time': 'sample_time',
            'model_name': 'model_name',
            'q0': 'q0',
            'u_max': 'u_max',
            'u_dot_max': 'u_dot_max',
            'q_scale': 'q_scale',
            'R_du': 'R_du',
            'R_u': 'R_u',
            'Q': 'Q',
            'error_c': 'error_c',
        }),
    )

    return [mpc_node]


def generate_launch_description():
    # Canonical parameter values live in config/mpc_params.yaml (handheld CTR
    # values). The launch arguments below default to empty and only override the
    # YAML when passed explicitly on the command line. To run the Grassmann CTR
    # instead, point them at that model, e.g.
    #   ros2 launch mpc launch.py model_name:=grassmann_ctr_v4.4.4 \
    #       q0:='[-0.100, -0.055, -0.005, 0.0, 0.0, 0.0]'
    return LaunchDescription([
        DeclareLaunchArgument(
            'sample_time', default_value='',
            description=f'seconds; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'model_name', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'q0', default_value='',
            description=f'SI units; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'u_max', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'u_dot_max', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'q_scale', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'R_u', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'R_du', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'Q', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'error_c', default_value='',
            description=f'unset -> {_YAML}'),
        OpaqueFunction(function=_launch_setup),
    ])
