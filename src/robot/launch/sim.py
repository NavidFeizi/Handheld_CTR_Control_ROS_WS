import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_YAML = 'config/robot_params.yaml'


def _parameters(context, params_file, mapping):
    """YAML first; only launch arguments actually typed on the CLI override it.

    mapping: {parameter_name: launch_argument_name}. An argument left at its
    empty default is dropped, so the YAML value stands.

    The value stays a ParameterValue substitution rather than the performed
    string: launch_ros infers the parameter type from the substitution result,
    so `f_dot:=0.5` reaches rclcpp as a double. Storing the plain string instead
    writes `f_dot: '0.5'` into the generated params file and rclcpp rejects it.
    """
    overrides = {}
    for param, arg in mapping.items():
        config = LaunchConfiguration(arg)
        if config.perform(context) != '':
            overrides[param] = ParameterValue(config)
    return [params_file, overrides] if overrides else [params_file]


def _launch_setup(context, *args, **kwargs):
    params_file = os.path.join(get_package_share_directory('robot'), 'config', 'robot_params.yaml')

    pinn_fk_node = Node(
        package='robot',
        executable='pinn_fk',
        name='robot_sim_node',
        # output='screen',
        prefix=['taskset -c 7'],
    )

    pinn_ekf_node = Node(
        package='robot',
        executable='ekf_node',
        name='ekf_node',
        # output='screen',
        prefix=['taskset -c 7'],
        parameters=_parameters(context, params_file, {
            'f_dot': 'f_dot',
        }),
    )

    return [pinn_fk_node, pinn_ekf_node]


def generate_launch_description():
    # R and the other EKF parameters come from config/robot_params.yaml, which
    # is authoritative; the launch arguments below only override it when passed
    # explicitly. Kp/Ki/maxVel/maxAcc are accepted for command-line parity with
    # robot.py but reach no node here — this file starts no robot_node.
    return LaunchDescription([
        DeclareLaunchArgument('Kp', default_value='', description='unused in sim.py'),
        DeclareLaunchArgument('Ki', default_value='', description='unused in sim.py'),
        DeclareLaunchArgument('maxVel', default_value='', description='unused in sim.py'),
        DeclareLaunchArgument('maxAcc', default_value='', description='unused in sim.py'),
        DeclareLaunchArgument(
            'f_dot', default_value='',
            description=f'N/s; unset -> {_YAML}'),
        OpaqueFunction(function=_launch_setup),
    ])
