import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    params_file = os.path.join(get_package_share_directory('robot'), 'config', 'robot_params.yaml')

    robot_node = Node(
        package='robot',
        executable='ctr_robot',
        name='robot_node',
        output='screen',
        prefix=['taskset -c 5'],
        parameters=_parameters(context, params_file, {
            'Kp': 'Kp',
            'Ki': 'Ki',
            'maxVel': 'maxVel',
            'maxAcc': 'maxAcc',
            'boot_max_attempts': 'boot_max_attempts',
            'boot_timeout_s': 'boot_timeout_s',
        }),
    )

    gui_node = Node(
        package='robot',
        executable='qt_gui',
        name='gui_node',
        output='screen',
        prefix=['taskset -c 6'],
    )

    cosserat_fk_node = Node(
        package='robot',
        executable='cosserat_fk',
        name='cosserat_fk_node',
        # output='screen',
        prefix=['taskset -c 7'],
        parameters=[params_file],
    )

    pinn_fk_node = Node(
        package='robot',
        executable='pinn_fk',
        name='pinn_fk_node',
        # output='screen',
        prefix=['taskset -c 7'],
        parameters=[params_file],
    )

    # Core 3 is unused system-wide (2: manager, 4: mpc, 5: robot, 6: gui,
    # 7: pinn_fk, 8: igtl, 9: planner, 10: emtracker) — the EKF used to share
    # core 7 with pinn_fk, starving both Torch inference loops.
    pinn_ekf_node = Node(
        package='robot',
        executable='ekf_node',
        name='ekf_node',
        # output='screen',
        prefix=['taskset -c 3'],
        parameters=_parameters(context, params_file, {
            'f_dot': 'f_dot',
        }),
    )

    delay_gui_node = TimerAction(period=5.0, actions=[gui_node])

    return [
        robot_node,
        delay_gui_node,
        # cosserat_fk_node,
        pinn_fk_node,
        pinn_ekf_node,
    ]


def generate_launch_description():
    # Canonical parameter values live in config/robot_params.yaml. The launch
    # arguments below default to empty and only override the YAML when passed
    # explicitly on the command line.
    return LaunchDescription([
        DeclareLaunchArgument(
            'Kp', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'Ki', default_value='',
            description=f'unset -> {_YAML}'),
        DeclareLaunchArgument(
            'maxVel', default_value='',
            description=f'[rad/s, m/s, rad/s, m/s]; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'maxAcc', default_value='',
            description=f'[rad/s^2, m/s^2, rad/s^2, m/s^2]; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'boot_max_attempts', default_value='',
            description=f'NMT reset retries before giving up; unset -> {_YAML}'),
        DeclareLaunchArgument(
            'boot_timeout_s', default_value='',
            description=f'seconds to wait per boot attempt; unset -> {_YAML}'),
        # EKF parameters (R lives in the YAML only)
        DeclareLaunchArgument(
            'f_dot', default_value='',
            description=f'N/s; unset -> {_YAML}'),
        OpaqueFunction(function=_launch_setup),
    ])
