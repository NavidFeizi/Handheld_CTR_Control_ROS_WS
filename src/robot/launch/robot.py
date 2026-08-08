import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Canonical parameter values live in config/robot_params.yaml; the launch
    # arguments below (same names and defaults as before) override them.
    params_file = os.path.join(get_package_share_directory('robot'), 'config', 'robot_params.yaml')

    kp_arg      = DeclareLaunchArgument('Kp', default_value='30.0')
    ki_arg      = DeclareLaunchArgument('Ki', default_value='5.0')
    maxvel_arg  = DeclareLaunchArgument('maxVel', default_value='[3.0, 0.012, 3.0, 0.012]') # SI units: [rad/s, m/s, rad/s, m/s]
    maxacc_arg  = DeclareLaunchArgument('maxAcc', default_value='[10.0, 0.10, 10.0, 0.10]') # SI units: [rad/s^2, m/s^2, rad/s^2, m/s^2]

    # EKF parameters (R lives in the YAML only)
    f_dot_arg   = DeclareLaunchArgument('f_dot', default_value='0.2')

    Kp      = LaunchConfiguration('Kp')
    Ki      = LaunchConfiguration('Ki')
    MaxVel  = LaunchConfiguration('maxVel')
    MaxAcc  = LaunchConfiguration('maxAcc')
    f_dot   = LaunchConfiguration('f_dot')

    robot_node = Node(
        package='robot',
        executable='ctr_robot',
        name='robot_node',
        output='screen',
        prefix=['taskset -c 5'],
        parameters=[params_file, {
            'Kp': Kp,
            'Ki': Ki,
            'maxVel': MaxVel,
            'maxAcc': MaxAcc,
        }],
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
        parameters=[params_file, {
            'f_dot': f_dot,
        }]
    )

    delay_gui_node = TimerAction(period=5.0, actions=[gui_node])

    ld = LaunchDescription()
    ld.add_action(kp_arg)
    ld.add_action(ki_arg)
    ld.add_action(maxvel_arg)
    ld.add_action(maxacc_arg)
    ld.add_action(f_dot_arg)

    ld.add_action(robot_node)
    ld.add_action(delay_gui_node)
    # ld.add_action(cosserat_fk_node)
    ld.add_action(pinn_fk_node)
    ld.add_action(pinn_ekf_node)

    return ld
