import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # R and the other EKF parameters come from config/robot_params.yaml.
    params_file = os.path.join(get_package_share_directory('robot'), 'config', 'robot_params.yaml')

    # Launch args
    kp_arg      = DeclareLaunchArgument('Kp', default_value='30.0')
    ki_arg      = DeclareLaunchArgument('Ki', default_value='5.0')
    maxvel_arg  = DeclareLaunchArgument('maxVel', default_value='[3.0, 0.012, 3.0, 0.012]') # SI units: [rad/s, m/s, rad/s, m/s]
    maxacc_arg  = DeclareLaunchArgument('maxAcc', default_value='[10.0, 0.10, 10.0, 0.10]') # SI units: [rad/s^2, m/s^2, rad/s^2, m/s^2]

    # EKF parameters
    f_dot_arg  = DeclareLaunchArgument('f_dot', default_value='1.0')
    f_dot = LaunchConfiguration('f_dot')

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
        parameters=[params_file, {
            'f_dot': f_dot,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(kp_arg)
    ld.add_action(ki_arg)
    ld.add_action(maxvel_arg)
    ld.add_action(maxacc_arg)
    ld.add_action(f_dot_arg)

    ld.add_action(pinn_fk_node)
    ld.add_action(pinn_ekf_node)

    return ld
