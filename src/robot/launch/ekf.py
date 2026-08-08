import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # R and the other EKF parameters come from config/robot_params.yaml.
    params_file = os.path.join(get_package_share_directory('robot'), 'config', 'robot_params.yaml')

    f_dot_arg = DeclareLaunchArgument('f_dot', default_value='0.1')
    f_dot = LaunchConfiguration('f_dot')

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

    ld = LaunchDescription()
    ld.add_action(f_dot_arg)
    ld.add_action(pinn_ekf_node)

    return ld
