from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    # Launch args
    f_dot_arg  = DeclareLaunchArgument('f_dot', default_value='0.1')

    # Extract R as a string, parse it, apply operations, and convert back
    R_ekf = np.array(
        [
            [1.75e-08, -3.22e-09, -7.38e-11, 0.00e+00, 0.00e+00, 0.00e+00],
            [-3.22e-09, 1.27e-08, -1.12e-10, 0.00e+00, 0.00e+00, 0.00e+00],
            [-7.38e-11, -1.12e-10, 3.24e-09, 0.00e+00, 0.00e+00, 0.00e+00],
            [0.00e+00, 0.00e+00, 0.00e+00, 1.05e-06, 3.08e-07, 1.12e-06],
            [0.00e+00, 0.00e+00, 0.00e+00, 3.08e-07, 2.02e-06, 4.82e-06],
            [0.00e+00, 0.00e+00, 0.00e+00, 1.12e-06, 4.82e-06, 1.17e-05],
        ]
    )
    R_ekf[3:6, 3:6] *= 5e4
    R_ekf *= 1.5
    R_default = '[' + ', '.join([f'{val:.2e}' for val in R_ekf.flatten()]) + ']'
    R_arg = DeclareLaunchArgument('R', default_value=R_default)    
    
    f_dot = LaunchConfiguration('f_dot')
    R = LaunchConfiguration('R')

    pinn_ekf_node = Node(
        package='robot',
        executable='ekf_node',
        name='ekf_node',
        # output='screen',
        prefix=['taskset -c 7'],
        parameters=[{
            'f_dot': f_dot,
            'R': R
        }]
    )

    ld = LaunchDescription()
    ld.add_action(f_dot_arg)
    ld.add_action(R_arg)

    ld.add_action(pinn_ekf_node)

    return ld
 