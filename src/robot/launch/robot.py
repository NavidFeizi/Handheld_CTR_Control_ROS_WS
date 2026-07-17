from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    # Launch args
    kp_arg      = DeclareLaunchArgument('Kp', default_value='30.0')
    ki_arg      = DeclareLaunchArgument('Ki', default_value='5.0')
    maxvel_arg  = DeclareLaunchArgument('maxVel', default_value='[3.0, 0.012, 3.0, 0.012]') # SI units: [rad/s, m/s, rad/s, m/s]
    maxacc_arg  = DeclareLaunchArgument('maxAcc', default_value='[10.0, 0.10, 10.0, 0.10]') # SI units: [rad/s^2, m/s^2, rad/s^2, m/s^2]

    Kp      = LaunchConfiguration('Kp')
    Ki      = LaunchConfiguration('Ki')
    MaxVel  = LaunchConfiguration('maxVel')
    MaxAcc  = LaunchConfiguration('maxAcc')
    
    # EKF parameters
    f_dot_arg  = DeclareLaunchArgument('f_dot', default_value='0.1')
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

    robot_node = Node(
        package='robot',
        executable='ctr_robot',
        name='robot_node',
        output='screen',
        prefix=['taskset -c 5'], 
        parameters=[{
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
    )

    pinn_fk_node = Node(
        package='robot',
        executable='pinn_fk',
        name='pinn_fk_node',
        # output='screen',
        prefix=['taskset -c 7'],
    )

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

    delay_gui_node = TimerAction(period=5.0, actions=[gui_node])

    ld = LaunchDescription()
    ld.add_action(kp_arg)
    ld.add_action(ki_arg)
    ld.add_action(maxvel_arg)
    ld.add_action(maxacc_arg)
    ld.add_action(f_dot_arg)
    ld.add_action(R_arg)

    ld.add_action(robot_node)
    ld.add_action(delay_gui_node)
    # ld.add_action(cosserat_fk_node)
    ld.add_action(pinn_fk_node)
    ld.add_action(pinn_ekf_node)

    return ld
 