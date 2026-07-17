from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # for Handheld CTR MPC
    sample_time_arg = DeclareLaunchArgument('sample_time', default_value='0.025') # seconds
    model_name_arg = DeclareLaunchArgument('model_name', default_value='ctr_8x91_0.18_tanh_9K_9K_50K_v3')
    q0_arg = DeclareLaunchArgument('q0', default_value='[-0.156, -0.072, 0.0, 0.0]') # SI units: [m, m, m, rad, rad, rad]
    u_max_arg = DeclareLaunchArgument('u_max', default_value='[0.012, 0.012, 3.0, 3.0]') 

    du_max_arg = DeclareLaunchArgument('u_dot_max', default_value='[0.03, 0.03, 5.0, 5.0]') 
    q_scale_arg = DeclareLaunchArgument('q_scale', default_value='[0.5, 0.5, 0.2, 0.2]') 
    R_u_arg = DeclareLaunchArgument('R_u', default_value='0.0') 

    R_du_arg = DeclareLaunchArgument('R_du', default_value='0.5') 
    Q_arg = DeclareLaunchArgument('Q', default_value='2000.0') 

    error_c_arg = DeclareLaunchArgument('error_c', default_value='1.0')

    # for Handheld CTR reference trajectory
    radius_base_arg = DeclareLaunchArgument('radius_base', default_value='0.022')
    radius_amp_arg = DeclareLaunchArgument('radius_amp', default_value='0.000')
    z_amp_arg = DeclareLaunchArgument('z_amp', default_value='0.005')
    omega_theta_arg = DeclareLaunchArgument('omega_theta', default_value='0.20')
    omega_radius_arg = DeclareLaunchArgument('omega_radius', default_value='1.2')
    omega_z_arg = DeclareLaunchArgument('omega_z', default_value='0.88')
    p_centre_arg = DeclareLaunchArgument('p_centre', default_value='[0.0, 0.0, 0.125]') 

    # # for Grassmann CTR MPC
    # sample_time_arg = DeclareLaunchArgument('sample_time', default_value='0.025') # seconds
    # model_name_arg = DeclareLaunchArgument('model_name', default_value='grassmann_ctr_v4.4.4')
    # q0_arg = DeclareLaunchArgument('q0', default_value='[-0.100, -0.055, -0.005, 0.0, 0.0, 0.0]') # SI units: [m, m, m, rad, rad, rad]
    # u_max_arg = DeclareLaunchArgument('u_max', default_value='[0.1, 0.1, 0.1, 5.0, 5.0, 5.0]') 
    # du_max_arg = DeclareLaunchArgument('du_max', default_value='[0.02, 0.02, 0.02, 1.0, 1.0, 1.0]') 
    # R_arg = DeclareLaunchArgument('R', default_value='[0.05, 0.05, 0.05, 0.005, 0.005, 0.005]') 
    # Q_arg = DeclareLaunchArgument('Q', default_value='[10000.0, 10000.0, 10000.0]') 

    # # for Grassmann reference trajectory
    # radius_base_arg = DeclareLaunchArgument('radius_base', default_value='0.040')
    # radius_amp_arg = DeclareLaunchArgument('radius_amp', default_value='0.015')
    # z_amp_arg = DeclareLaunchArgument('z_amp', default_value='0.015')
    # omega_theta_arg = DeclareLaunchArgument('omega_theta', default_value='0.25')
    # omega_radius_arg = DeclareLaunchArgument('omega_radius', default_value='2.0')
    # omega_z_arg = DeclareLaunchArgument('omega_z', default_value='1.5')
    # p_centre_arg = DeclareLaunchArgument('p_centre', default_value='[0.0, 0.0, 0.140]') 

    ### =============================================================================== ###

    sample_time  = LaunchConfiguration('sample_time')
    model_name  = LaunchConfiguration('model_name')
    q0  = LaunchConfiguration('q0')
    u_max  = LaunchConfiguration('u_max')
    u_dot_max  = LaunchConfiguration('u_dot_max')
    q_scale  = LaunchConfiguration('q_scale')
    Q  = LaunchConfiguration('Q')
    R_u  = LaunchConfiguration('R_u')
    R_du  = LaunchConfiguration('R_du')
    error_c  = LaunchConfiguration('error_c')

    mpc_node = Node(
        package='mpc',
        executable='mpc',
        name='mpc',
        output='screen',
        prefix=['taskset -c 4'],
        parameters=[{
            'sample_time': sample_time,
            'model_name': model_name,
            'q0': q0,
            'u_max': u_max,
            'u_dot_max': u_dot_max,
            'q_scale': q_scale,
            'R_du': R_du,
            'R_u': R_u,
            'Q': Q,
            'error_c': error_c,
        }],
    )

    omega_theta = LaunchConfiguration('omega_theta')
    omega_radius = LaunchConfiguration('omega_radius')
    omega_z = LaunchConfiguration('omega_z')
    radius_base = LaunchConfiguration('radius_base')
    radius_amp = LaunchConfiguration('radius_amp')
    z_amp = LaunchConfiguration('z_amp')
    p_centre = LaunchConfiguration('p_centre')

    reference_node = Node(
        package='mpc',
        executable='reference',
        name='reference_node',
        output='screen',
        prefix=['taskset -c 5'],
        parameters=[{
            'sample_time': sample_time,
            'omega_theta': omega_theta,
            'omega_radius': omega_radius,
            'omega_z': omega_z,
            'radius_base': radius_base,
            'radius_amp': radius_amp,
            'z_amp': z_amp,
            'p_centre': p_centre,
        }],
    )

    delayed_mpc_node = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[mpc_node],
    )

    ld = LaunchDescription()
    ld.add_action(sample_time_arg)
    ld.add_action(model_name_arg)
    ld.add_action(q0_arg)
    ld.add_action(u_max_arg)
    ld.add_action(du_max_arg)
    ld.add_action(R_u_arg)
    ld.add_action(R_du_arg)
    ld.add_action(q_scale_arg)
    ld.add_action(Q_arg)
    ld.add_action(error_c_arg)

    ld.add_action(omega_theta_arg)
    ld.add_action(omega_radius_arg)
    ld.add_action(omega_z_arg)
    ld.add_action(radius_base_arg)
    ld.add_action(radius_amp_arg)
    ld.add_action(z_amp_arg)
    ld.add_action(p_centre_arg)
    
    ld.add_action(mpc_node)
    # ld.add_action(reference_node)

    return ld
