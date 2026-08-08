from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Robot launch arguments
    kp_arg = DeclareLaunchArgument('Kp', default_value='30.0')
    ki_arg = DeclareLaunchArgument('Ki', default_value='5.0')
    maxvel_arg = DeclareLaunchArgument('maxVel', default_value='[3.0, 0.012, 3.0, 0.012]')
    maxacc_arg = DeclareLaunchArgument('maxAcc', default_value='[10.0, 0.10, 10.0, 0.10]')
    f_dot_arg = DeclareLaunchArgument('f_dot', default_value='0.2')

    host_name_arg = DeclareLaunchArgument('host_name', default_value='/dev/ttyUSB1')
    send_on_igtl_arg = DeclareLaunchArgument('send_on_igtl', default_value='false')
    enable_position_logging_arg = DeclareLaunchArgument('enable_position_logging', default_value='false')

    hostname_module_arg = DeclareLaunchArgument('hostname_module', default_value='10.15.232.114')
    port_module_arg = DeclareLaunchArgument('port_module', default_value='18975')
    hostname_slicer_arg = DeclareLaunchArgument('hostname_slicer', default_value='localhost')
    port_slicer_arg = DeclareLaunchArgument('port_slicer', default_value='18944')
    auto_connect_arg = DeclareLaunchArgument('auto_connect', default_value='false')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot'), 'launch', 'robot.py'])
        ),
        launch_arguments={
            'Kp': LaunchConfiguration('Kp'),
            'Ki': LaunchConfiguration('Ki'),
            'maxVel': LaunchConfiguration('maxVel'),
            'maxAcc': LaunchConfiguration('maxAcc'),
            'f_dot': LaunchConfiguration('f_dot'),
        }.items(),
    )

    emtracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('emtracker'), 'launch', 'launch.py'])
        ),
        launch_arguments={
            'host_name': LaunchConfiguration('host_name'),
            'send_on_igtl': LaunchConfiguration('send_on_igtl'),
            'enable_position_logging': LaunchConfiguration('enable_position_logging'),
        }.items(),
    )

    igtlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('igtlink_bridge'), 'launch', 'launch.py'])
        ),
        launch_arguments={
            'hostname_module': LaunchConfiguration('hostname_module'),
            'port_module': LaunchConfiguration('port_module'),
            'hostname_slicer': LaunchConfiguration('hostname_slicer'),
            'port_slicer': LaunchConfiguration('port_slicer'),
            'auto_connect': LaunchConfiguration('auto_connect'),
        }.items(),
    )
    
    delayed_robot_launch = TimerAction(
        period=14.0,
        actions=[robot_launch],
    )

    delayed_igtlink_launch = TimerAction(
        period=20.0,
        actions=[igtlink_launch],
    )

    ld = LaunchDescription()
    ld.add_action(kp_arg)
    ld.add_action(ki_arg)
    ld.add_action(maxvel_arg)
    ld.add_action(maxacc_arg)
    ld.add_action(f_dot_arg)
    ld.add_action(host_name_arg)
    ld.add_action(send_on_igtl_arg)
    ld.add_action(enable_position_logging_arg)
    ld.add_action(hostname_module_arg)
    ld.add_action(port_module_arg)
    ld.add_action(hostname_slicer_arg)
    ld.add_action(port_slicer_arg)
    ld.add_action(auto_connect_arg)
    ld.add_action(delayed_robot_launch)
    ld.add_action(emtracker_launch)
    ld.add_action(delayed_igtlink_launch)

    return ld
