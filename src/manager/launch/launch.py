from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch args
    # sample_time_arg  = DeclareLaunchArgument('sample_time', default_value='.001')
    recorder_sample_time_arg  = DeclareLaunchArgument('recorder_sample_time', default_value='.025')

    # sample_time      = LaunchConfiguration('sample_time')
    recorder_sample_time = LaunchConfiguration('recorder_sample_time')

    manager_node = Node(
        package='manager',
        executable='manage',
        name='manager_node',
        output='screen',
        prefix=['taskset -c 2'], 
        # parameters=[{
        #     'sample_time': sample_time,
        # }],
    )

    procedure_node = Node(
        package='manager',
        executable='procedure',
        name='procedure_node',
        output='screen',
        prefix=['taskset -c 2'], 
    )

    joint_path_node = Node(
        package='manager',
        executable='joint_path',
        name='jointPath_node',
        output='screen',
        prefix=['taskset -c 2'], 
    )    

    master_node = Node(
        package='manager',
        executable='master',
        name='master_node',
        output='screen',
        prefix=['taskset -c 2'], 
    )
    
    recorder_node = Node(
        package='manager',
        executable='record',
        name='recorder_node',
        output='screen',
        prefix=['taskset -c 2'], 
        parameters=[{
            'sample_time': recorder_sample_time,
        }],
    )

    ld = LaunchDescription()
    # ld.add_action(sample_time_arg)
    ld.add_action(recorder_sample_time_arg)

    # Delay the start of the observer_node by 5 seconds
    delay_manager_node = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[manager_node],
    )

    # ld.add_action(delay_manager_node)
    ld.add_action(recorder_node)
    # ld.add_action(procedure_node)
    # ld.add_action(dataset_node)
    # ld.add_action(joint_path_node)
    ld.add_action(master_node)

    return ld
