from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch args
    recorder_sample_time_arg  = DeclareLaunchArgument('recorder_sample_time', default_value='.025')

    recorder_sample_time = LaunchConfiguration('recorder_sample_time')

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
    ld.add_action(recorder_sample_time_arg)

    ld.add_action(recorder_node)
    ld.add_action(master_node)

    return ld
