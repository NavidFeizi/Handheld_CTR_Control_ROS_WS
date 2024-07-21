from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()
    
    controller_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '0',  # This sets the affinity to CPU core 0
            # 'gnome-terminal', '--',
            'ros2', 'run', 'controller', 'control',
            '--ros-args',
            '-p', 'control_sample_time:=0.004',     # [s]
            '-p', 'feedback_sample_time:=0.015',    # [s]
            '--remap', '__node:=control_node'
        ],
        output='screen',
    )

    ld.add_action(controller_node)

    return ld
