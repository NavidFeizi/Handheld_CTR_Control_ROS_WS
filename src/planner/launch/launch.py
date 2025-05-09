from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()
    
    planner_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '9',  # This sets the affinity to CPU core 0
            # 'gnome-terminal', '--',
            'ros2', 'run', 'planner', 'plan',
            '--ros-args',
            # '-p', 'temp_dir:=/tmp_ctr/path_data/', 
            '--remap', '__node:=control_node'
        ],
        output='screen',
    )

    ld.add_action(planner_node)

    return ld
