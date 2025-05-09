from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    tracker_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '4',  # This sets the affinity to CPU core 0
            # 'gnome-terminal', '--',
            'ros2', 'run', 'emtracker', 'track',
            '--ros-args',
            '-p', 'cutoff_freq:=6.6',
            '-p', 'send_on_igtl:=false',
            '--remap', '__node:=emtracker_node'
        ],
        output='screen',
    )

    ld.add_action(tracker_node)
    return ld
