from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    tracker_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '8',  # This sets the affinity to CPU core 0
            # 'gnome-terminal', '--',
            'ros2', 'run', 'igtlink_bridge', 'bridge',
            '--ros-args',
            '-p', 'hostname:=localhost',
            # '-p', 'port:=18944',
            # '-p', 'hostname:="10.15.225.222"',
            '-p', 'port:=18975',
            '-p', 'm_conv_to_ascension:=false',
            # '--ros-args', '--log-level', 'tf2:=warn',
            '--remap', '__node:=igtlink_bridge_node'
        ],
        output='screen',
    )

    ld.add_action(tracker_node)
    return ld
