from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    robot_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '4',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'robot', 'ctr_robot',
            # '--ros-args',
            '-p', 'Kp:=30.0',  # Correct syntax for setting parameter
            '-p', 'Ki:=5.0',  # Correct syntax for setting parameter
            '--remap', '__node:=robot_node'
        ],
        output='screen',
    )

    ld.add_action(robot_node)

    return ld
