from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    robot_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '3',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'robot', 'ctr_robot',
            # '--ros-args',
            '-p', 'Kp:=30.0',  # Correct syntax for setting parameter
            '-p', 'Ki:=5.0',  # Correct syntax for setting parameter
            '--remap', '__node:=robot_node'
        ],
        output='screen',
    )

    keyboard_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'robot', 'key_input',
            '--ros-args',
            # '-p', 'sample_time:=0.001',  # Correct syntax for setting parameter
            '--remap', '__node:=Publisher_node'
        ],
        output='screen',
    )

    # Delay the start of the observer_node by 5 seconds
    delay_keyboard_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[keyboard_node],
    )

    ld.add_action(robot_node)
    ld.add_action(delay_keyboard_node)

    return ld
