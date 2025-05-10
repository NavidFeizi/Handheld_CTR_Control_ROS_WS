from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    robot_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '5',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'robot', 'ctr_robot',
            '-p', 'Kp:=30.0',  # Correct syntax for setting parameter
            '-p', 'Ki:=5.0',  # Correct syntax for setting parameter
            '--ros-args --remap __node:=robot_node'
        ],
        output='screen',
    )

    gui_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '6',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'robot', 'qt_gui',
            '--ros-args --remap __node:=gui_node'
        ],
        output='screen',
    )

    fk_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '7',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'robot', 'forward_kin',
            '--ros-args --remap __node:=fk_node'
        ],
        output='screen',
    )

    # Delay the start of the observer_node by 5 seconds
    delay_gui_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[gui_node],
    )

    ld.add_action(robot_node)
    ld.add_action(delay_gui_node)
    ld.add_action(fk_node)

    return ld
