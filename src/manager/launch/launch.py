from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    manager_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'manager', 'mange',
            '--ros-args',
            '-p', 'sample_time:=0.001',  # Correct syntax for setting parameter
            '--remap', '__node:=Manager_node'
        ],
        output='screen',
    )

    recorder_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'manager', 'record',
            '--ros-args',
            '-p', 'sample_time:=0.004',  # Correct syntax for setting parameter
            '--remap', '__node:=Recorder_node'
        ],
        # output='screen',
    )

    # Delay the start of the observer_node by 5 seconds
    delay_manager_node = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[manager_node],
    )

    # ld.add_action(catheter_target_sim_node)
    ld.add_action(delay_manager_node)
    ld.add_action(recorder_node)
    return ld
