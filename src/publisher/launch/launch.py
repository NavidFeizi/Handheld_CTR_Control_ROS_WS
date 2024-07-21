from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'publisher', 'publish',
            '--ros-args',
            '-p', 'sample_time:=0.001',  # Correct syntax for setting parameter
            '--remap', '__node:=Publisher_node'
        ],
        # output='screen',
    )

    catheter_target_sim_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'publisher', 'simulator_koopman',
            '--ros-args',
            '-p', 'sample_time:=0.001',  # Correct syntax for setting parameter
            '--remap', '__node:=catheter_target_sim_node'
        ],
        output='screen',
    )

    recorder_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 1
            'ros2', 'run', 'publisher', 'record',
            '--ros-args',
            '-p', 'sample_time:=0.004',  # Correct syntax for setting parameter
            '--remap', '__node:=Recorder_node'
        ],
        output='screen',
    )

    # Delay the start of the observer_node by 5 seconds
    delay_observer_node = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[publisher_node],
    )

    # ld.add_action(catheter_target_sim_node)
    ld.add_action(publisher_node)
    ld.add_action(recorder_node)
    return ld
