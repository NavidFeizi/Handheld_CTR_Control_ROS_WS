import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('planner'), 'config', 'planner_params.yaml')

    planner_node = Node(
        package='planner',
        executable='plan',
        name='planner_node',
        output='screen',
        prefix=['taskset -c 9'],
        parameters=[params_file],
    )

    ld = LaunchDescription()
    ld.add_action(planner_node)

    return ld
