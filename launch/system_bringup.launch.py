# Compatibility shim: the bring-up launch moved to the ctr_bringup package.
# This file keeps the documented invocation working:
#     ros2 launch launch/system_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ctr_bringup'), 'launch', 'system_bringup.launch.py'])
            )
        )
    ])
