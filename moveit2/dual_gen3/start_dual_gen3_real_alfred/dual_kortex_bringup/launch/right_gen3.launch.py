from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    kortex_launch = os.path.join(
        get_package_share_directory('kortex_bringup'),
        'launch',
        'gen3.launch.py'
    )

    right_arm = GroupAction([
        PushRosNamespace('right'),

        SetRemap(src='/joint_states', dst='/right/joint_states'),
        SetRemap(src='/dynamic_joint_states', dst='/right/dynamic_joint_states'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kortex_launch),
            launch_arguments={
                'robot_ip': '192.168.1.11',
                'prefix': 'right_',
                'launch_rviz': 'false',
            }.items()
        )
    ])

    return LaunchDescription([
        right_arm
    ])
