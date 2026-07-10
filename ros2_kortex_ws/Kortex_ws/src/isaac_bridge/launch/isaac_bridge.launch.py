import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 引入 Kortex 的官方啟動檔案
    moveit_launch_dir = get_package_share_directory('kinova_gen3_7dof_robotiq_2f_85_moveit_config')
    
    kinova_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, 'launch', 'robot.launch.py')
        ),

        launch_arguments={
            'robot_ip': 'yyy.yyy.yyy.yyy',
            'use_fake_hardware': 'true'
        }.items()
    )

    # 2. 宣告 Python Bridge 節點
    isaac_bridge_node = Node(
        package='isaac_bridge',
        executable='moveit_bridge', # 確保與 setup.py 中 entry_points 一致
        name='moveit_to_isaac_bridge',
        output='screen'
    )


    return LaunchDescription([
        kinova_moveit_launch,
        isaac_bridge_node
    ])
