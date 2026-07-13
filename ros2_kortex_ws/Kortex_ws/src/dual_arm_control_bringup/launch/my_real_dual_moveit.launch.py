# my_real_dual_moveit.launch.py
# ⚠️ WARNING: 自訂實機 MoveIt2 啟動腳本
# 此檔案由 Antigravity 自動生成，用於載入自訂的關節限幅檔案以防止手臂亂轉。

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 取得你新 package 的 share 路徑
    my_pkg_share = get_package_share_directory("dual_arm_control_bringup")
    
    custom_urdf_path = os.path.join(my_pkg_share, "config", "my_dual_gen3.urdf")
    custom_joint_limits_path = os.path.join(my_pkg_share, "config", "my_joint_limits.yaml")
    
    moveit_config = (
        MoveItConfigsBuilder("dual_gen3", package_name="dual_gen3_moveit_config")
        .robot_description(file_path=custom_urdf_path)  # 👈 強制覆寫為我們在 scripts 下的 URDF 設定！
        .robot_description_semantic(file_path="config/dual_gen3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/real_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path=custom_joint_limits_path)  # 👈 強制覆寫為我們的限制設定！
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("dual_gen3_moveit_config"),
        "config",
        "moveit.rviz"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    trajectory_execution_timeout_params = {
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 200.0,
            "allowed_goal_duration_margin": 120.0,
            "allowed_start_tolerance": 0.1,
            "execution_duration_monitoring": False,
        },
        "trajectory_execution.allowed_execution_duration_scaling": 200.0,
        "trajectory_execution.allowed_goal_duration_margin": 120.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
        "trajectory_execution.execution_duration_monitoring": False,
    }


    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'publish_planning_scene_hz': 2.0},
            trajectory_execution_timeout_params,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([

        robot_state_publisher,
        move_group,

        TimerAction(period=3.0, actions=[rviz]),
    ])
