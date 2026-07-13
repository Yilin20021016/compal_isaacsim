# my_real_dual_cumotion.launch.py
# ⚠️ WARNING: 這是為 cuMotion 測試建立的實機 MoveIt2 啟動腳本
# 此檔案用於測試 NVIDIA cuMotion GPU 運動規劃。
# 注意：在執行前必須確保已編譯安裝 isaac_ros_cumotion 並生成了 my_dual_gen3.xrdf

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 建立自訂的 MoveIt 配置，將 joint_limits 與 URDF 覆寫路徑指向我們的自定義檔案
    custom_urdf_path = "/home/alfred/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred/my_dual_gen3.urdf"
    custom_joint_limits_path = "/home/alfred/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred/my_joint_limits.yaml"
    
    # ⚠️ 預留的 XRDF 檔案路徑，需透過 Isaac Sim 生成
    custom_xrdf_path = "/home/alfred/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred/my_dual_gen3.xrdf"

    moveit_config = (
        MoveItConfigsBuilder("dual_gen3", package_name="dual_gen3_moveit_config")
        .robot_description(file_path=custom_urdf_path)
        .robot_description_semantic(file_path="config/dual_gen3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/real_moveit_controllers.yaml")
        # 🚨 將 planning_pipelines 更改為包含 isaac_ros_cumotion，並放在第一位作為預設管線
        .planning_pipelines(pipelines=["isaac_ros_cumotion", "ompl"])
        .joint_limits(file_path=custom_joint_limits_path)
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

    # 🚨 cuMotion Action Server 節點
    # 當 MoveIt 收到規劃請求時，會將其傳遞給這個背景執行的 GPU Server
    cumotion_node = Node(
        package="isaac_ros_cumotion",
        executable="cumotion_planner_node",
        name="cumotion_planner_node",
        output="screen",
        parameters=[
            {"robot": custom_xrdf_path},
            {"urdf_path": custom_urdf_path},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group,
        rviz,
        cumotion_node,
    ])
