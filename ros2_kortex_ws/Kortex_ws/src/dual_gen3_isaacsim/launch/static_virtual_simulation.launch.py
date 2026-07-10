import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_setup_assistant.launch_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 指定你的 moveit config package 名稱
    pkg_name = "my_dual_gen3_moveit_config"
    
    # 2. 使用 MoveItConfigsBuilder 自動載入 URDF, SRDF 與 limit YAMLs
    moveit_config = (
        MoveItConfigsBuilder("dual_gen3", package_name=pkg_name)
        .robot_description(file_path="config/dual_gen3.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_gen3.srdf")
        .joint_limits(file_path="config/joint_limits.yaml")
        # 關鍵點：啟用 fake_hardware
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # 3. 啟動 MoveGroup Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # 4. 啟動 Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 5. 關鍵：ROS 2 Control Fake Hardware 驅動與 Joint State Broadcaster
    # 這裡利用 moveit_resources 或 kortex_description 提供的 fake components
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory(pkg_name), "config", "ros2_controllers.yaml")
        ],
        output="screen",
    )

    # 載入 joint_state_broadcaster (負責吐出你要的 joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 載入你的雙臂軌跡控制器 (例如 joint_trajectory_controller)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dual_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 6. 可選：啟動 RViz2 視覺化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory(pkg_name), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        run_move_group_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz_node,
    ])
