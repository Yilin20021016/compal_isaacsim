import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_name = "dual_gen3_moveit_config"
    
    # 這裡抓取你最新的與夾爪整合的檔案，或是 dual_gen3.urdf
    # 建議先用你驗證過 ok 的檔，例如 config/dual_gen3_with_gripper.urdf.good 或是目前的 checkpoint
    urdf_file = "config/dual_gen3.urdf" 
    srdf_file = "config/dual_gen3.srdf"

    moveit_config = (
        MoveItConfigsBuilder("dual_gen3", package_name=pkg_name)
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # 關鍵點 1：強制讓 MoveIt 載入 fake_hardware 參數
        .to_moveit_configs()
    )

    # 調整 move_group 參數
    move_group_parameters = moveit_config.to_dict()
    # 確保開啟模擬時間（如果後面要接 Isaac Sim）或關閉（純粹靠 moveit2 內部循環跑軌跡）
    move_group_parameters["use_sim_time"] = True 

    # 1. MoveGroup Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_parameters],
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # 關鍵點 2：使用 mock_components 代替 Kinova 實機硬體連線
    # 這會阻斷對實機 IP 的連線，改由 ros2_control 在本地記憶體模擬狀態
    ros2_controllers_path = os.path.join(
        get_package_share_directory(pkg_name), "config", "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": False}
        ],
        output="screen",
    )

    # 3. Spawners (負責把你的控制器掛載上去並啟動)
    # 這裡的名稱（如 joint_state_broadcaster）必須與你的 ros2_controllers.yaml 定義一致
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 請根據你的 ros2_controllers.yaml 裡面的 arm controller 名稱修改下面這個字串
    # 可能是 dual_arm_trajectory_controller 或是 left_arm_controller 等
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. RViz 視覺化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False}
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
