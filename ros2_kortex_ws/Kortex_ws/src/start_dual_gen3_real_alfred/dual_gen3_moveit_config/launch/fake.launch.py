import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_name = "dual_gen3_moveit_config"
    
    # 1. 載入對齊的 URDF 與 SRDF
    urdf_path = os.path.join(get_package_share_directory(pkg_name), "config", "dual_gen3.urdf")
    srdf_path = os.path.join(get_package_share_directory(pkg_name), "config", "dual_gen3.srdf")
    
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
        
    with open(srdf_path, 'r') as f:
        robot_description_semantic_content = f.read()

    # 2. 載入基本規劃參數
    kinematics_yaml = load_yaml(pkg_name, "config/kinematics.yaml")
    joint_limits_yaml = load_yaml(pkg_name, "config/joint_limits.yaml")
    ompl_planning_yaml = load_yaml(pkg_name, "config/ompl_planning.yaml")
    
    # 3. 指定 MoveIt 2 使用內部虛擬控制器管理器
    fake_controller_manager = {
        "moveit_controller_manager": "moveit_fake_controller_manager/MoveItFakeControllerManager",
        "fake_execution_type": "interpolate",
    }

    move_group_parameters = {
        "robot_description": robot_description_content,
        "robot_description_semantic": robot_description_semantic_content,
        "robot_description_kinematics": kinematics_yaml,
        "joint_limits": joint_limits_yaml,
        "ompl": ompl_planning_yaml,
        "use_sim_time": False,
    }
    move_group_parameters.update(fake_controller_manager)

    # Node A: MoveGroup 核心
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_parameters],
    )

    # Node B: Robot State Publisher (負責將 joint_states 轉換為座標系 TF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": False}],
    )

    # Node C: 修正版 - 讓 joint_state_publisher 直接吃 robot_description 參數
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "robot_description": robot_description_content, # ⚠️ 加上這行，讓它直接拿到 URDF
            "source_list": ["/move_group/fake_controller_joint_states"],
            "use_sim_time": False,
            "rate": 50
        }]
    )

    # Node D: 修正版 RViz 2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": False},
            {"robot_description_semantic": robot_description_semantic_content}
        ]
    )

    return LaunchDescription([
        run_move_group_node,
        robot_state_publisher,
        joint_state_publisher_node,
        rviz_node,
    ])