# launch/total_dual_sim_planning.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
import subprocess

def launch_arm_setup(context, prefix_str):
    """
    動態生成單臂的 ros2_control 虛擬硬體節點，包含你原本寫的 XML 注入 Workaround
    """
    prefix = prefix_str # 'left_' 或 'right_'
    robot_ip = '192.168.1.10' if prefix == 'left_' else '192.168.1.11'
    gripper = 'robotiq_2f_85'
    gripper_joint_name = 'robotiq_85_left_knuckle_joint'
    dof = '7'
    
    kortex_description_share = get_package_share_directory('kortex_description')
    xacro_file = os.path.join(kortex_description_share, 'robots', 'gen3.xacro')
    
    cmd = [
        'xacro', xacro_file,
        f'robot_ip:={robot_ip}', 'name:=arm', 'arm:=gen3', f'dof:={dof}',
        f'prefix:={prefix}', 'use_fake_hardware:=true', 'mock_sensor_commands:=true',
        f'gripper:={gripper}', 'use_internal_bus_gripper_comm:=true',
        'gripper_max_velocity:=100.0', 'gripper_max_force:=100.0',
        f'gripper_joint_name:={gripper_joint_name}'
    ]
    
    try:
        robot_description_xml = subprocess.check_output(cmd, encoding='utf-8')
    except Exception as e:
        print(f"[ERROR] 執行 xacro 失敗: {e}")
        raise e

    # 動態注入 prefix 至 gripper_joint_name
    target_str = f'<param name="gripper_joint_name">{gripper_joint_name}</param>'
    replacement_str = f'<param name="gripper_joint_name">{prefix}{gripper_joint_name}</param>'
    if target_str in robot_description_xml:
        robot_description_xml = robot_description_xml.replace(target_str, replacement_str)

    robot_description = {'robot_description': robot_description_xml}
    
    # 載入對應命名空間的控制器設定檔
    robot_controllers = os.path.join(
        get_package_share_directory("dual_arm_control_bringup"), # 假設你的包名
        "config",
        f"{prefix}ros2_controllers.yaml"
    )

    # 啟動該臂的 controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output="screen",
    )

    # 依序 Spawner 拉起控制器
    jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "controller_manager"])
    jtc = Node(package="controller_manager", executable="spawner", arguments=["joint_trajectory_controller", "-c", "controller_manager"])
    rgc = Node(package="controller_manager", executable="spawner", arguments=["robotiq_gripper_controller", "-c", "controller_manager"])

    return [control_node, jsb, jtc, rgc]


def generate_launch_description():
    # 取得目前這個包的路徑
    my_pkg_share = get_package_share_directory("dual_arm_control_bringup")

    # 1. 左臂群組 (Namespace: /left)
    left_arm_group = GroupAction([
        PushRosNamespace('left'),
        OpaqueFunction(function=lambda context: launch_arm_setup(context, 'left_'))
    ])

    # 2. 右臂群組 (Namespace: /right)
    right_arm_group = GroupAction([
        PushRosNamespace('right'),
        OpaqueFunction(function=lambda context: launch_arm_setup(context, 'right_'))
    ])

    # 3. 關節狀態收集中心 (Joint State Combiner)
    # 因為 MoveIt2 需要看一整個整體的 /joint_states，而兩支手臂各自發布在 /left/joint_states 與 /right/joint_states
    # 我們引入 joint_state_publisher 來把兩者融合成同一個 Topic 給 MoveIt2 看
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/left/joint_states", "/right/joint_states"],
            "rate": 100
        }]
    )

    # 4. 引入你原本寫的 MoveIt2 + RViz2 啟動腳本 (包含 my_joint_limits.yaml 限制)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_pkg_share, "launch", "my_real_dual_moveit.launch.py")
        )
    )

    return LaunchDescription([
        left_arm_group,
        right_arm_group,
        joint_state_publisher,
        moveit_launch
    ])
