# my_left_gen3.launch.py
# ⚠️ WARNING: 自訂右手實機啟動腳本 (Alfred 動態描述檔注入版)
# 此檔案由 Antigravity 自動生成，用於非侵入式地動態注入 gripper_joint_name 的 prefix，
# 解決右手 Robotiq 2F-85 夾爪硬體介面註冊衝突，同時保持 src/ros2_kortex/kortex_description 原始碼完全乾淨。

import os
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # 手臂與夾爪參數設定
    robot_ip = '192.168.1.10'
    prefix = 'left_'
    gripper = 'robotiq_2f_85'
    gripper_joint_name = 'robotiq_85_left_knuckle_joint'
    dof = '7'
    controllers_file = 'ros2_controllers.yaml'
    
    # 獲取原廠描述包路徑
    kortex_description_share = get_package_share_directory('kortex_description')
    xacro_file = os.path.join(kortex_description_share, 'robots', 'gen3.xacro')
    
    # 執行 xacro 命令生成 robot_description XML 字串
    cmd = [
        'xacro',
        xacro_file,
        f'robot_ip:={robot_ip}',
        'name:=arm',
        'arm:=gen3',
        f'dof:={dof}',
        f'prefix:={prefix}',
        'use_fake_hardware:=true',
        'fake_sensor_commands:=true',
        f'gripper:={gripper}',
        'use_internal_bus_gripper_comm:=true',
        'gripper_max_velocity:=100.0',
        'gripper_max_force:=100.0',
        f'gripper_joint_name:={gripper_joint_name}'
    ]
    
    try:
        robot_description_xml = subprocess.check_output(cmd, encoding='utf-8')
    except Exception as e:
        print(f"[ERROR] 執行 xacro 失敗: {e}")
        raise e

    # 🚨 [動態 XML 修正] 尋找原本的夾爪關節參數並將其替換為帶有前綴的版本
    # 原廠 XML 內容: <param name="gripper_joint_name">robotiq_85_left_knuckle_joint</param>
    # 替換後 XML 內容: <param name="gripper_joint_name">left_robotiq_85_left_knuckle_joint</param>
    target_str = f'<param name="gripper_joint_name">{gripper_joint_name}</param>'
    replacement_str = f'<param name="gripper_joint_name">{prefix}{gripper_joint_name}</param>'
    
    if target_str in robot_description_xml:
        robot_description_xml = robot_description_xml.replace(target_str, replacement_str)
        print("[SUCCESS] 成功動態注入 prefix 至 gripper_joint_name！已恢復原始 src/ 代碼且不影響夾爪功能。")
    else:
        print("[WARN] 無法在產出的 robot_description XML 中找到目標參數，請確認 xacro 生成結構。")

    robot_description = {'robot_description': robot_description_xml}

    # 載入控制器配置路徑 (使用自訂的 left_ros2_controllers.yaml)
    robot_controllers = os.path.join(
        os.path.dirname(__file__),
        "left_ros2_controllers.yaml"
    )

    # 1. 啟動 ros2_control_node (硬體驅動端)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # 2. 啟動 robot_state_publisher (發布右手 TF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 3. 啟動右臂關節狀態廣播器 (joint_state_broadcaster)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    # 4. 啟動右臂軌跡控制器 (joint_trajectory_controller)
    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "controller_manager"],
    )

    # 5. 啟動右臂笛卡爾笛坐標控制器 (twist_controller, 設為預設不啟用)
    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "controller_manager"],
    )

    # 6. 啟動右手 Robotiq 2F-85 夾爪控制器 (robotiq_gripper_controller)
    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "controller_manager"],
    )

    # 7. 啟動右手錯誤監測控制器 (fault_controller)
    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fault_controller", "-c", "controller_manager"],
    )

    return [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        fault_controller_spawner,
    ]

def generate_launch_description():
    from launch.actions import GroupAction
    from launch_ros.actions import PushRosNamespace, SetRemap

    # 將所有右手節點包入命名空間與 Remap 設定中
    left_arm = GroupAction([
        PushRosNamespace('left'),
        # SetRemap(src='/joint_states', dst='/left/joint_states'),
        # SetRemap(src='/dynamic_joint_states', dst='/left/dynamic_joint_states'),
        OpaqueFunction(function=launch_setup)
    ])

    return LaunchDescription([
        left_arm
    ])
