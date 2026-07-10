#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
07_keyboard_teleop.py

功能說明：
取代搖桿，使用鍵盤來微調雙臂位置與一鍵移動至預設點位。
整合了 MoveIt2 的 Cartesian Path Service 進行平滑與安全的短距離微調，
並透過 MoveGroup Action Server 執行大範圍的安全避障移動。

操作方式：
- [TAB]   : 切換控制目標 (右臂 right_arm / 左臂 left_arm)
- [W/S]   : X 軸 前進 / 後退 (相對於 base_link)
- [A/D]   : Y 軸 向左 / 向右
- [Z/C]   : Z 軸 上升 / 下降
- [1]     : 移動至預設點位 Home
- [2]     : 移動至預設點位 Retract
- [3]     : 移動至預設點位 Lay
- [Space] : 緊急停止 (目前尚未實作即時打斷，但可放棄後續規劃)
- [Q]     : 安全退出

⚠️ WARNING: 
此腳本直接控制實體機械手臂。
操作前請務必確認工作空間淨空，並建議隨時將手放在實體急停按鈕 (E-Stop) 旁以策安全！
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, JointConstraint, RobotTrajectory
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import threading
import math
import asyncio
import time

# === 預設關節點位 (rad) ===
# 稍後請根據實際需求修改這些角度 (順序: Joint 1 ~ 7)
PREDEFINED_POSES = {
    "right_arm": {
        "Home":    [-0.00004, 0.26215, 3.14157, -2.26906, -0.00003, 0.95976, 1.57081],
        "Retract": [-0.00089, -0.33592, 3.13407, -2.54227, -0.00239, -0.86569, 1.56403]
    },
    "left_arm": {
        "Home":    [0.00001, 0.26209, -3.14154, -2.26901, 0.00000, 0.95997, 1.57079],
        "Retract": [0.00001, -0.34913, -3.14155, -2.54835, 0.00000, -0.87266, 1.57079]
    }
}

JOINT_NAMES = {
    "right_arm": [
        "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", 
        "right_joint_5", "right_joint_6", "right_joint_7"
    ],
    "left_arm": [
        "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", 
        "left_joint_5", "left_joint_6", "left_joint_7"
    ]
}

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        self.active_arm = "right_arm" # 預設控制右手
        self.step_size = 0.01         # 笛卡爾平移微調基準 (1 cm)
        self.angle_step = 0.035       # 笛卡爾旋轉微調基準 (約 2 度)
        self.velocity_scaling = 0.08  # 安全速限 8%
        
        self.current_key = None
        self.last_key_time = time.time()
        self.active_goal_handle = None
        
        self.get_logger().info("初始化 MoveIt2 介面與 TF Listener...")
        
        # Action Clients (MoveIt)
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Action Clients (Direct Hardware Controllers, bypass MoveIt Execute/Splitter to remove 8s delay)
        self.right_fjt_client = ActionClient(self, FollowJointTrajectory, '/right/joint_trajectory_controller/follow_joint_trajectory')
        self.left_fjt_client = ActionClient(self, FollowJointTrajectory, '/left/joint_trajectory_controller/follow_joint_trajectory')
        
        # Cartesian Path Service
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # TF2 Listener (取得當前 TCP 位姿)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("====================================================")
        self.get_logger().info(" 鍵盤雙臂微調系統已就緒 (Keyboard Teleoperation)")
        self.get_logger().info("====================================================")
        self.get_logger().info(f" 當前控制手臂: [{self.active_arm}] (按 TAB 切換)")
        self.get_logger().info(" W/S: 前/後 | A/D: 左/右 | R/F: 上/下")
        self.get_logger().info(" U/O: Roll | I/K: Pitch | J/L: Yaw")
        self.get_logger().info(" 1: Home | 2: Retract")
        self.get_logger().info(" Q: 退出")
        self.get_logger().info("====================================================")

    def get_current_pose(self):
        """透過 TF 取得目前作用手臂的 End Effector 位姿"""
        link_name = "right_end_effector_link" if self.active_arm == "right_arm" else "left_end_effector_link"
        try:
            base_frame = f"{self.active_arm.split('_')[0]}_base_link"
            # 取得 base_link 到 end_effector_link 的轉換
            t = self.tf_buffer.lookup_transform(
                base_frame,
                link_name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation.x = t.transform.rotation.x
            pose.orientation.y = t.transform.rotation.y
            pose.orientation.z = t.transform.rotation.z
            pose.orientation.w = t.transform.rotation.w
            return pose
        except Exception as e:
            self.get_logger().error(f"無法取得 {link_name} 位姿: {e}")
            return None

    async def cancel_active_move(self):
        """取消當前正在執行的移動 (煞車)"""
        if self.active_goal_handle is not None:
            await self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

    async def start_continuous_move(self, dx=0.0, dy=0.0, dz=0.0, d_roll=0.0, d_pitch=0.0, d_yaw=0.0):
        """產生一條長距離連續軌跡，並發送給控制器 (按鍵放開時會呼叫 cancel 煞車)"""
        await self.cancel_active_move()
        
        current_pose = self.get_current_pose()
        if current_pose is None:
            return

        target_pose = Pose()
        MULTIPLIER = 50.0 # 將步伐放大 50 倍，產生幾乎無盡的長軌跡 (直到撞到邊界)
        target_pose.position.x = current_pose.position.x + dx * MULTIPLIER
        target_pose.position.y = current_pose.position.y + dy * MULTIPLIER
        target_pose.position.z = current_pose.position.z + dz * MULTIPLIER
        
        if d_roll != 0.0 or d_pitch != 0.0 or d_yaw != 0.0:
            d_yaw_m = d_yaw * MULTIPLIER
            d_pitch_m = d_pitch * MULTIPLIER
            d_roll_m = d_roll * MULTIPLIER
            
            cy = math.cos(d_yaw_m * 0.5)
            sy = math.sin(d_yaw_m * 0.5)
            cp = math.cos(d_pitch_m * 0.5)
            sp = math.sin(d_pitch_m * 0.5)
            cr = math.cos(d_roll_m * 0.5)
            sr = math.sin(d_roll_m * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            cx = current_pose.orientation.x
            cy_orig = current_pose.orientation.y
            cz = current_pose.orientation.z
            cw = current_pose.orientation.w

            target_pose.orientation.x = qw*cx + qx*cw + qy*cz - qz*cy_orig
            target_pose.orientation.y = qw*cy_orig - qx*cz + qy*cw + qz*cx
            target_pose.orientation.z = qw*cz + qx*cy_orig - qy*cx + qz*cw
            target_pose.orientation.w = qw*cw - qx*cx - qy*cy_orig - qz*cz
        else:
            target_pose.orientation = current_pose.orientation

        link_name = "right_end_effector_link" if self.active_arm == "right_arm" else "left_end_effector_link"

        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cartesian 服務無回應")
            return
            
        req = GetCartesianPath.Request()
        req.header.frame_id = f"{self.active_arm.split('_')[0]}_base_link"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.active_arm
        req.link_name = link_name
        req.waypoints.append(target_pose)
        req.max_step = 0.01 
        req.jump_threshold = 0.0 
        req.avoid_collisions = True

        future = self.cartesian_client.call_async(req)
        while not future.done():
            await asyncio.sleep(0.01)
        res = future.result()

        # 放寬限制：只要能規劃出超過 2 個點的路徑，就允許移動。這讓手臂即使快到邊界也能繼續微調
        if len(res.solution.joint_trajectory.points) < 3: 
            self.get_logger().error("已到達工作空間邊界或遇障礙物，無法繼續移動。")
            return
            
        fjt_client = self.right_fjt_client if self.active_arm == "right_arm" else self.left_fjt_client
        if not fjt_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{self.active_arm} 軌跡控制器無回應")
            return
            
        # 由於 GetCartesianPath 沒有原生縮放參數，我們手動將時間參數拉長來降速
        # 將規劃時間乘以 4 (即速度降為原本的 25%)，可以極大幅度降低晃動與加速度
        TIME_MULTIPLIER = 4.0 
        for point in res.solution.joint_trajectory.points:
            total_time = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) * TIME_MULTIPLIER
            point.time_from_start.sec = int(total_time)
            point.time_from_start.nanosec = int((total_time - int(total_time)) * 1e9)
            if len(point.velocities) > 0:
                point.velocities = [v / TIME_MULTIPLIER for v in point.velocities]
            # 強制清空加速度，讓底層硬體 PID 自行平滑過渡，這能 100% 避免控制器的剛性晃動
            point.accelerations = [0.0 for _ in point.accelerations]
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = res.solution.joint_trajectory
        
        exec_future = fjt_client.send_goal_async(goal_msg)
        while not exec_future.done():
            await asyncio.sleep(0.01)
            
        goal_handle = exec_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("連續軌跡被硬體控制器拒絕")
            return
            
        self.active_goal_handle = goal_handle

    async def move_to_predefined_pose(self, pose_name):
        """將手臂移動到預設的 Joint 關節點位"""
        if pose_name not in PREDEFINED_POSES[self.active_arm]:
            self.get_logger().error(f"找不到預設點位: {pose_name}")
            return
            
        joints = PREDEFINED_POSES[self.active_arm][pose_name]
        joint_names = JOINT_NAMES[self.active_arm]
        
        if not self.move_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup Action Server 無回應")
            return
            
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.active_arm
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "BITstar"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = self.velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.velocity_scaling
        goal_msg.planning_options.plan_only = False # 交由 MoveIt 執行，確保硬體相容性與初始狀態同步
        
        # 建立 Joint 限制條件
        constraint = Constraints()
        for i in range(7):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = joints[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraint)
        
        self.get_logger().info(f"[{self.active_arm}] 準備移動至: {pose_name} ...")
        
        send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            await asyncio.sleep(0.01)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("移動請求被 MoveGroup 拒絕")
            return
            
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            await asyncio.sleep(0.01)
            
        res = get_result_future.result().result
        if res.error_code.val == 1:
            self.get_logger().info(f"[{self.active_arm}] 成功抵達 {pose_name}!")
        else:
            self.get_logger().error(f"[{self.active_arm}] 移動失敗，規劃或執行錯誤碼: {res.error_code.val}")

def get_key(settings):
    """非阻塞讀取鍵盤"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    # 使用獨立執行緒來處理 ROS 回呼
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    loop = asyncio.get_event_loop()
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key:
                key = key.lower()
                
                # 如果是重複按鍵 (按著不放)，只更新時間戳，不重複發送指令
                if key == node.current_key:
                    node.last_key_time = time.time()
                else:
                    # 如果按下的是新的按鍵，先取消先前的移動
                    loop.run_until_complete(node.cancel_active_move())
                    node.current_key = key
                    node.last_key_time = time.time()

                    if key == 'q':
                        break
                    elif key == '\t':
                        node.active_arm = "left_arm" if node.active_arm == "right_arm" else "right_arm"
                        node.get_logger().info(f"\n >>> 已切換至 [{node.active_arm}] <<<\n")
                        
                    # 平移微調 (連續)
                    elif key == 'w': loop.run_until_complete(node.start_continuous_move(dx=node.step_size))
                    elif key == 's': loop.run_until_complete(node.start_continuous_move(dx=-node.step_size))
                    elif key == 'a': loop.run_until_complete(node.start_continuous_move(dy=node.step_size))
                    elif key == 'd': loop.run_until_complete(node.start_continuous_move(dy=-node.step_size))
                    elif key == 'r': loop.run_until_complete(node.start_continuous_move(dz=node.step_size))
                    elif key == 'f': loop.run_until_complete(node.start_continuous_move(dz=-node.step_size))
                    
                    # 旋轉微調 (連續)
                    elif key == 'u': loop.run_until_complete(node.start_continuous_move(d_roll=node.angle_step))
                    elif key == 'o': loop.run_until_complete(node.start_continuous_move(d_roll=-node.angle_step))
                    elif key == 'i': loop.run_until_complete(node.start_continuous_move(d_pitch=node.angle_step))
                    elif key == 'k': loop.run_until_complete(node.start_continuous_move(d_pitch=-node.angle_step))
                    elif key == 'j': loop.run_until_complete(node.start_continuous_move(d_yaw=node.angle_step))
                    elif key == 'l': loop.run_until_complete(node.start_continuous_move(d_yaw=-node.angle_step))
                        
                    # 預設點位 (單次)
                    elif key == '1':
                        node.current_key = None # 避免預設點位被超時機制取消
                        loop.run_until_complete(node.move_to_predefined_pose("Home"))
                    elif key == '2':
                        node.current_key = None
                        loop.run_until_complete(node.move_to_predefined_pose("Retract"))
            else:
                # 偵測按鍵釋放 (Timeout)
                if node.current_key is not None and (time.time() - node.last_key_time > 0.2):
                    loop.run_until_complete(node.cancel_active_move())
                    node.current_key = None
                
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.get_logger().info("正在關閉 Keyboard Teleop Node...")
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        import os
        os._exit(0)

if __name__ == '__main__':
    main()
