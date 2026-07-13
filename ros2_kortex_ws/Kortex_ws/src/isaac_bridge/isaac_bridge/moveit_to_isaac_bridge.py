#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DualArmSplitIsaacBridge(Node):
    def __init__(self):
        super().__init__('dual_arm_split_isaac_bridge')
        
        # 單臂標準關節結構（必須與 Isaac Sim 的 Available DOFs 完全一致）
        self.base_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7', 'finger_joint'
        ]
        
        # 訂閱 MoveIt 2 發出的完整關節狀態 (通常帶有 left_ 或 right_ 前綴)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 分流發布：左臂與右臂獨立 Topic
        self.left_publisher = self.create_publisher(
            JointState,
            '/isaac_left_joint_command',
            10
        )
        self.right_publisher = self.create_publisher(
            JointState,
            '/isaac_right_joint_command',
            10
        )
        
        self.get_logger().info('Dual Arm Split Isaac Bridge (DOF Prefix Stripped) has started.')

    def _build_arm_msg(self, header, prefix, source_data):
        """建立單一手臂的 JointState 訊息，對來源資料進行前綴查找，但發布時移除前綴"""
        msg = JointState()
        msg.header = header
        
        # 【關鍵修正】：發布給該手臂控制器時，名字不需要帶有 left_ 或 right_ 前綴！
        msg.name = self.base_names
        
        num_joints = len(self.base_names)
        positions = [0.0] * num_joints
        msg.velocity = [0.0] * num_joints
        msg.effort = [0.0] * num_joints
        
        for t_idx, base_name in enumerate(self.base_names):
            if base_name == 'finger_joint':
                # 夾爪映射邏輯：從 MoveIt 的帶前綴名稱中抽取數據
                gripper_src = f"{prefix}robotiq_85_left_knuckle_joint"
                if gripper_src in source_data:
                    positions[t_idx] = source_data[gripper_src]
            else:
                # 手臂軸 1 ~ 7 映射邏輯：從 MoveIt (如 left_joint_1) 拿數值
                full_joint_name = f"{prefix}{base_name}"
                if full_joint_name in source_data:
                    positions[t_idx] = source_data[full_joint_name]
                    
        msg.position = positions
        return msg

    def joint_state_callback(self, msg):
        # 將收到的所有關節數據建立字典
        source_data = {}
        max_len = min(len(msg.name), len(msg.position))
        for i in range(max_len):
            source_data[msg.name[i]] = msg.position[i]
        
        # 分別建立左、右臂的命令訊息 (傳入 prefix 用於在字典中尋找正確的 MoveIt 關節資料)
        left_cmd = self._build_arm_msg(msg.header, 'left_', source_data)
        right_cmd = self._build_arm_msg(msg.header, 'right_', source_data)
        
        # 分流送出
        self.left_publisher.publish(left_cmd)
        self.right_publisher.publish(right_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DualArmSplitIsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()