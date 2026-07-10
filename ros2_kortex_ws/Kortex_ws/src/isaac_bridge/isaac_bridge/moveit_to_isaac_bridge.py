#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MoveItToIsaacBridge(Node):
    def __init__(self):
        super().__init__('moveit_to_isaac_bridge')
        
        # 定義標準 8 個關節順序
        self.target_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 
            'joint_5', 'joint_6', 'joint_7', 'finger_joint'
        ]
        
        # 訂閱 MoveIt 2 發出的亂序 joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 發布符合 Isaac Sim 格式的 /isaac_joint_command
        self.publisher = self.create_publisher(
            JointState,
            '/isaac_joint_command',
            10
        )
        
        self.get_logger().info('Kinova Gen3 MoveIt to Isaac Bridge Node has started.')

    def joint_state_callback(self, msg):
        cmd_msg = JointState()
        cmd_msg.header = msg.header
        cmd_msg.name = self.target_names
        
        # 初始化 8 個關節的位置、速度與力矩陣列
        num_targets = len(self.target_names)
        positions = [0.0] * num_targets
        velocities = [0.0] * num_targets
        efforts = [0.0] * num_targets
        
        # 將收到的亂序資料建立成字典，方便用名字直接查找
        # 格式如: {'joint_1': 2.023, 'robotiq_85_left_knuckle_joint': 0.799}
        source_data = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                source_data[name] = msg.position[i]
                
        # 開始映射到目標 8 個關節
        for t_idx, t_name in enumerate(self.target_names):
            if t_name == 'finger_joint':
                # 將 MoveIt 的主動夾爪關節數值映射給 finger_joint
                # 這裡使用 robotiq_85_left_knuckle_joint 作為來源
                gripper_src = 'robotiq_85_left_knuckle_joint'
                if gripper_src in source_data:
                    positions[t_idx] = source_data[gripper_src]
            else:
                # 處理 joint_1 到 joint_7
                if t_name in source_data:
                    positions[t_idx] = source_data[t_name]
                    
        # 填入處理好的數據
        cmd_msg.position = positions
        cmd_msg.velocity = velocities
        cmd_msg.effort = efforts
        
        # 發布到 /isaac_joint_command
        self.publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveItToIsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
