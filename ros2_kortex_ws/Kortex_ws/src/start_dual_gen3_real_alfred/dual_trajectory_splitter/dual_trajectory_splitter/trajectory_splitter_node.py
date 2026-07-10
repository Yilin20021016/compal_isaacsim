#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class DualTrajectorySplitter(Node):
    def __init__(self):
        super().__init__('trajectory_splitter_node_SAFE_SOURCE')

        self.callback_group = ReentrantCallbackGroup()

        # 儲存目前實體手臂 joint state，讓 splitter 送 trajectory 時可以用真正 current position 當起點
        self.latest_joint_positions = {}

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # MoveIt 可能送到的 action 名稱。
        # 你的 MoveIt 只會用其中一個，但多開幾個是為了避免名稱對不上。
        self.input_action_names = [
            '/dual_arm_controller/follow_joint_trajectory',
            '/dual_trajectory_splitter/follow_joint_trajectory',
            '/trajectory_splitter/follow_joint_trajectory',
            '/dual_trajectory_controller/follow_joint_trajectory',
            '/both_arms_controller/follow_joint_trajectory',
            '/joint_trajectory_controller/follow_joint_trajectory',
        ]

        # 左右手實體 controller 的 action 名稱
        self.left_controller_action = '/left/joint_trajectory_controller/follow_joint_trajectory'
        self.right_controller_action = '/right/joint_trajectory_controller/follow_joint_trajectory'

        # 依照你目前 controller 顯示的 joint 名稱
        self.left_joint_names = [
            'left_joint_1',
            'left_joint_2',
            'left_joint_3',
            'left_joint_4',
            'left_joint_5',
            'left_joint_6',
            'left_joint_7',
        ]

        self.right_joint_names = [
            'right_joint_1',
            'right_joint_2',
            'right_joint_3',
            'right_joint_4',
            'right_joint_5',
            'right_joint_6',
            'right_joint_7',
        ]

        # 建立 action clients，負責送給左右手 controller
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.left_controller_action,
            callback_group=self.callback_group
        )

        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.right_controller_action,
            callback_group=self.callback_group
        )

        # 建立 action servers，讓 MoveIt Execute 可以送進來
        self.action_servers = []

        for action_name in self.input_action_names:
            server = ActionServer(
                self,
                FollowJointTrajectory,
                action_name,
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.callback_group
            )
            self.action_servers.append(server)
            self.get_logger().info(f'Action server ready: {action_name}')

        self.get_logger().info('SAFE SOURCE SPLITTER VERSION LOADED')
        self.get_logger().info('Dual trajectory splitter node started.')
        self.get_logger().info(f'Left controller action: {self.left_controller_action}')
        self.get_logger().info(f'Right controller action: {self.right_controller_action}')

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = position

    def get_current_positions_for_joints(self, joint_names):
        if not joint_names:
            return None

        positions = []

        for joint_name in joint_names:
            if joint_name not in self.latest_joint_positions:
                return None
            positions.append(self.latest_joint_positions[joint_name])

        return positions

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request from MoveIt.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Received cancel request from MoveIt.')
        return CancelResponse.REJECT

    def make_result(self, error_code=FollowJointTrajectory.Result.SUCCESSFUL, error_string=''):
        result = FollowJointTrajectory.Result()
        result.error_code = int(error_code)
        result.error_string = str(error_string)
        self.get_logger().info(
            f"Returning result to MoveIt: error_code={result.error_code}, "
            f"error_string='{result.error_string}'"
        )
        return result

    def wait_for_future(self, future, timeout_sec):
        """
        等待 action future 完成。
        使用 threading.Event，避免在 callback 裡面重複 spin 造成卡死。
        """
        event = threading.Event()

        def _done_callback(_):
            event.set()

        future.add_done_callback(_done_callback)

        if not future.done():
            event.wait(timeout=timeout_sec)

        if not future.done():
            return None

        return future.result()

    def select_values(self, values, indices, full_joint_count):
        """
        從完整 trajectory point 裡面取出左手或右手需要的數值。
        positions / velocities / accelerations / effort 都會用到。
        """
        if values is None:
            return []

        if len(values) == 0:
            return []

        if len(values) != full_joint_count:
            return []

        return [values[i] for i in indices]

    def build_sub_trajectory(self, full_traj, target_joint_names):
        """
        從 MoveIt 給的完整 trajectory 中，拆出指定 joints 的 trajectory。
        例如拆出 right_joint_1 ~ right_joint_7。
        """
        full_joint_names = list(full_traj.joint_names)
        full_joint_count = len(full_joint_names)

        indices = []
        selected_joint_names = []

        # 依照 controller 需要的 joint 順序建立 trajectory
        for joint_name in target_joint_names:
            if joint_name in full_joint_names:
                indices.append(full_joint_names.index(joint_name))
                selected_joint_names.append(joint_name)

        if len(indices) == 0:
            return None

        sub_traj = JointTrajectory()
        sub_traj.header = copy.deepcopy(full_traj.header)
        sub_traj.joint_names = selected_joint_names

        for point in full_traj.points:
            new_point = JointTrajectoryPoint()

            new_point.positions = self.select_values(
                point.positions,
                indices,
                full_joint_count
            )

            new_point.velocities = self.select_values(
                point.velocities,
                indices,
                full_joint_count
            )

            new_point.accelerations = self.select_values(
                point.accelerations,
                indices,
                full_joint_count
            )

            new_point.effort = self.select_values(
                point.effort,
                indices,
                full_joint_count
            )

            new_point.time_from_start = copy.deepcopy(point.time_from_start)

            sub_traj.points.append(new_point)

        return sub_traj

    def refresh_trajectory_time(self, traj, start_stamp):
        """
        重新整理 trajectory 時間。
        1. 更新 header.stamp，避免 controller 判定時間太舊
        2. 拉長 time_from_start，讓實體手臂動慢一點
        3. 同步縮小速度與加速度，讓動作比較平順
        """
        traj.header.stamp = start_stamp

        # 放慢倍率。5.0 代表整段動作約慢 5 倍
        time_scale_factor = 40.0

        # 每個 waypoint 至少間隔 0.3 秒
        min_step_ns = 300_000_000

        # 第一個點至少 0.3 秒後
        first_point_ns = 300_000_000

        last_ns = -1

        for i, point in enumerate(traj.points):
            original_ns = (
                point.time_from_start.sec * 1_000_000_000
                + point.time_from_start.nanosec
            )

            current_ns = int(original_ns * time_scale_factor)

            if i == 0 and current_ns < first_point_ns:
                current_ns = first_point_ns

            if i > 0 and current_ns <= last_ns + min_step_ns:
                current_ns = last_ns + min_step_ns

            point.time_from_start.sec = int(current_ns // 1_000_000_000)
            point.time_from_start.nanosec = int(current_ns % 1_000_000_000)

            if len(point.velocities) > 0:
                point.velocities = [v / time_scale_factor for v in point.velocities]

            if len(point.accelerations) > 0:
                point.accelerations = [
                    a / (time_scale_factor * time_scale_factor)
                    for a in point.accelerations
                ]

            last_ns = current_ns

        return traj

    def validate_trajectory(self, traj, arm_name):
        """
        檢查拆出來的 trajectory 是否合理。
        """
        if traj is None:
            return False, f'{arm_name} trajectory is None.'

        if len(traj.joint_names) == 0:
            return False, f'{arm_name} trajectory has no joint names.'

        if len(traj.points) == 0:
            return False, f'{arm_name} trajectory has no points.'

        joint_count = len(traj.joint_names)

        for i, point in enumerate(traj.points):
            if len(point.positions) != joint_count:
                return False, (
                    f'{arm_name} point {i} positions length mismatch. '
                    f'positions={len(point.positions)}, joints={joint_count}'
                )

        return True, ''

    def make_safe_two_point_trajectory(self, traj, controller_name):

        # [FORCE_DISABLE_SAFE_2POINT] 關閉 2-point，保留 MoveIt2 原始多點避障路徑

        self.get_logger().info(

            f'[FORCE_DISABLE_SAFE_2POINT] keep original trajectory for {controller_name}, points={len(traj.points)}'

        )

        return traj
        """
        將 MoveIt 送來的多點 trajectory 轉成乾淨的 2-point trajectory。
        目的：
        1. 避免 MoveIt 原本 trajectory 的時間格式讓 controller 瞬間判定完成
        2. 模仿 direct test 的穩定送法
        3. 保留 MoveIt 規劃的最後目標位置
        """
        if traj is None or len(traj.points) < 2:
            return traj

        # 第一點一定要用實體目前 joint state，不能用 MoveIt trajectory 的舊 first point。
        # direct test 能成功的關鍵就是 start_positions 來自 /joint_states。
        current_pos = self.get_current_positions_for_joints(list(traj.joint_names))

        if current_pos is not None:
            first_pos = list(current_pos)
            self.get_logger().info(
                f'{controller_name} using REAL current joint state as first point: {first_pos}'
            )
        else:
            first_pos = list(traj.points[0].positions)
            self.get_logger().warn(
                f'{controller_name} could not read current joint state, using trajectory first point.'
            )

        last_pos = list(traj.points[-1].positions)

        if len(first_pos) == 0 or len(last_pos) == 0:
            return traj

        if len(first_pos) != len(last_pos):
            return traj

        deltas = [abs(last_pos[i] - first_pos[i]) for i in range(len(first_pos))]
        max_delta = max(deltas) if len(deltas) > 0 else 0.0

        # 如果位移幾乎為 0，就不改
        if max_delta < 0.001:
            self.get_logger().warn(
                f'{controller_name} max_delta is very small: {max_delta}. '
                f'Trajectory may not visibly move.'
            )
            return traj

        # 依照位移大小給安全時間，至少 6 秒，最多 15 秒
        duration_sec = max(6.0, min(15.0, max_delta * 6.0))

        safe_traj = JointTrajectory()
        safe_traj.header = copy.deepcopy(traj.header)
        safe_traj.joint_names = list(traj.joint_names)

        p0 = JointTrajectoryPoint()
        p0.positions = first_pos
        p0.velocities = [0.0] * len(first_pos)
        p0.accelerations = [0.0] * len(first_pos)
        p0.time_from_start = Duration(seconds=5.0).to_msg()

        p1 = JointTrajectoryPoint()
        p1.positions = last_pos
        p1.velocities = [0.0] * len(last_pos)
        p1.accelerations = [0.0] * len(last_pos)
        p1.time_from_start = Duration(seconds=duration_sec).to_msg()

        safe_traj.points = [p0, p1]

        self.get_logger().info(
            f'{controller_name} converted to safe 2-point trajectory. '
            f'max_delta={max_delta}, duration={duration_sec}s'
        )

        return safe_traj


    def send_to_controller(self, client, controller_name, traj):
        """
        把拆好的 trajectory 送到左手或右手 joint_trajectory_controller。
        """
        self.get_logger().info(f'Waiting for {controller_name} action server...')

        if not client.wait_for_server(timeout_sec=5.0):
            return False, f'{controller_name} action server not available.'

        # 將 MoveIt trajectory 轉成穩定的 2-point trajectory，避免 controller 瞬間完成但實體沒動。
        # [FORCE_DISABLE_SAFE_2POINT] 不再轉成 2-point，保留 MoveIt2 原始 trajectory
        # traj = self.make_safe_two_point_trajectory(traj, controller_name)

        # 每次真正送給 controller 前，都重新設定 header stamp。
        # [COMMON_STAMP_FIX] 不在 send_to_controller 裡重新產生 stamp，保留外層 common start_stamp
        # fresh_start_stamp = (self.get_clock().now() + Duration(seconds=0.5)).to_msg()
        # [SYNC_FIX] 只更新 header.stamp，保留 MoveIt2 原本的 time_from_start
        # [COMMON_STAMP_FIX] 不覆蓋外層 common start_stamp
        # traj.header.stamp = fresh_start_stamp
        # traj = self.refresh_trajectory_time(traj, fresh_start_stamp)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info(f'Sending trajectory to {controller_name}...')
        self.get_logger().info(f'{controller_name} joints: {list(traj.joint_names)}')
        self.get_logger().info(
            f'{controller_name} stamp: '
            f'{traj.header.stamp.sec}.{traj.header.stamp.nanosec}'
        )
        self.get_logger().info(f'{controller_name} points: {len(traj.points)}')
        if len(traj.points) > 0:
            first_pos = list(traj.points[0].positions)
            last_pos = list(traj.points[-1].positions)

            deltas = [
                abs(last_pos[i] - first_pos[i])
                for i in range(min(len(first_pos), len(last_pos)))
            ]

            max_delta = max(deltas) if len(deltas) > 0 else 0.0

            self.get_logger().info(f'{controller_name} first positions: {first_pos}')
            self.get_logger().info(f'{controller_name} last positions:  {last_pos}')
            self.get_logger().info(f'{controller_name} max joint delta: {max_delta}')

        send_goal_future = client.send_goal_async(goal_msg)
        child_goal_handle = self.wait_for_future(send_goal_future, timeout_sec=10.0)

        if child_goal_handle is None:
            return False, f'{controller_name} send goal timeout.'

        if not child_goal_handle.accepted:
            return False, f'{controller_name} rejected the goal.'

        self.get_logger().info(f'{controller_name} accepted the goal.')

        result_future = child_goal_handle.get_result_async()
        result_response = self.wait_for_future(result_future, timeout_sec=120.0)

        if result_response is None:
            return False, f'{controller_name} execution timeout.'

        child_result = result_response.result

        if child_result.error_code != 0:
            return False, (
                f'{controller_name} execution failed. '
                f'error_code={child_result.error_code}, '
                f'error_string={child_result.error_string}'
            )

        self.get_logger().info(f'{controller_name} execution completed.')
        return True, ''

    def execute_callback(self, goal_handle):
        """
        MoveIt Execute 送進來後，會進到這裡。
        """
        self.get_logger().info('Received trajectory from MoveIt.')

        request = goal_handle.request
        incoming_traj = request.trajectory

        self.get_logger().info(f'Incoming joints: {list(incoming_traj.joint_names)}')
        self.get_logger().info(f'Incoming points: {len(incoming_traj.points)}')

        if len(incoming_traj.joint_names) == 0:
            msg = 'Incoming trajectory has no joint names.'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(error_code=-2, error_string=msg)

        if len(incoming_traj.points) == 0:
            msg = 'Incoming trajectory has no points.'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(error_code=-1, error_string=msg)

        # 拆左右手 trajectory
        left_traj = self.build_sub_trajectory(incoming_traj, self.left_joint_names)
        right_traj = self.build_sub_trajectory(incoming_traj, self.right_joint_names)

        has_left = left_traj is not None
        has_right = right_traj is not None

        if not has_left and not has_right:
            msg = 'No left or right joints found in incoming trajectory.'
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(error_code=-2, error_string=msg)

        # 讓左右手用同一個新的開始時間，避免 timestamp 太舊
        # 設為現在後 1 秒，給 action 傳送與 controller 接收一些緩衝時間
        start_stamp = (self.get_clock().now() + Duration(seconds=5.0)).to_msg()

        if has_left:
            self.get_logger().info('Splitting left trajectory...')
            left_traj = self.refresh_trajectory_time(left_traj, start_stamp)
            ok, msg = self.validate_trajectory(left_traj, 'left')
            if not ok:
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(error_code=-1, error_string=msg)

        if has_right:
            self.get_logger().info('Splitting right trajectory...')
            right_traj = self.refresh_trajectory_time(right_traj, start_stamp)

            # [SPLIT_CHECK] log trajectory point counts
            def _traj_duration_sec(_traj):
                if not _traj.points:
                    return 0.0
                _t = _traj.points[-1].time_from_start
                return float(_t.sec) + float(_t.nanosec) / 1e9
            self.get_logger().info(f"[SPLIT_CHECK] left points={len(left_traj.points)}, duration={_traj_duration_sec(left_traj):.3f}s")
            self.get_logger().info(f"[SPLIT_CHECK] right points={len(right_traj.points)}, duration={_traj_duration_sec(right_traj):.3f}s")
            ok, msg = self.validate_trajectory(right_traj, 'right')
            if not ok:
                self.get_logger().error(msg)
                goal_handle.abort()
                return self.make_result(error_code=-1, error_string=msg)

        errors = []
        results = {}

        def run_child(name, client, traj):
            self.get_logger().info(f'Starting parallel execution: {name}')
            ok, msg = self.send_to_controller(client, name, traj)
            results[name] = (ok, msg)

        threads = []

        # 左右手同時送出 trajectory，不再等左手完成才送右手
        if has_left:
            t_left = threading.Thread(
                target=run_child,
                args=(self.left_client, self.left_client, left_traj)
            )
            # 修正 name 參數
            t_left = threading.Thread(
                target=run_child,
                args=('left joint_trajectory_controller', self.left_client, left_traj)
            )
            threads.append(t_left)

        if has_right:
            t_right = threading.Thread(
                target=run_child,
                args=('right joint_trajectory_controller', self.right_client, right_traj)
            )
            threads.append(t_right)

        for t in threads:
            t.start()

        for t in threads:
            t.join()

        for name, (ok, msg) in results.items():
            if not ok:
                self.get_logger().error(msg)
                errors.append(msg)

        if len(errors) > 0:
            final_msg = ' | '.join(errors)
            self.get_logger().error(f'Trajectory execution failed: {final_msg}')
            goal_handle.abort()
            return self.make_result(error_code=-1, error_string=final_msg)

        self.get_logger().info('Parallel dual-arm trajectory execution succeeded.')
        goal_handle.succeed()
        return self.make_result(error_code=FollowJointTrajectory.Result.SUCCESSFUL, error_string='')


def main(args=None):
    rclpy.init(args=args)

    node = DualTrajectorySplitter()

    # 使用 MultiThreadedExecutor，避免 action server callback 等待 action client 時卡住
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Shutting down splitter node.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
