#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Clean Dual Trajectory Splitter

功能：
1. 接收 MoveIt2 both_arms 的 FollowJointTrajectory goal
2. 只拆 joint trajectory，不改 MoveIt2 規劃出的路徑
3. left_joint_* 送到左臂 controller
4. right_joint_* 送到右臂 controller
5. 保留所有 trajectory points
6. 保留 time_from_start 的相對順序
7. 可用 time_scale_factor 等比例放慢
8. Execute 後延遲 start_delay_sec 秒再開始實機動作

重要：
- 不做 safe 2-point
- 不丟掉中間 waypoint
- 不用 real current joint state 覆蓋第一點
"""

import copy
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class CleanDualTrajectorySplitter(Node):
    def __init__(self):
        super().__init__("trajectory_splitter_node_clean")

        # ===== 可調參數 =====
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("time_scale_factor", 40.0)
        self.declare_parameter("min_motion_duration_sec", 10.0)

        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.time_scale_factor = float(self.get_parameter("time_scale_factor").value)
        self.min_motion_duration_sec = float(self.get_parameter("min_motion_duration_sec").value)

        if self.time_scale_factor <= 0.0:
            self.get_logger().warn("time_scale_factor <= 0，已改成 1.0")
            self.time_scale_factor = 1.0

        # ===== 左右臂 joint 名稱 =====
        self.left_joint_names = [
            "left_joint_1",
            "left_joint_2",
            "left_joint_3",
            "left_joint_4",
            "left_joint_5",
            "left_joint_6",
            "left_joint_7",
        ]

        self.right_joint_names = [
            "right_joint_1",
            "right_joint_2",
            "right_joint_3",
            "right_joint_4",
            "right_joint_5",
            "right_joint_6",
            "right_joint_7",
        ]

        # ===== 真實左右 controller action client =====
        self.left_action_name = "/left/joint_trajectory_controller/follow_joint_trajectory"
        self.right_action_name = "/right/joint_trajectory_controller/follow_joint_trajectory"

        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.left_action_name,
        )

        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.right_action_name,
        )

        # ===== 提供給 MoveIt2 的 action server 名稱 =====
        # 保留多個常見名字，避免你 MoveIt2 controller yaml 指到不同名稱
        self.server_names = [
            "/dual_arm_controller/follow_joint_trajectory",
            "/dual_trajectory_splitter/follow_joint_trajectory",
            "/trajectory_splitter/follow_joint_trajectory",
            "/both_arms_controller/follow_joint_trajectory",
            "/joint_trajectory_controller/follow_joint_trajectory",
        ]

        self.servers = []
        for name in self.server_names:
            server = ActionServer(
                self,
                FollowJointTrajectory,
                name,
                execute_callback=self.execute_callback,
            )
            self.servers.append(server)
            self.get_logger().info(f"Action server ready: {name}")

        self.get_logger().info("==============================================")
        self.get_logger().info("CLEAN dual trajectory splitter loaded")
        self.get_logger().info(f"left controller : {self.left_action_name}")
        self.get_logger().info(f"right controller: {self.right_action_name}")
        self.get_logger().info(f"start_delay_sec : {self.start_delay_sec}")
        self.get_logger().info(f"time_scale_factor: {self.time_scale_factor}")
        self.get_logger().info(f"min_motion_duration_sec: {self.min_motion_duration_sec}")
        self.latest_joint_positions = {}
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )
        self.get_logger().info("Current-state alignment enabled: first point follows /joint_states.")
        self.latest_joint_positions = {}
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )
        self.get_logger().info("Current-state alignment enabled.")
        self.get_logger().info("Position-only trajectory enabled.")
        self.get_logger().info("NO safe 2-point. NO waypoint dropping.")
        self.get_logger().info("==============================================")

    # ------------------------------------------------------------
    # 小工具：等待 future，不在 callback 裡重新 spin
    # ------------------------------------------------------------
    def wait_for_future(self, future, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok():
            if future.done():
                return True
            if time.time() - start > timeout_sec:
                return False
            time.sleep(0.01)
        return False

    # ------------------------------------------------------------
    # 小工具：建立 FollowJointTrajectory Result
    # ------------------------------------------------------------
    def make_result(self, error_code=FollowJointTrajectory.Result.SUCCESSFUL, error_string=""):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    # ------------------------------------------------------------
    # 小工具：選出對應 joint 的資料
    # ------------------------------------------------------------
    def select_values(self, values, indices: List[int], full_count: int):
        if len(values) == 0:
            return []
        if len(values) != full_count:
            # 有些 trajectory 不帶 velocities / accelerations / effort
            return []
        return [values[i] for i in indices]

    # ------------------------------------------------------------
    # 讀取最新 joint_states
    # ------------------------------------------------------------
    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = pos

    # ------------------------------------------------------------
    # 只把第一個 point 對齊實機目前 joint state，不刪除任何 MoveIt2 waypoint
    # ------------------------------------------------------------
    def align_first_point_to_current_state(self, traj: JointTrajectory) -> JointTrajectory:
        if traj is None or len(traj.points) == 0:
            return traj

        missing = [jn for jn in traj.joint_names if jn not in self.latest_joint_positions]
        if missing:
            self.get_logger().warn(
                f"[CURRENT_ALIGN] missing current joint states: {missing}. Keep original first point."
            )
            return traj

        traj = copy.deepcopy(traj)

        current_positions = [
            self.latest_joint_positions[jn]
            for jn in traj.joint_names
        ]

        traj.points[0].positions = current_positions
        traj.points[0].velocities = [0.0] * len(current_positions)
        traj.points[0].accelerations = [0.0] * len(current_positions)

        self.get_logger().info(
            f"[CURRENT_ALIGN] aligned first point to real current state for joints: {traj.joint_names}"
        )

        return traj


    # ------------------------------------------------------------
    # 拆 trajectory：保留所有 points，不改路徑
    # ------------------------------------------------------------
    def build_sub_trajectory(
        self,
        incoming_traj: JointTrajectory,
        target_joint_names: List[str],
    ) -> Optional[JointTrajectory]:

        full_joint_names = list(incoming_traj.joint_names)
        full_count = len(full_joint_names)

        indices = []
        selected_joint_names = []

        for jn in target_joint_names:
            if jn in full_joint_names:
                indices.append(full_joint_names.index(jn))
                selected_joint_names.append(jn)

        if len(indices) == 0:
            return None

        sub_traj = JointTrajectory()
        sub_traj.header = copy.deepcopy(incoming_traj.header)
        sub_traj.joint_names = selected_joint_names

        for point in incoming_traj.points:
            new_point = JointTrajectoryPoint()

            new_point.positions = self.select_values(
                point.positions,
                indices,
                full_count,
            )

            new_point.velocities = self.select_values(
                point.velocities,
                indices,
                full_count,
            )

            new_point.accelerations = self.select_values(
                point.accelerations,
                indices,
                full_count,
            )

            new_point.effort = self.select_values(
                point.effort,
                indices,
                full_count,
            )

            # 保留 MoveIt2 原始 time_from_start，之後再等比例放慢
            new_point.time_from_start = copy.deepcopy(point.time_from_start)

            sub_traj.points.append(new_point)

        return sub_traj

    # ------------------------------------------------------------
    # 只改時間，不改 positions / points 數量
    # ------------------------------------------------------------
    def apply_common_timing(
        self,
        traj: JointTrajectory,
        start_stamp,
    ) -> JointTrajectory:

        traj = copy.deepcopy(traj)
        traj.header.stamp = start_stamp

        scale = self.time_scale_factor

        # 第一個點給 controller 一點緩衝
        offset_ns = 500_000_000

        # waypoint 至少間隔 0.1 秒
        min_step_ns = 100_000_000

        last_ns = -1

        # 第一次：依 time_scale_factor 放慢
        for point in traj.points:
            original_ns = (
                int(point.time_from_start.sec) * 1_000_000_000
                + int(point.time_from_start.nanosec)
            )

            scaled_ns = int(original_ns * scale) + offset_ns

            if scaled_ns <= last_ns:
                scaled_ns = last_ns + min_step_ns

            point.time_from_start.sec = int(scaled_ns // 1_000_000_000)
            point.time_from_start.nanosec = int(scaled_ns % 1_000_000_000)

            # position-only：保留 positions，不傳 velocities / accelerations / effort
            point.velocities = []
            point.accelerations = []
            point.effort = []

            last_ns = scaled_ns

        # 第二次：如果總時間小於 min_motion_duration_sec，就整體拉長到 15 秒
        if traj.points:
            final_t = traj.points[-1].time_from_start
            final_ns = int(final_t.sec) * 1_000_000_000 + int(final_t.nanosec)
            min_total_ns = int(self.min_motion_duration_sec * 1_000_000_000)

            if final_ns < min_total_ns and final_ns > 0:
                stretch = min_total_ns / final_ns

                self.get_logger().info(
                    f"[MIN_DURATION] stretch trajectory duration from {final_ns / 1e9:.3f}s "
                    f"to {self.min_motion_duration_sec:.3f}s, factor={stretch:.3f}"
                )

                last_ns = -1
                for point in traj.points:
                    current_ns = (
                        int(point.time_from_start.sec) * 1_000_000_000
                        + int(point.time_from_start.nanosec)
                    )

                    new_ns = int(current_ns * stretch)

                    if new_ns <= last_ns:
                        new_ns = last_ns + min_step_ns

                    point.time_from_start.sec = int(new_ns // 1_000_000_000)
                    point.time_from_start.nanosec = int(new_ns % 1_000_000_000)

                    last_ns = new_ns

        self.get_logger().info(
            f"[POSITION_ONLY_TIMING] points={len(traj.points)}, duration={self.duration_sec(traj):.3f}s"
        )

        return traj

    def duration_sec(self, traj: JointTrajectory) -> float:
        if len(traj.points) == 0:
            return 0.0
        t = traj.points[-1].time_from_start
        return float(t.sec) + float(t.nanosec) / 1e9

    # ------------------------------------------------------------
    # 送 trajectory 到單邊 controller
    # ------------------------------------------------------------
    def send_to_controller(
        self,
        client: ActionClient,
        controller_name: str,
        traj: JointTrajectory,
    ) -> Tuple[bool, str]:

        self.get_logger().info(f"Waiting for {controller_name} action server...")

        if not client.wait_for_server(timeout_sec=5.0):
            return False, f"{controller_name} action server not available."

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info(f"Sending trajectory to {controller_name}")
        self.get_logger().info(f"{controller_name} joints: {list(traj.joint_names)}")
        self.get_logger().info(f"{controller_name} points: {len(traj.points)}")
        self.get_logger().info(f"{controller_name} duration: {self.duration_sec(traj):.3f}s")

        if len(traj.points) > 0:
            self.get_logger().info(
                f"{controller_name} first positions: {list(traj.points[0].positions)}"
            )
            self.get_logger().info(
                f"{controller_name} last positions: {list(traj.points[-1].positions)}"
            )

        send_future = client.send_goal_async(goal_msg)

        if not self.wait_for_future(send_future, timeout_sec=10.0):
            return False, f"{controller_name} send goal timeout."

        goal_handle = send_future.result()

        if goal_handle is None:
            return False, f"{controller_name} goal handle is None."

        if not goal_handle.accepted:
            return False, f"{controller_name} rejected the goal."

        self.get_logger().info(f"{controller_name} accepted the goal.")

        result_future = goal_handle.get_result_async()

        # duration + buffer
        timeout_sec = max(30.0, self.duration_sec(traj) + 10.0)

        if not self.wait_for_future(result_future, timeout_sec=timeout_sec):
            return False, f"{controller_name} execution timeout."

        result = result_future.result()

        if result is None:
            return False, f"{controller_name} result is None."

        error_code = result.result.error_code
        error_string = result.result.error_string

        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, f"{controller_name} failed: code={error_code}, msg={error_string}"

        self.get_logger().info(f"{controller_name} execution completed.")
        return True, ""

    # ------------------------------------------------------------
    # MoveIt2 呼叫進來的主 callback
    # ------------------------------------------------------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("Received goal request from MoveIt2.")

        incoming_traj = goal_handle.request.trajectory

        self.get_logger().info(f"Incoming joints: {list(incoming_traj.joint_names)}")
        self.get_logger().info(f"Incoming points: {len(incoming_traj.points)}")

        if len(incoming_traj.points) == 0:
            msg = "Incoming trajectory has zero points."
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string=msg,
            )

        left_traj = self.build_sub_trajectory(
            incoming_traj,
            self.left_joint_names,
        )

        right_traj = self.build_sub_trajectory(
            incoming_traj,
            self.right_joint_names,
        )

        has_left = left_traj is not None and len(left_traj.points) > 0
        has_right = right_traj is not None and len(right_traj.points) > 0

        if not has_left and not has_right:
            msg = "No left or right joints found in incoming trajectory."
            self.get_logger().error(msg)
            goal_handle.abort()
            return self.make_result(
                error_code=FollowJointTrajectory.Result.INVALID_JOINTS,
                error_string=msg,
            )

        # 共同開始時間：按 Execute 後，延遲 start_delay_sec 秒才開始動
        common_start_stamp = (
            self.get_clock().now()
            + Duration(seconds=self.start_delay_sec)
        ).to_msg()

        if has_left:
            left_traj = self.align_first_point_to_current_state(left_traj)
            left_traj = self.apply_common_timing(left_traj, common_start_stamp)

        if has_right:
            right_traj = self.align_first_point_to_current_state(right_traj)
            right_traj = self.apply_common_timing(right_traj, common_start_stamp)

        if has_left:
            self.get_logger().info(
                f"[SPLIT_CHECK] left points={len(left_traj.points)}, "
                f"duration={self.duration_sec(left_traj):.3f}s"
            )

        if has_right:
            self.get_logger().info(
                f"[SPLIT_CHECK] right points={len(right_traj.points)}, "
                f"duration={self.duration_sec(right_traj):.3f}s"
            )

        import threading

        errors = []
        results = {}

        def run_child(name, client, traj):
            ok, msg = self.send_to_controller(client, name, traj)
            results[name] = (ok, msg)

        threads = []

        if has_left:
            t_left = threading.Thread(
                target=run_child,
                args=("left_joint_trajectory_controller", self.left_client, left_traj),
                daemon=True,
            )
            threads.append(t_left)

        if has_right:
            t_right = threading.Thread(
                target=run_child,
                args=("right_joint_trajectory_controller", self.right_client, right_traj),
                daemon=True,
            )
            threads.append(t_right)

        # 左右同時送出
        for t in threads:
            t.start()

        for t in threads:
            t.join()

        for name, (ok, msg) in results.items():
            if not ok:
                self.get_logger().error(msg)
                errors.append(msg)

        if len(errors) > 0:
            final_msg = " | ".join(errors)
            self.get_logger().error(f"Trajectory execution failed: {final_msg}")
            goal_handle.abort()
            return self.make_result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string=final_msg,
            )

        self.get_logger().info("Parallel dual-arm trajectory execution succeeded.")
        goal_handle.succeed()
        return self.make_result(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL,
            error_string="",
        )


def main(args=None):
    rclpy.init(args=args)

    node = CleanDualTrajectorySplitter()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Shutting down clean splitter.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
