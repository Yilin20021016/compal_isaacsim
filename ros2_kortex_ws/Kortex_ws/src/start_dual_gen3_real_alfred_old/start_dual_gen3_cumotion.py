#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Kinova Gen3 雙臂實機 MoveIt 一鍵啟動器 (cuMotion GPU 加速測試版)

功能說明：
- 透過獨立的 Launch 腳本加載客製化的 joint limits (my_joint_limits.yaml)
- 限制右臂奇數軸（1, 3, 5, 7 軸），防止大擺幅亂轉與打結
- ⚠️ 此版本啟動 my_real_dual_cumotion.launch.py 以啟用 NVIDIA cuMotion
- 完全不污染/不修改 GitHub clone 的第三方 MoveIt 設定與硬體描述包

⚠️ WARNING: 涉及實體手臂動作。
首次執行請務必確認工作空間淨空，並建議配合 5_Capture_Segment_Register.py 的 SAFETY_TEST_MODE 進行空跑。
並且請確認已經安裝 isaac_ros_cumotion 以及生成了 .xrdf 檔案。
"""

import os
import time
import shutil
import subprocess

WS = os.path.expanduser("~/workspace/ros2_kortex_ws")
APF = os.path.expanduser("~/apf_test")

# ============================================================
# 速度與精簡啟動設定區
# ============================================================
START_DELAY_SEC = 2.0
TIME_SCALE_FACTOR = 6.0
MIN_MOTION_DURATION_SEC = 6.0

# 👈 True: 所有背景驅動在後台靜默啟動 (0 彈出視窗) | False: 彈出 11 個 gnome-terminal 視窗
QUIET_MODE = True 


def run_job(title: str, command: str, ws_dir=WS):
    """ 依據 QUIET_MODE 決定在背景靜默啟動或是彈出新終端機執行 """
    if not QUIET_MODE:
        # 傳統模式：彈出 gnome-terminal
        if shutil.which("gnome-terminal") is None:
            print("[ERROR] 找不到 gnome-terminal。請安裝：sudo apt install gnome-terminal")
            return
        full_command = f"""
{command}
exec bash
"""
        subprocess.Popen([
            "gnome-terminal",
            "--title", title,
            "--",
            "bash",
            "-lc",
            full_command
        ])
    else:
        # 精簡模式：背景執行，不彈出視窗，但將輸出重定向至 /tmp 方便除錯
        log_name = f"/tmp/ros2_job_{title.replace(' ', '_')}.log"
        print(f"  > [後台啟動] {title} (日誌：{log_name})...")
        log_file = open(log_name, "w")
        subprocess.Popen(
            ["bash", "-c", command],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            cwd=ws_dir,
            preexec_fn=os.setsid  # 使其在獨立的進程組中運行
        )


def main():
    print("=== 啟動 Kinova Gen3 雙臂實機環境 (cuMotion GPU 加速測試版) ===")
    print(f"Workspace: {WS}")
    print(f"APF folder: {APF}")
    print("")
    print("=== 速度與限制策略 ===")
    print(f"  > quiet_mode = {QUIET_MODE} (精簡模式：{('開啟，零額外視窗' if QUIET_MODE else '關閉，彈出11個視窗')})")
    print(f"  > time_scale_factor = {TIME_SCALE_FACTOR} (時間拉長 {TIME_SCALE_FACTOR} 倍)")
    print(f"  > min_motion_duration_sec = {MIN_MOTION_DURATION_SEC} 秒")
    print(f"  > 關節限制檔: scripts/start_dual_gen3_real_alfred/my_joint_limits.yaml")
    print("")

    print("=== 自動清理所有舊的手臂控制、驅動進程與視覺化視窗 ===")
    subprocess.run("pkill -f kortex", shell=True)
    subprocess.run("pkill -f joint_state", shell=True)
    subprocess.run("pkill -f trajectory", shell=True)
    subprocess.run("pkill -f merge_joint_states", shell=True)
    subprocess.run("pkill -f clean_trajectory_splitter", shell=True)
    subprocess.run("pkill -f trajectory_splitter_node", shell=True)
    subprocess.run("pkill -f move_group", shell=True)
    subprocess.run("pkill -f rviz2", shell=True)
    subprocess.run("pkill -f kortex_watchdog", shell=True)
    # 也清理可能遺留的 cumotion node
    subprocess.run("pkill -f cumotion", shell=True)
    print("正在等待系統釋放連線與連接埠...")
    time.sleep(2.0)

    print("=== 開始依序啟動雙臂系統組件 ===")

    run_job(
        "T1 left driver",
        f"""
cd {WS}
source /opt/ros/humble/setup.bash
source install/setup.bash

echo '=== Terminal 1：啟動左手 driver ==='
ros2 launch dual_kortex_bringup left_gen3.launch.py
"""
    )

    time.sleep(5)

    run_job(
        "T2 right driver",
        f"""
cd {WS}
source /opt/ros/humble/setup.bash
source install/setup.bash

echo '=== Terminal 2：啟動右手 driver ==='
ros2 launch {WS}/scripts/start_dual_gen3_real_alfred/my_right_gen3.launch.py
"""
    )

    print("等待左右手 controller_manager 啟動 (12秒)...")
    time.sleep(12)

    run_job(
        "T3 left joint_state_broadcaster",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 3：等待 /left/controller_manager ==='
until ros2 service list | grep -q '/left/controller_manager/list_controllers'; do
  echo '等待左手 controller_manager...'
  sleep 1
done

echo '=== 啟動左手 joint_state_broadcaster ==='
ros2 run controller_manager spawner joint_state_broadcaster \\
  --controller-manager /left/controller_manager \\
  --controller-type joint_state_broadcaster/JointStateBroadcaster

echo ''
echo '=== 檢查 /left/joint_states ==='
ros2 topic echo /left/joint_states --once
"""
    )

    time.sleep(3)

    run_job(
        "T4 right joint_state_broadcaster",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 4：等待 /right/controller_manager ==='
until ros2 service list | grep -q '/right/controller_manager/list_controllers'; do
  echo '等待右手 controller_manager...'
  sleep 1
done

echo '=== 啟動右手 joint_state_broadcaster ==='
ros2 run controller_manager spawner joint_state_broadcaster \\
  --controller-manager /right/controller_manager \\
  --controller-type joint_state_broadcaster/JointStateBroadcaster

echo ''
echo '=== 檢查 /right/joint_states ==='
ros2 topic echo /right/joint_states --once
"""
    )

    time.sleep(3)

    run_job(
        "T5 left trajectory controller",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 5：啟動左手 joint_trajectory_controller ==='
ros2 run controller_manager spawner joint_trajectory_controller \\
  --controller-manager /left/controller_manager \\
  --controller-type joint_trajectory_controller/JointTrajectoryController \\
  --param-file {WS}/src/dual_kortex_bringup/config/left_joint_trajectory_controller.yaml

echo ''
echo '=== 檢查左手 controller 狀態 ==='
ros2 control list_controllers -c /left/controller_manager
"""
    )

    time.sleep(3)

    run_job(
        "T6 right trajectory controller",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 6：啟動右手 joint_trajectory_controller ==='
ros2 run controller_manager spawner joint_trajectory_controller \\
  --controller-manager /right/controller_manager \\
  --controller-type joint_trajectory_controller/JointTrajectoryController \\
  --param-file {WS}/src/dual_kortex_bringup/config/right_joint_trajectory_controller.yaml

echo ''
echo '=== 檢查右手 controller 狀態 ==='
ros2 control list_controllers -c /right/controller_manager
"""
    )

    time.sleep(4)

    # ── 🚨 [關鍵] 啟動右手 Robotiq 2F-85 夾爪控制器 ──
    run_job(
        "T6.5 right gripper controller",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 6.5：啟動右手 Robotiq 夾爪控制器 ==='
ros2 run controller_manager spawner robotiq_gripper_controller \\
  --controller-manager /right/controller_manager \\
  --controller-type position_controllers/GripperActionController \\
  --param-file {WS}/scripts/start_dual_gen3_real_alfred/right_gripper_controller.yaml

echo ''
echo '=== 檢查右手夾爪 controller 狀態 ==='
ros2 control list_controllers -c /right/controller_manager
"""
    )

    time.sleep(3)

    run_job(
        "T7 joint states merger",
        f"""
cd {APF}
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 7：啟動 joint_states merger ==='
/usr/bin/python3 merge_joint_states.py
"""
    )

    time.sleep(3)

    run_job(
        "T8 check joint_states",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 8：檢查合併後 /joint_states ==='
ros2 topic echo /joint_states --once

echo ''
echo '=== 檢查 /joint_states publisher 數量 ==='
ros2 topic info /joint_states -v
"""
    )

    time.sleep(3)

    run_job(
        "T9 clean trajectory splitter slow mode",
        f"""
cd {WS}
source /opt/ros/humble/setup.bash
source install/setup.bash

echo '=== Terminal 9：啟動 clean trajectory splitter ==='
echo '使用 clean_trajectory_splitter'
echo 'start_delay_sec = {START_DELAY_SEC}'
echo 'time_scale_factor = {TIME_SCALE_FACTOR}'
echo 'min_motion_duration_sec = {MIN_MOTION_DURATION_SEC}'
echo ''

ros2 run dual_trajectory_splitter clean_trajectory_splitter --ros-args \\
  -p start_delay_sec:={START_DELAY_SEC} \\
  -p time_scale_factor:={TIME_SCALE_FACTOR} \\
  -p min_motion_duration_sec:={MIN_MOTION_DURATION_SEC}
"""
    )

    time.sleep(5)

    run_job(
        "T10 check action servers",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== Terminal 10：檢查 follow_joint_trajectory action server ==='
ros2 action list -t | grep follow_joint_trajectory || true

echo ''
echo '=== 檢查 /dual_arm_controller/follow_joint_trajectory ==='
ros2 action info /dual_arm_controller/follow_joint_trajectory || true
"""
    )

    time.sleep(3)

    # ── 啟動 MoveIt 規劃場景障礙物與保護區發布器 (常駐半透明橘色 Box) ──
    run_job(
        "T10.5 planning scene publisher",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== 啟動 Planning Scene 靜態障礙物與保護區發布器 ==='
/usr/bin/python3 {WS}/scripts/Dual_Arm_Control/02_publish_planning_scene.py {WS}/scripts/Protection_Zones
"""
    )

    time.sleep(2)

    # ── 啟動 Kortex 自動恢復看門狗 ──
    run_job(
        "kortex watchdog",
        f"""
source /opt/ros/humble/setup.bash
source {WS}/install/setup.bash

echo '=== 啟動 Kortex 自動恢復看門狗 ==='
/usr/bin/python3 {WS}/scripts/start_dual_gen3_real_alfred/kortex_watchdog.py
"""
    )

    time.sleep(1)

    # 🚨 關鍵修改：加載 Alfred 版 cuMotion 測試 Launch 檔
    custom_launch_path = os.path.join(WS, "scripts/start_dual_gen3_real_alfred/my_real_dual_cumotion.launch.py")
    
    if QUIET_MODE:
        print("\n🎉 [系統就緒] 正在主終端機啟動 MoveIt + RViz (cuMotion GPU 測試版)...")
        print(f"載入配置: {custom_launch_path}")
        print("💡 提示：此視窗為 MoveIt 日誌輸出端。如需關閉整個手臂控制環境，請直接在此按下 Ctrl+C 即可。\n")
        print("⚠️ 注意：若噴出找不到 isaac_ros_cumotion 的錯誤，代表環境尚未配置完成，請先安裝套件並產生 XRDF 檔案。")
        
        cmd = f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && ros2 launch {custom_launch_path}"
        try:
            subprocess.run(["bash", "-c", cmd], cwd=WS)
        except KeyboardInterrupt:
            print("\n[INFO] 偵測到 Ctrl+C 中斷，正在自動關閉並清理所有手臂控制進程...")
            subprocess.run("pkill -f kortex", shell=True)
            subprocess.run("pkill -f joint_state", shell=True)
            subprocess.run("pkill -f trajectory", shell=True)
            subprocess.run("pkill -f merge_joint_states", shell=True)
            subprocess.run("pkill -f clean_trajectory_splitter", shell=True)
            subprocess.run("pkill -f trajectory_splitter_node", shell=True)
            subprocess.run("pkill -f publish_planning_scene", shell=True)
            subprocess.run("pkill -f move_group", shell=True)
            subprocess.run("pkill -f rviz2", shell=True)
            subprocess.run("pkill -f kortex_watchdog", shell=True)
            subprocess.run("pkill -f cumotion", shell=True)
            print("所有手臂進程已關閉，環境清理完畢。")
    else:
        run_job(
            "T11 real dual cuMotion MoveIt RViz",
            f"""
cd {WS}
source /opt/ros/humble/setup.bash
source install/setup.bash

echo '=== Terminal 11：啟動實機 MoveIt + RViz (cuMotion 測試版) ==='
echo '載入：{custom_launch_path}'

ros2 launch {custom_launch_path}
"""
        )
        print("")
        print("=== 已依順序開啟所有 Terminal ===")
        print(f"已使用 cuMotion 版自訂限制 MoveIt launch。")
        print("")


if __name__ == "__main__":
    main()

