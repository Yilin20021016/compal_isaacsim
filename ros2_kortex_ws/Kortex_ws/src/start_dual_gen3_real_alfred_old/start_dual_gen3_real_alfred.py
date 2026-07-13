#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Kinova Gen3 雙臂實機 MoveIt 一鍵啟動器 (Alfred 關節限制覆寫版 - 主動輪詢極速版)

功能說明：
- 透過獨立的 Launch 腳本加載客製化的 joint limits (my_joint_limits.yaml)
- 限制右臂奇數軸（1, 3, 5, 7 軸），防止大擺幅亂轉與打結
- 使用主動輪詢 (Active Polling) 取代寫死的 time.sleep()，將啟動時間從 >45 秒大幅縮短至硬體實際所需時間。
- 完全不污染/不修改 GitHub clone 的第三方 MoveIt 設定與硬體描述包

⚠️ WARNING: 涉及實體手臂動作。
首次執行請務必確認工作空間淨空，並建議配合 5_Capture_Segment_Register.py 的 SAFETY_TEST_MODE 進行空跑。
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

# 👈 True: 所有背景驅動在後台靜默啟動 (0 彈出視窗) | False: 彈出多個 gnome-terminal 視窗
QUIET_MODE = True 

def run_job(title: str, command: str, ws_dir=WS):
    """ 依據 QUIET_MODE 決定在背景靜默啟動或是彈出新終端機執行 """
    if not QUIET_MODE:
        if shutil.which("gnome-terminal") is None:
            print("[ERROR] 找不到 gnome-terminal。請安裝：sudo apt install gnome-terminal")
            return
        full_command = f"{command}\nexec bash\n"
        subprocess.Popen([
            "gnome-terminal",
            "--title", title,
            "--",
            "bash",
            "-lc",
            full_command
        ])
    else:
        log_name = f"/tmp/ros2_job_{title.replace(' ', '_').replace('/', '_')}.log"
        print(f"  > [並行啟動] {title} (日誌：{log_name})...")
        log_file = open(log_name, "w")
        subprocess.Popen(
            ["bash", "-c", command],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            cwd=ws_dir,
            preexec_fn=os.setsid
        )

def wait_for_output(description: str, command: str, timeout: int = 30):
    """ 主動輪詢直到指令回傳成功 (Return Code == 0) """
    print(f"  ⏳ {description} (最多等待 {timeout} 秒)...")
    start_time = time.time()
    # 將 source 指令包進檢查指令中，以確保 ROS 2 環境變數已加載
    full_cmd = f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && {command}"
    while time.time() - start_time < timeout:
        res = subprocess.run(full_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, executable='/bin/bash')
        if res.returncode == 0:
            elapsed = time.time() - start_time
            print(f"  ✅ 就緒！(耗時 {elapsed:.1f} 秒)")
            return True
        time.sleep(0.5)
    print(f"  ❌ [警告] 等待超時: {description}")
    return False

def main():
    print("=== 啟動 Kinova Gen3 雙臂實機環境 (主動輪詢極速版) ===")
    print("自動清理舊的進程...")
    subprocess.run("pkill -f kortex_driver", shell=True)
    subprocess.run("pkill -f my_right_gen3.launch.py", shell=True)
    subprocess.run("pkill -f left_gen3.launch.py", shell=True)
    subprocess.run("pkill -f joint_state_broadcaster", shell=True)
    subprocess.run("pkill -f joint_trajectory_controller", shell=True)
    subprocess.run("pkill -f robotiq_gripper_controller", shell=True)
    subprocess.run("pkill -f merge_joint_states", shell=True)
    subprocess.run("pkill -f clean_trajectory_splitter", shell=True)
    subprocess.run("pkill -f move_group", shell=True)
    subprocess.run("pkill -f rviz2", shell=True)
    subprocess.run("pkill -f kortex_watchdog.py", shell=True)
    print("等待 2 秒釋放連接埠...")
    time.sleep(2.0)

    # 共用的 Source 指令前綴
    base_cmd = f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && "

    print("\n=== [階段 1] 啟動底層通訊 Driver ===")
    run_job("T1 left driver", base_cmd + "ros2 launch dual_kortex_bringup left_gen3.launch.py")
    run_job("T2 right driver", base_cmd + f"ros2 launch {WS}/scripts/start_dual_gen3_real_alfred/my_right_gen3.launch.py")
    
    # 關鍵：同時主動等待兩臂的 Controller Manager 上線
    wait_for_output("等待左臂 Controller Manager", "ros2 service list | grep -q '/left/controller_manager/list_controllers'", 45)
    wait_for_output("等待右臂 Controller Manager", "ros2 service list | grep -q '/right/controller_manager/list_controllers'", 45)

    print("\n=== [階段 2] 啟動所有 Controller Spawners ===")
    # 由於 Controller Manager 已上線，可以放心並發啟動所有 Spawner
    run_job("T3 left joint_state_broadcaster", base_cmd + "ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /left/controller_manager --controller-type joint_state_broadcaster/JointStateBroadcaster")
    run_job("T4 right joint_state_broadcaster", base_cmd + "ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /right/controller_manager --controller-type joint_state_broadcaster/JointStateBroadcaster")
    run_job("T5 left trajectory controller", base_cmd + f"ros2 run controller_manager spawner joint_trajectory_controller --controller-manager /left/controller_manager --controller-type joint_trajectory_controller/JointTrajectoryController --param-file {WS}/src/dual_kortex_bringup/config/left_joint_trajectory_controller.yaml")
    run_job("T6 right trajectory controller", base_cmd + f"ros2 run controller_manager spawner joint_trajectory_controller --controller-manager /right/controller_manager --controller-type joint_trajectory_controller/JointTrajectoryController --param-file {WS}/src/dual_kortex_bringup/config/right_joint_trajectory_controller.yaml")
    run_job("T6.5 right gripper controller", base_cmd + f"ros2 run controller_manager spawner robotiq_gripper_controller --controller-manager /right/controller_manager --controller-type position_controllers/GripperActionController --param-file {WS}/scripts/start_dual_gen3_real_alfred/right_gripper_controller.yaml")

    # 必須等待 Joint States 發布後才能啟動 Merger
    wait_for_output("等待左臂 joint_states", "ros2 topic list | grep -q '^/left/joint_states$'", 30)
    wait_for_output("等待右臂 joint_states", "ros2 topic list | grep -q '^/right/joint_states$'", 30)

    print("\n=== [階段 3] 啟動 Merger 與 Splitter ===")
    run_job("T7 joint states merger", base_cmd + f"cd {APF} && /usr/bin/python3 merge_joint_states.py")
    wait_for_output("等待全域 /joint_states", "ros2 topic list | grep -q '^/joint_states$'", 20)

    # 啟動 Splitter 並等待 Action Server 出現
    run_job("T9 clean trajectory splitter", base_cmd + f"ros2 run dual_trajectory_splitter clean_trajectory_splitter --ros-args -p start_delay_sec:={START_DELAY_SEC} -p time_scale_factor:={TIME_SCALE_FACTOR} -p min_motion_duration_sec:={MIN_MOTION_DURATION_SEC}")
    wait_for_output("等待 Action Server (/dual_arm_controller)", "ros2 action list | grep -q '^/dual_arm_controller/follow_joint_trajectory$'", 20)

    print("\n=== [階段 4] 啟動輔助節點與 MoveIt2 ===")
    run_job("T10.5 planning scene publisher", base_cmd + f"/usr/bin/python3 {WS}/scripts/Dual_Arm_Control/02_publish_planning_scene.py {WS}/scripts/Protection_Zones")
    run_job("kortex watchdog", base_cmd + f"/usr/bin/python3 {WS}/scripts/start_dual_gen3_real_alfred/kortex_watchdog.py")

    # 短暫等待保證輔助節點順利註冊
    time.sleep(1.0) 
    
    custom_launch_path = os.path.join(WS, "scripts/start_dual_gen3_real_alfred/my_real_dual_moveit.launch.py")
    
    if QUIET_MODE:
        print("\n🎉 [系統就緒] 正在主終端機啟動 MoveIt + RViz...")
        print("💡 提示：此視窗為 MoveIt 日誌輸出端。如需關閉整個手臂控制環境，請直接在此按下 Ctrl+C 即可。\n")
        cmd = base_cmd + f"ros2 launch {custom_launch_path}"
        try:
            subprocess.run(["bash", "-c", cmd], cwd=WS)
        except KeyboardInterrupt:
            print("\n[INFO] 偵測到 Ctrl+C 中斷，正在自動關閉並清理所有手臂控制進程...")
            subprocess.run("pkill -f kortex_driver", shell=True)
            subprocess.run("pkill -f joint_state", shell=True)
            subprocess.run("pkill -f trajectory", shell=True)
            subprocess.run("pkill -f merge_joint_states", shell=True)
            subprocess.run("pkill -f clean_trajectory_splitter", shell=True)
            subprocess.run("pkill -f move_group", shell=True)
            subprocess.run("pkill -f rviz2", shell=True)
            subprocess.run("pkill -f kortex_watchdog.py", shell=True)
            print("所有手臂進程已關閉，環境清理完畢。")
    else:
        run_job("T11 real dual MoveIt RViz", base_cmd + f"ros2 launch {custom_launch_path}")
        print("\n=== 已依順序開啟所有 Terminal ===")

if __name__ == "__main__":
    main()
