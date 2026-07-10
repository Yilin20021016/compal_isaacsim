#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
⚠️ WARNING: Kortex 雙臂實機控制器自動復位看門狗 (Watchdog)
此腳本用於在背景監控右手手臂的狀態。一旦偵測到手臂從急停或手動拖動中恢復（綠燈，出現狀態不匹配），
會自動呼叫 ROS2 Control 服務重啟控制器，以安全、平滑且免人工輸入地恢復系統。
"""

import os
import time
import subprocess

LOG_PATH = "/tmp/ros2_job_T2_right_driver.log"
TRIGGER_STRING = "Fault was not recognized on the robot but combination of Control Mode and Active State"
RECOVERY_COOLDOWN_SEC = 5.0  # 每次復原後冷卻時間，防止重複觸發

def restart_controllers():
    print("\n[WATCHDOG] 🚨 偵測到手臂已恢復為 Ready 狀態，但控制器存在不匹配。開始執行自動安全復歸...")
    try:
        # 停用控制器 (個別停用，忽略已停用報錯)
        subprocess.run(["ros2", "control", "set_controller_state", "-c", "/right/controller_manager", "joint_trajectory_controller", "inactive"], capture_output=True, timeout=5)
        subprocess.run(["ros2", "control", "set_controller_state", "-c", "/right/controller_manager", "robotiq_gripper_controller", "inactive"], capture_output=True, timeout=5)
        
        # 啟用控制器
        res1 = subprocess.run(["ros2", "control", "set_controller_state", "-c", "/right/controller_manager", "joint_trajectory_controller", "active"], capture_output=True, text=True, timeout=5)
        res2 = subprocess.run(["ros2", "control", "set_controller_state", "-c", "/right/controller_manager", "robotiq_gripper_controller", "active"], capture_output=True, text=True, timeout=5)
        
        if res1.returncode == 0 and res2.returncode == 0:
            print("[WATCHDOG] ✅ 自動安全復位成功！控制器已完成無彈跳同步。")
        else:
            print("[WATCHDOG] ⚠️ 自動復位已執行，但有部分控制器狀態未達預期：")
            print(f"  - joint_trajectory_controller: {'成功' if res1.returncode == 0 else '失敗'}")
            print(f"  - robotiq_gripper_controller: {'成功' if res2.returncode == 0 else '失敗'}")
    except Exception as e:
        print(f"[WATCHDOG] ❌ 執行自動復位時發生異常：{e}")

def main():
    # ⚠️ WARNING: 此看門狗會自動切換與重啟控制器。請確保在實機測試時操作人員手持 E-Stop 鈕，以確保安全。
    print("[WATCHDOG] 🐕 Kortex 恢復看門狗已啟動，開始監視日誌...")
    
    last_recovery_time = 0.0
    first_trigger_time = None
    last_line_time = 0.0
    
    while True:
        if not os.path.exists(LOG_PATH):
            time.sleep(1.0)
            continue
            
        try:
            with open(LOG_PATH, "r", errors="ignore") as f:
                # 移動到檔案末尾，不讀取歷史舊日誌
                f.seek(0, os.SEEK_END)
                
                while True:
                    # 檢查檔案是否被截斷（例如重啟時）
                    current_position = f.tell()
                    try:
                        file_size = os.path.getsize(LOG_PATH)
                    except FileNotFoundError:
                        break
                        
                    if file_size < current_position:
                        # 檔案被重新寫入，重新讀取
                        print("[WATCHDOG] 🔄 偵測到日誌檔案被截斷/重置，重新載入...")
                        break
                        
                    line = f.readline()
                    if not line:
                        now = time.time()
                        # 若已超過 0.5 秒沒有新日誌，代表 debug 警告流已結束，重置觸發計時器
                        if first_trigger_time is not None and (now - last_line_time > 0.5):
                            first_trigger_time = None
                        time.sleep(0.02)
                        continue
                        
                    if TRIGGER_STRING in line:
                        now = time.time()
                        last_line_time = now
                        if first_trigger_time is None:
                            first_trigger_time = now
                        else:
                            elapsed = now - first_trigger_time
                            # ⚠️ WARNING: 當偵測到持續列印此警告大於 1.2 秒時才執行復位，避免正常切換控制器時的短暫狀態不一致觸發誤判。
                            if elapsed >= 1.2:
                                if now - last_recovery_time > RECOVERY_COOLDOWN_SEC:
                                    restart_controllers()
                                    last_recovery_time = time.time()
                                    # 讀取完後清除目前緩衝，防止連鎖觸發
                                    f.seek(0, os.SEEK_END)
                                    first_trigger_time = None
                            
        except KeyboardInterrupt:
            print("[WATCHDOG] 🛑 看門狗已手動關閉。")
            break
        except Exception as e:
            print(f"[WATCHDOG] ⚠️ 讀取日誌時發生異常：{e}。1秒後重試...")
            time.sleep(1.0)

if __name__ == "__main__":
    main()
