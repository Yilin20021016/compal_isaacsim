# Kinova Gen3 雙臂實機環境啟動與配置指南

本資料夾（`start_dual_gen3_real_alfred`）包含了啟動並控制 **Kinova Gen3 雙臂機器人**（含右臂 Robotiq 2F-85 夾爪）所需的所有客製化啟動腳本、URDF 覆寫檔及硬體驅動配置。

透過本指南，廠商與開發人員可直接基於官方的 ROS 2 驅動程式（`ros2_kortex`），快速且無痛地在本地端建立起一套極低延遲、無衝突的雙臂 MoveIt 2 控制環境。

---

## 🌟 核心特性

1. **零侵入式修改 (Zero Intrusion)**：
   完全不需要修改官方 `ros2_kortex` 的原始碼。所有的修正（如：解決夾爪名稱前綴衝突、動態 URDF 注入、關節限制覆寫）均在 Launch 執行階段於記憶體中動態完成，確保系統乾淨且易於後續升級。
2. **極速啟動與主動輪詢 (Active Polling)**：
   捨棄傳統腳本中死板的 `time.sleep()`，改用主動輪詢檢查 Controller Manager 與 Action Server 狀態。系統啟動時間從原本的 >45 秒大幅縮短至實際硬體所需的 15 秒左右。
3. **硬體安全防護機制 (Safety Limits)**：
   內建客製化的 `my_joint_limits.yaml` 與 `my_dual_gen3.urdf`。針對奇數軸加入了安全擺幅限制，防止手臂在狹小的工作環境中打結或發生運動學奇異點（Singularity）翻轉。
4. **高階規劃器支援 (Advanced Planners)**：
   同時提供標準的 OMPL 啟動腳本以及 NVIDIA cuMotion (GPU 加速) 的專屬啟動路徑。

---

## 🛠️ 從零開始建置指南 (Setup from Scratch)

如果您是第一次接手這套系統，請依照以下步驟建立環境：

### 1. 系統與網路準備
- **作業系統**：Ubuntu 22.04 + ROS 2 Humble
- **網路設定**：請將與機械手臂連接的電腦網卡設定為固定 IP (例如：`192.168.1.100` / 子網路遮罩 `255.255.255.0`)。
- **手臂預設 IP**：
  - 左臂：`192.168.1.10`
  - 右臂：`192.168.1.11`

### 2. 下載官方驅動與建立工作空間
```bash
# 建立工作空間
mkdir -p ~/workspace/ros2_kortex_ws/src
cd ~/workspace/ros2_kortex_ws/src

# Clone 官方 Kinova ROS 2 repository
git clone https://github.com/Kinovarobotics/ros2_kortex.git -b humble

# (若有客製的雙臂基礎封裝包 dual_kortex_bringup 也請一併放入 src)
```

### 3. 編譯工作空間
```bash
cd ~/workspace/ros2_kortex_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

### 4. 匯入本整合腳本包
請確保本資料夾 (`start_dual_gen3_real_alfred`) 被放置於工作空間的 `scripts` 目錄下：
路徑應為：`~/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred/`

---

## 🚀 一鍵啟動步驟 (Launch Sequence)

本資料夾提供了一鍵式啟動 Python 腳本。它會自動在後台為您依序拉起：底層驅動、Controller Spawners、Joint States Merger、Trajectory Splitter 以及最終的 MoveIt 2 RViz 介面。

### 選項 A：標準 OMPL 規劃啟動 (穩定版)
最穩定且預設的啟動方式，使用傳統 CPU 運算的 OMPL 進行運動規劃。
```bash
cd ~/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred
python3 start_dual_gen3_real_alfred.py
```

### 選項 B：NVIDIA cuMotion 規劃啟動 (GPU 加速版)
專為需要毫秒級規劃反應的閉環控制場景設計。
```bash
cd ~/workspace/ros2_kortex_ws/scripts/start_dual_gen3_real_alfred
python3 start_dual_gen3_cumotion.py
```
> [!WARNING]
> 執行此腳本前，請確認您已經安裝了 `isaac_ros_cumotion`，並且已生成了對應的 `.xrdf` 描述檔（詳細教學請參見目錄下的 `README_XRDF_Generation.md`）。

---

## 📁 檔案結構與核心原理字典

了解以下檔案的作用，將有助於您日後進行系統的微調與除錯：

### 主控腳本 (Master Scripts)
- **`start_dual_gen3_real_alfred.py`**：標準版一鍵啟動腳本。具備自動清理殘留進程（pkill）、主動輪詢（檢查 ros2 service/topic 狀態）的特性。
- **`start_dual_gen3_cumotion.py`**：cuMotion 版一鍵啟動腳本，與標準版流程相似，但在最後階段會呼叫 cuMotion 的專屬 Launch 檔。

### Launch 檔與動態注入 (Launch & Injection)
- **`my_right_gen3.launch.py`**：右臂硬體驅動啟動檔。**核心亮點**：它會利用 `subprocess` 擷取官方 `xacro` 產出的 XML，並在記憶體中動態將 `robotiq_85_left_knuckle_joint` 替換為 `right_robotiq_85_left_knuckle_joint`，完美解決了 ROS 2 控制器在雙臂環境下遇到相同夾爪關節名稱而崩潰的 Bug。
- **`my_real_dual_moveit.launch.py`** / **`my_real_dual_cumotion.launch.py`**：MoveIt 2 的核心啟動檔。透過 `MoveItConfigsBuilder` 強制覆寫 `robot_description` 與 `joint_limits` 的路徑，使其指向本資料夾內的客製化安全參數。

### URDF 與參數設定 (Configs & URDF)
- **`my_dual_gen3.urdf`**：靜態化且組合完成的雙臂機器人 URDF。內部包含了右臂末端的 Bota FT 感測器以及 Realsense 深度相機的精確物理尺寸與碰撞外殼（Collision Mesh），是避障規劃的基礎藍圖。
- **`my_joint_limits.yaml`**：客製化關節限制。將手臂特定關節的速度、加速度與旋轉極限下修，保障手術器械夾取時的安全。
- **`right_ros2_controllers.yaml`** / **`right_gripper_controller.yaml`**：ROS 2 Control 控制器參數，指定了關節狀態發布的更新頻率以及 Trajectory Controller 的命令介面映射。

### 維護與除錯工具 (Utilities)
- **`kortex_watchdog.py`**：背景守護進程。會以 10Hz 監聽兩支手臂的系統狀態，當偵測到硬體發生錯誤（如 Minor Faults）時，自動呼叫 `/clear_faults` 服務嘗試恢復，避免系統因小干擾而死鎖。
- **`recover_controllers.sh`**：手動快速重啟 Trajectory Controllers 與 Gripper Controllers 的腳本（適用於開發測試期間的急救）。
- **`01_keyboard_teleop.py`**：提供終端機鍵盤遙控功能的除錯工具，可用於手動微調雙臂姿態。
