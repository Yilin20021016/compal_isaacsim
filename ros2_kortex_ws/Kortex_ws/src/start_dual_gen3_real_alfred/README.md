# Start Dual Gen3 Real (Alfred)

## 📖 目錄簡介 (Overview)
本目錄包含啟動「Kinova Gen3 雙臂實體手臂」系統的所有底層 Launch 腳本、設定檔 (URDF, YAML) 與進階維護工具。這套啟動環境專為解決原生 ROS 2 Kortex 驅動器在雙臂環境下的衝突、以及 MoveIt 2 在 14 軸系統下的種種異常行為而深度客製化。

此環境**沒有修改任何第三方的 Github clone 套件庫**，而是透過「外部覆寫 (Override)」的方式，乾淨且獨立地掛載所有修改，確保系統升級時不受影響。

---

## 🚀 核心啟動腳本 (Orchestrators)

### 1. `start_dual_gen3_real_alfred.py` (預設主動輪詢極速版)
**這是系統的總電源開關！**
取代了傳統 ROS 2 Launch 容易因為單一節點啟動過慢而全線崩潰的缺點，此 Python 腳本會依序且安全地拉起整個雙臂世界：
*   **階段 1 (Driver)**: 啟動左右手臂的硬體驅動器 (Kortex Driver)。並使用「主動輪詢 (Active Polling)」等待 Controller Manager 上線。
*   **階段 2 (Spawners)**: 發射所有的 Joint Trajectory Controller 與 Robotiq Gripper Controller。
*   **階段 3 (Merger & Splitter)**: 啟動解決 `joint_states` 衝突的整併節點，以及拆分 Action Server 目標的 Splitter 節點。
*   **階段 4 (MoveIt2 & RViz)**: 啟動高階路徑規劃器。
> 💡 **自動大掃除**：腳本啟動前會自動 `pkill` 所有可能遺留的殭屍進程，確保每次開機都是乾淨狀態。

### 2. `start_dual_gen3_cumotion.py`
專為整合 **NVIDIA cuMotion** GPU 加速軌跡規劃器所客製的啟動版本。運作邏輯與上述雷同，但在最後階段會拉起帶有 cuMotion 支援的 MoveIt 2 配置。

---

## 🛠️ ROS 2 Launch 與設定檔 (Launch & Configs)

*   **`my_real_dual_moveit.launch.py`**: 
    客製化的 MoveIt 2 啟動檔。利用 `MoveItConfigsBuilder` 動態讀取原廠包，但**強行覆寫** `robot_description` 為我們的自訂 URDF，並覆寫 `joint_limits` 為我們閹割過的限制設定。
*   **`my_dual_gen3.urdf`**: 
    終極合體版 URDF。包含了：兩隻 Gen3 手臂、兩個 Robotiq 2F-85 夾爪、以及精準掛載的 **Bota Force/Torque 感測器** 與 **Ensenso 相機外部參數校正 (Hand-Eye Calibration)**。
    > 🐛 **歷史 Bug 避坑指南**：官方 `ros2_kortex` 在雙臂設定 (使用 prefix) 時，底層 C++ Driver 存在字串匹配缺陷，會導致找不到帶 prefix 的 `gripper_joint_name` 而使夾爪罷工。我們在這份靜態 URDF 中直接將 `<param name="gripper_joint_name">right_robotiq_85_left_knuckle_joint</param>` 寫死，**成功繞過了官方 Bug，且無須修改任何原廠 C++ 或 xacro 源代碼**。
*   **`my_joint_limits.yaml`**: 
    安全核心！大幅調降了所有關節的最大速度與加速度 (Velocity/Accel Scaling)。並特別針對雙臂的**第 1、3、5、7 奇數軸限制了擺動角度**，從物理層面強制杜絕手臂產生大迴旋 (亂轉打結) 的危險動作。
*   **`my_left_gen3.launch.py` & `my_right_gen3.launch.py`**:
    左右手的專屬硬體通訊 Launch，內部配置了專屬的 Prefix (例如 `left_`) 以區隔 TF 樹與 ROS 命名空間。
*   **`*_ros2_controllers.yaml` & `*_gripper_controller.yaml`**:
    針對左右手的 ROS 2 Control 設定，確保夾爪的 Action Server 不會互相打架。

---

## 🧰 開發與維護工具 (Utilities)

*   **`01_keyboard_teleop.py`**: 
    **強烈建議在空跑測試時使用！** 透過鍵盤 WASD 操作，可以動態在左手與右手之間切換，並直接命令 MoveIt 2 送出 Cartesian 位移與夾爪開合指令，用於驗證 TF 樹與 IK 解算器是否正常。
*   **`recover_controllers.sh`**: 
    緊急救援腳本。當您執行非同步排程發生 `FollowJointTrajectory` 硬體容忍度異常，導致手臂發呆卡死時，不需要重啟整個巨大環境。只需執行此腳本，即可瞬間把當掉的 Controller 重啟並奪回控制權。
*   **`kortex_watchdog.py`**: 
    底層守門員。不斷監聽硬體的連線狀態與 Error Code，若有異常會立即印出紅色警示，協助除錯。
*   **`pad_urdf_collision.py`**:
    自動化腳本。用來為某些缺乏碰撞幾何 (Collision Geometry) 的 URDF Link 補上預設的圓柱體或方塊，防止 MoveIt 規劃出物理干涉的路徑。

---

## ⚠️ 操作警語 (Safety Warning)
執行本目錄下的 `start_dual_gen3_*.py` 將會與**實體高壓機電系統**連線。
1. 啟動前請確認急停按鈕 (E-Stop) 在伸手可及之處。
2. 確認桌面已淨空，雙臂處於安全互不干涉的位姿。
3. 若發生不可預期的暴衝或互鎖，請立刻按下急停按鈕，並在執行腳本的終端機按下 `Ctrl+C` 讓系統自動執行 `pkill` 清理流程。
