# 雙臂 Gen3 XRDF 生成與 URDF 轉移指南

本資料夾包含了用於雙臂 Gen3 (掛載 Robotiq 2F-85 與右臂 Bota 感測器) 整合 NVIDIA cuMotion 的核心靜態模型藍圖：`my_dual_gen3.urdf`。

如果您需要將 `my_dual_gen3.urdf` 移至**另一台電腦**上的 Isaac Sim 來生成 XRDF 防撞球體，請**務必**仔細閱讀本指南，否則 Isaac Sim 將無法讀取 3D 網格 (Mesh) 導致模型載入失敗。

---

## 📦 必須打包的關聯 3D 模型 (Meshes) 清單

`my_dual_gen3.urdf` 內所有關聯的 STL / DAE 網格模型皆位於本機 `install` 資料夾中。跨電腦轉移時，請將以下三個 `meshes` 資料夾隨同 URDF 一併拷貝帶走：

### 1. Kinova Gen3 手臂本體 (共 16 個關節 x 左右手)
- **來源套件**：`kortex_description`
- **本機路徑**：`/home/alfred/workspace/ros2_kortex_ws/install/kortex_description/share/kortex_description/arms/gen3/7dof/meshes/`
- **包含的核心 STL**：
  - `base_link.STL` (基座)
  - `shoulder_link.STL` (肩膀)
  - `half_arm_1_link.STL` / `half_arm_2_link.STL` (大手臂)
  - `forearm_link.STL` (小手臂)
  - `spherical_wrist_1_link.STL` / `spherical_wrist_2_link.STL` (球型手腕)
  - `bracelet_no_vision_link.STL` (法蘭手環)

### 2. Robotiq 2F-85 夾爪 (外觀與碰撞體 x 左右手)
- **來源套件**：`robotiq_description`
- **本機路徑**：`/home/alfred/workspace/ros2_kortex_ws/install/robotiq_description/share/robotiq_description/meshes/`
- **包含的核心 STL / DAE** (分為 `visual/` 與 `collision/` 子資料夾)：
  - `robotiq_base` (夾爪基座主體)
  - `left_knuckle` / `right_knuckle` (指關節)
  - `left_finger` / `right_finger` (主手指)
  - `left_inner_knuckle` / `right_inner_knuckle` (內連桿)
  - `left_finger_tip` / `right_finger_tip` (指尖/指甲)

### 3. Bota Systems 力矩感測器 (僅右手)
- **來源套件**：`bota_driver`
- **本機路徑**：`/home/alfred/workspace/ros2_kortex_ws/install/bota_driver/share/bota_driver/meshes/BFT_KG3_IND2_SW/`
- **包含的核心 DAE**：
  - `mounting.dae` (擁有真實鎖孔細節的金屬感測器外觀)

---

## 🚨 跨電腦轉移 URDF 修正步驟

目前 `my_dual_gen3.urdf` 中的所有 `<mesh filename="...">` 都是被自動化腳本寫死的 **Alfred 本機絕對路徑** (Absolute Path)。若直接在其他電腦開啟，會出現找不到檔案的錯誤。

請在另一台電腦上執行以下步驟：

1. **放置模型**：將上述打包來的三個 `meshes` 資料夾，放置在該電腦的任意已知路徑中（例如 `C:\ROS2_Meshes\` 或 `/home/user/meshes/`）。
2. **開啟 URDF**：使用 VS Code 或任何純文字編輯器開啟 `my_dual_gen3.urdf`。
3. **批次取代路徑**：
   - 使用 **全部取代 (Find & Replace)** 功能。
   - 將原本的本機前綴：`file:///home/alfred/workspace/ros2_kortex_ws/install/`
   - 替換成：`file:///<你存放 meshes 的新路徑>/`
4. **載入與生成**：存檔後，即可將 URDF 載入 Isaac Sim 中的 **Lula Robot Description Editor**，並透過 Auto-Generate 功能生成初步防撞球體。

---

## ⚠️ Isaac Sim XRDF 微調注意事項

自動生成的防撞球體通常過於粗糙，對於夾爪的操作極度不友善。請負責 XRDF 的人員**務必**手動進行以下微調：

1. **清空夾爪夾持區**：將 Robotiq 2F-85 夾爪「U 型開口正中間」的巨大防撞球縮小或刪除，否則 cuMotion 將無法規劃靠近物體的夾取軌跡。
2. **Bota 感測器包覆**：由於我們為了避免 MoveIt 報錯，將 `right_bota_link` 設為純視覺模型 (`<visual>`) 而無實體碰撞標籤 (`<collision>`)，演算法可能不會為其生成球體。**請手動在右手腕與夾爪之間的黑色感測器處，放置一顆半徑約 `0.04m` 的防撞球。**
3. **手指連動確認**：確保手指尖端的球體正確綁定在 `left_finger_tip_link` 等活動節點上，而非全部綁死在夾爪基座。

完成後將檔案匯出為 **`my_dual_gen3.xrdf`** 並放回此資料夾，即可解鎖 GPU 加速運動規劃！
