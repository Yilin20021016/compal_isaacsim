import random
import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.dynamic_control import _dynamic_control
from omni.physx.scripts import utils
from pxr import Gf, UsdGeom

dc = _dynamic_control.acquire_dynamic_control_interface()

def spawn_minimal_instruments(num_to_spawn=15):
    # 1. 建立 25 種器械的 Windows USD 路徑陣列
    tooling_usd_paths = []
    base_dir = "C:/isaacsim/moveit2/Tooling/Tooling/Tooling_URDF"
    
    for idx in range(1, 26):
        folder_name = f"Tooling_{idx}"
        usd_path = f"{base_dir}/{folder_name}/urdf/{folder_name}/{folder_name}.usd"
        tooling_usd_paths.append(usd_path)

    # 2. 核心坐標與高度設定
    base_x, base_y = 0.0, 0.0
    initial_z = 0.62  # 起始高度固定從 0.62 公尺開始
    z_gap = 0.15  # 每次向上遞增的間距 (公尺)，防止 Mesh 完全重疊

    print(f"[INFO] Spawning {num_to_spawn} random instruments sequentially starting at Z={initial_z}...")
    
    # 3. 迴圈隨機挑選並載入 Stage
    for i in range(num_to_spawn):
        # 隨機挑選 1~25 其中一個器械
        selected_usd = random.choice(tooling_usd_paths)
        prim_path = f"/World/Tooling/Tooling_{i}"

        
        # 計算當前物件的 Z 軸位置
        new_location = Gf.Vec3f(base_x, base_y, initial_z + (i * z_gap))  # Replace with your desired location
        new_rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)  # Replace with your desired rotation
        
        # 將該原始 USD 作為 Reference 引入當前場景
        add_reference_to_stage(usd_path=selected_usd, prim_path=prim_path)
        
        # 精準設定其在 Stage 上的世界位置 (Position)
        prim = dc.get_rigid_body(prim_path+"/base_link")
        if prim.IsValid():
            new_transform = UsdGeom.TransformAPI(prim)
            new_transform.SetTransform(UsdGeom.XformOp.Transform(Gf.Matrix4d(new_rotation.GetMatrix(), new_location)))
            dc.set_rigid_body_pose(prim, new_transform.GetLocalTransformation())
        else:
            print(f"[WARNING] Failed to reference or find prim at: {prim_path}")

    print("[INFO] Done. All requested instruments have been successfully loaded and stacked.")

# 執行：例如隨機引入 10 個器械
spawn_minimal_instruments(num_to_spawn=10)