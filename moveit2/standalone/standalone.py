import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. 啟動 SimulationApp
from isaacsim import SimulationApp

app_config = {
    "headless": False,
    "renderer": "RayTracedLighting",
    "display_options": 0,
    "vsync": False,
}
simulation_app = SimulationApp(app_config)

# 2. 導入 Isaac Sim 與 Omniverse 核心 API
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import open_stage

import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
extension_to_enable = "isaacsim.ros2.bridge" 
if not ext_manager.is_extension_enabled(extension_to_enable):
    print(f"Enabling extension: {extension_to_enable}")
    ext_manager.set_extension_enabled_immediate(extension_to_enable, True)
    simulation_app.update()

try:
    import isaacsim.ros2.bridge as ros2_bridge
    print("Extension successfully imported into Standalone Script!")
except ImportError as e:
    print(f"Failed to import extension modules: {e}")

# 3. 載入基礎環境 USD
usd_env_path = "./demo.usd"
if not open_stage(usd_env_path):
    print(f"Failed to open stage: {usd_env_path}")
    simulation_app.close()
    exit()

print(f"Successfully loaded base environment: {usd_env_path}")


# ==================== 座標系轉換邏輯 ====================
# 手臂基座在世界座標系下的姿態
ARM_BASE_TRANS = np.array([0.0, -0.675, 0.75])
ARM_BASE_ROT = R.from_euler('xyz', [0.0, 0.0, 0.0], degrees=True)

# 建立 T_World_to_ArmBase 4x4 矩陣
T_W_A = np.identity(4)
T_W_A[:3, :3] = ARM_BASE_ROT.as_matrix()
T_W_A[:3, 3] = ARM_BASE_TRANS

SPAWN_Z_OFFSET = 0.07


def spawn_instruments_from_json(json_file_path):
    """讀取 JSON 檔案，進行座標轉換後生成物件到世界座標系"""
    tooling_prims = []

    try:
        with open(json_file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"[Error] Failed to read JSON file: {e}")
        return tooling_prims

    instruments = data.get("instruments", [])
    print(f"Found {len(instruments)} instruments in JSON. Transforming coordinates & spawning...")

    for idx, item in enumerate(instruments):
        class_id = item.get("class_id")
        class_name = item.get("class_name", "Unknown")
        
        # 1. 取得物件相對於手臂基座的 4x4 變換矩陣 T_ArmBase_to_Object
        T_A_O = np.array(item["transformation_matrix_4x4"])

        # 2. 矩陣相乘轉換至世界座標系：T_World_to_Object = T_World_to_ArmBase * T_ArmBase_to_Object
        T_W_O = T_W_A @ T_A_O

        # 3. 拆解出世界座標系下的 Translation 與 Quaternion
        world_pos = T_W_O[:3, 3].tolist()
        world_pos[2] += SPAWN_Z_OFFSET
        if idx == 0:
            world_pos[0] += 0.01
        if idx < 2:
            world_pos[2] += 0.004
        world_rot_matrix = T_W_O[:3, :3]
        
        # 轉回 Quaternion (scipy 輸出為 [x, y, z, w])
        r_world = R.from_matrix(world_rot_matrix)
        quat_xyzw = r_world.as_quat()
        # Isaac Sim create_prim 支援 (w, x, y, z) 格式
        world_quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]

        # 4. 生成物件
        usd_path = f"./Tooling/2026_0401_5/tool_{class_id}.usd"
        prim_path = f"/World/tooling/Tooling_{idx}"

        prim_utils.create_prim(
            prim_path=prim_path,
            usd_path=usd_path,
            position=world_pos,
            orientation=world_quat_wxyz
        )

        print(f"[{idx+1}/{len(instruments)}] '{class_name}' (ID: {class_id}) spawned at World Pose: {world_pos}")
        tooling_prims.append(prim_path)

    return tooling_prims


# 4. 初始化物理模擬上下文
sim_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)

# 5. 從 JSON 載入物件
json_path = "./scene_20260713_142158.json"
instruments = spawn_instruments_from_json(json_path)

# 6. 重置模擬環境套用設定
sim_context.reset()

print("Initialization complete! Entering main simulation loop...")

# 7. 主循環
while simulation_app.is_running():
    sim_context.step(render=True)

# 8. 關閉應用程式
simulation_app.close()