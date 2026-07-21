import random
import numpy as np

# 1. 啟動 SimulationApp
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 2. 導入 Isaac Sim 與 Omniverse 核心 API
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import open_stage
from isaacsim.util.debug_draw import _debug_draw

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

# 3. 載入你的基礎環境 USD
# 請確保此相對路徑相對於你執行 python 腳本的位置是正確的
usd_env_path = "./demo.usd"
if not open_stage(usd_env_path):
    print(f"Failed to open stage: {usd_env_path}")
    simulation_app.close()
    exit()

print(f"Successfully loaded base environment: {usd_env_path}")

# ==================== 配置參數設定 ====================
# np.random.seed(52)
CENTER = np.array([0.0, 0.0, 0.75])
X_RANGE = 0.4
Y_RANGE = 0.3

# 手術器械 USD 路徑
TOOLING_USD_PATHS = [
    f"./Tooling/Tooling/Tooling_URDF/Tooling_{i}/urdf/Tooling_{i}/Tooling_{i}.usd"
    for i in range(1, 28)
]

X_MIN, X_MAX = CENTER[0] - X_RANGE / 2, CENTER[0] + X_RANGE / 2
Y_MIN, Y_MAX = CENTER[1] - Y_RANGE / 2, CENTER[1] + Y_RANGE / 2
# =======================================================


def spawn_and_stack_instruments(num_objects=5):
    """在指定範圍內動態載入器械並堆疊"""
    tooling_prims = []
    z=CENTER[2]

    print("Simulating instrument generation and stacking...")

    for i in range(4, num_objects+4):
        # 如果路徑清單為空，則跳過
        if not TOOLING_USD_PATHS:
            print("[Warning] INSTRUMENT_USD_PATHS is empty. Skipping asset spawn.")
            break

        usd_path = random.choice(TOOLING_USD_PATHS)
        
        # 隨機生成 X, Y 座標
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        
        # Z 軸方向依序遞增生成 (每層提早 0.02m)，避免重疊引發 PhysX 爆炸
        z = CENTER[2] + (i * 0.02)
        position = (x, y, z)

        # 隨機旋轉 (繞 Z 軸隨機 Yaw 旋轉，最適合強化學習夾取的隨機多樣性)
        q = np.random.randn(4)
        orientation = (q / np.linalg.norm(q)).tolist()

        prim_path = f"/World/tooling/Tooling_{i}"

        # 使用 Reference 載入 USD 檔案
        prim_utils.create_prim(
            prim_path=prim_path,
            usd_path=usd_path,
            position=position,
            orientation=orientation
        )
        
        tooling_prims.append(prim_path)

    # draw_workspace_bbox(z_range=z)  # 在場景中繪製邊界框，方便觀察生成範圍

    return tooling_prims

def draw_workspace_bbox(z_range=0.5):
    """在 Isaac Sim 世界中繪製隨機生成的邊界框"""
    # 1. 獲取 debug 繪圖接口
    draw = _debug_draw.acquire_debug_draw_interface()
    
    # 2. 清除之前畫的線條（防止重複繪製疊加）
    draw.clear_lines()
    
    # 定義相對於中心點的 8 個局部座標
    world_vertices = np.array([
        [X_MIN, Y_MIN, CENTER[2]], [X_MAX, Y_MIN, CENTER[2]], [X_MAX, Y_MAX, CENTER[2]], [X_MIN, Y_MAX, CENTER[2]], # 底面 4 個點 (0,1,2,3)
        [X_MIN, Y_MIN, z_range], [X_MAX, Y_MIN, z_range], [X_MAX,  Y_MAX, z_range], [X_MIN, Y_MAX, z_range]  # 頂面 4 個點 (4,5,6,7)
    ])

    # 4. 定義構成無蓋/有蓋立方體框線的「起點」與「終點」對 (Line Pairs)
    # 這裡我們把它畫成一個完整的 3D 盒子，共 12 條邊
    start_points = []
    end_points = []
    
    # 底面 4 條邊
    for i in range(4):
        start_points.append(world_vertices[i])
        end_points.append(world_vertices[(i + 1) % 4])
        
    # 頂面 4 條邊
    for i in range(4, 8):
        start_points.append(world_vertices[i])
        end_points.append(world_vertices[4 + (i - 3) % 4])
        
    # 連接頂面與底面的 4 條垂直邊
    for i in range(4):
        start_points.append(world_vertices[i])
        end_points.append(world_vertices[i + 4])
        
    # 5. 設定線條顏色與粗細 (RGBA 格式)
    colors = [[255, 0.0, 0.0, 1.0]] * len(start_points) # [R, G, B, A]
    sizes = [3.0] * len(start_points) # 線條粗細
    
    # 6. 一口氣把所有框線畫到 Stage 上
    draw.draw_lines(start_points, end_points, colors, sizes)
    print("Workspace bounding box visualization added to scene.")

# 4. 初始化物理模擬上下文 (SimulationContext)
# 必須在 open_stage 之後執行，這會接管當前 Stage 的 PhysX 物理場
sim_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)

# 5. 執行動態物件生成
instruments = spawn_and_stack_instruments(num_objects=10)

# 6. 重置模擬環境以套用所有物理設定
sim_context.reset()

print("Allowing objects to settle under gravity...")
# 7. 預跑 60 幀，讓物件自然掉落、碰撞、並堆疊穩定在 (0, 0, 0.65) 的平台上
for _ in range(60):
    sim_context.step(render=True)

print("Stacking complete! Entering main simulation loop for RL...")

# 8. 進入主循環 (後續強化學習的 step 就在這裡跑)
while simulation_app.is_running():
    # 這裡可以放你的 RL Agent step 邏輯、獲取觀測值(Observation)或下達 Action
    sim_context.step(render=True)

# 9. 關閉應用程式
simulation_app.close()