"""
================================================================================
腳本名稱: batch_mesh_to_usd.py
功能說明:
    本腳本專為手術器械 3D 數位資產標準化打造，支援 Blender 無頭背景模式 (Headless Mode)
    批次自動化處理。能夠將 3D 掃描取得之原始 STL 檔案進行幾何修復、水密化、PCA 姿態統一、
    PBR 材質賦予，並獨立導出標準 USD 檔案供 Isaac Sim / Isaac Lab 進行剛體物理模擬。

核心處理流程:
    1. 尺度自適應校正: 自動判斷長度單位 (mm 自動縮放 0.001 轉為 m)。
    2. 底層幾何清理: 透過 bmesh 刪除孤立點/邊、溶解零長度退化邊面、縫合微小破洞。
    3. 自適應水密化 (Watertight): 根據器械尺寸動態計算 Voxel Remesh 體素解析度，
       防止記憶體溢出 (OOM Killed)，並徹底消除 PhysX 剛體碰撞彈飛問題。
    4. 平滑化與著色: 套用 Smooth 修改器消除體素階梯稜角，並啟用平滑著色 (Smooth Shading)。
    5. PCA 姿態標準化:
       - 第一長軸對齊 X 軸
       - 第二長軸 (寬度) 對齊 Y 軸
       - 翹面 (弧度朝向) 一律對齊 +Z 軸
    6. 物理標準化: 原點精確重設至體積質心 (Origin to Center of Mass)，座標歸零 (0, 0, 0)。
    7. PBR 金屬材質: 套用手術不銹鋼 (#D9E2E8FF, Metallic 1.0, Roughness 0.3, IOR 1.5)。
    8. 獨立解耦匯出: 每隻器械單獨輸出為獨立 USD 檔案，無位置偏移殘留。
    9. 終端檢核佈局: 自動將所有處理後的器械排版展開，另存為 .blend 專案檔供人工快速檢視。

執行環境需求:
    - Blender 3.6 / 4.x / 5.x (內建 Python 環境含 numpy, bmesh, mathutils)
    - 支援背景執行無頭模式

命令列使用範例:
    blender -b -P batch_mesh_to_usd.py -- \\
      --stl-dir /path/to/stl_folder \\
      --usd-dir /path/to/usd_output_folder \\
      --inspect-blend /path/to/usd_output_folder/check_all.blend \\
      --smooth-iterations 3 \\
      --grid-spacing 0.25 \\
      --cols 6

參數說明:
    --stl-dir            [必要] 存放原始 STL 檔案的目錄路徑。
    --usd-dir            [必要] 輸出個別獨立 USD 檔案的目標目錄路徑。
    --inspect-blend      [選填] 儲存供人工終端檢核排版之 .blend 檔案路徑 (預設: inspection_layout.blend)。
    --smooth-iterations  [選填] 表面平滑迭代次數，建議 2~5 (預設: 3)。
    --grid-spacing       [選填] 終端檢核排版時各器械間距，單位公尺 (預設: 0.25)。
    --cols               [選填] 檢核排版每列擺放之物件數量 (預設: 6)。

終端檢視 SOP:
    腳本執行完畢後，開啟產生的檢視檔:
        blender /path/to/usd_output_folder/check_all.blend
    在 3D 視圖中切換為 Material Preview 預覽，旋轉確認外觀無塌陷變形即可完成檢核。
================================================================================
"""

import argparse
import gc
import os
import subprocess
import sys

import bmesh

import bpy

from mathutils import Matrix

import numpy as np


def parse_args():
    if "--" in sys.argv:
        args_idx = sys.argv.index("--") + 1
        custom_args = sys.argv[args_idx:]
    else:
        custom_args = []

    parser = argparse.ArgumentParser(description="批次修復手術器械 STL 並個別匯出為獨立 USD 檔案 (含平滑化處理)")
    parser.add_argument("--stl-dir", type=str, required=True, help="存放原始 STL 檔案的目錄")
    parser.add_argument("--usd-dir", type=str, required=True, help="輸出獨立 USD 檔案的目錄")
    parser.add_argument("--inspect-blend", type=str, default="inspection_layout.blend", help="輸出供檢核的 .blend 檔案路徑")
    parser.add_argument("--smooth-iterations", type=int, default=3, help="平滑迭代次數 (建議 2~5)")
    parser.add_argument("--grid-spacing", type=float, default=0.25, help="檢視排版間距 (m)")
    parser.add_argument("--cols", type=int, default=6, help="檢視矩陣每列數量")

    return parser.parse_args(custom_args)


def hex_to_linear_rgba(hex_str):
    hex_str = hex_str.lstrip('#')
    if len(hex_str) == 6:
        r, g, b = [int(hex_str[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
        a = 1.0
    elif len(hex_str) == 8:
        r, g, b, a = [int(hex_str[i:i + 2], 16) / 255.0 for i in (0, 2, 4, 6)]
    else:
        return (0.7, 0.7, 0.7, 1.0)

    def srgb_to_linear(c):
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    return (srgb_to_linear(r), srgb_to_linear(g), srgb_to_linear(b), a)


def setup_surgical_material():
    mat_name = "Surgical_Steel"
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
        mat.use_nodes = True

    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    bsdf = next((n for n in nodes if n.type == 'BSDF_PRINCIPLED'), None)
    if not bsdf:
        bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
        output_node = next((n for n in nodes if n.type == 'OUTPUT_MATERIAL'), None)
        if not output_node:
            output_node = nodes.new(type='ShaderNodeOutputMaterial')
        links.new(bsdf.outputs['BSDF'], output_node.inputs['Surface'])

    base_color_rgba = hex_to_linear_rgba("#D9E2E8FF")

    target_inputs = {
        "Base Color": base_color_rgba,
        "Metallic": 1.0,
        "Roughness": 0.3,
        "IOR": 1.5,
        "Alpha": 1.0
    }

    for socket_name, val in target_inputs.items():
        if socket_name in bsdf.inputs:
            bsdf.inputs[socket_name].default_value = val
        else:
            for inp in bsdf.inputs:
                if inp.identifier.lower() == socket_name.lower():
                    inp.default_value = val
                    break

    return mat


def align_orientation_pca(obj):
    mesh = obj.data
    verts = np.array([v.co for v in mesh.vertices])
    if len(verts) < 3:
        return

    centroid = np.mean(verts, axis=0)
    centered_verts = verts - centroid

    cov = np.cov(centered_verts, rowvar=False)
    eig_vals, eig_vecs = np.linalg.eigh(cov)

    sort_indices = np.argsort(eig_vals)[::-1]
    v_x = eig_vecs[:, sort_indices[0]]
    v_y = eig_vecs[:, sort_indices[1]]

    v_z = np.cross(v_x, v_y)
    v_y = np.cross(v_z, v_x)

    v_x /= np.linalg.norm(v_x)
    v_y /= np.linalg.norm(v_y)
    v_z /= np.linalg.norm(v_z)

    R_align = np.vstack([v_x, v_y, v_z])
    mat_rot = Matrix([
        [R_align[0, 0], R_align[0, 1], R_align[0, 2], 0.0],
        [R_align[1, 0], R_align[1, 1], R_align[1, 2], 0.0],
        [R_align[2, 0], R_align[2, 1], R_align[2, 2], 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    mesh.transform(mat_rot)
    mesh.update()

    verts_aligned = np.array([v.co for v in mesh.vertices])
    x_coords = verts_aligned[:, 0]
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    x_range = x_max - x_min

    tips_mask = (x_coords > x_max - 0.15 * x_range) | (x_coords < x_min + 0.15 * x_range)
    mid_mask = (x_coords >= x_min + 0.3 * x_range) & (x_coords <= x_max - 0.3 * x_range)

    z_tips = np.mean(verts_aligned[tips_mask, 2]) if np.any(tips_mask) else 0.0
    z_mid = np.mean(verts_aligned[mid_mask, 2]) if np.any(mid_mask) else 0.0

    if z_tips < z_mid:
        flip_x = Matrix.Rotation(np.pi, 4, 'X')
        mesh.transform(flip_x)
        mesh.update()

    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)


def process_and_repair_mesh(obj, smooth_iterations=3):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    # 1. 尺度自適應校正 (mm 轉 m)
    max_dim = max(obj.dimensions)
    if max_dim > 2.0:
        obj.scale = (0.001, 0.001, 0.001)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # 2. 原點預校正至體積質心
    bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_VOLUME', center='MEDIAN')

    # 3. 使用 bmesh 進行底層幾何清理
    bm = bmesh.new()
    bm.from_mesh(obj.data)

    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.0001)
    bmesh.ops.dissolve_degenerate(bm, dist=0.0001, edges=bm.edges)
    loose_verts = [v for v in bm.verts if not v.link_edges]
    bmesh.ops.delete(bm, geom=loose_verts, context='VERTS')
    loose_edges = [e for e in bm.edges if not e.link_faces]
    bmesh.ops.delete(bm, geom=loose_edges, context='EDGES')

    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()

    # 4. 補破洞防護
    try:
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.fill_holes(sides=4)
        bpy.ops.object.mode_set(mode='OBJECT')
    except Exception:
        bpy.ops.object.mode_set(mode='OBJECT')

    # 5. 安全動態 Voxel Remesh (水密化)
    safe_voxel_size = max(max(obj.dimensions) / 250.0, 0.0005)
    remesh_mod = obj.modifiers.new(name="Auto_Voxel_Remesh", type='REMESH')
    remesh_mod.mode = 'VOXEL'
    remesh_mod.voxel_size = safe_voxel_size
    bpy.ops.object.modifier_apply(modifier=remesh_mod.name)

    # 6. 【新增】平滑化處理 (消除階梯銳利感，平整表面)
    if smooth_iterations > 0:
        smooth_mod = obj.modifiers.new(name="Auto_Smooth", type='SMOOTH')
        smooth_mod.factor = 0.5                      # 平滑強度
        smooth_mod.iterations = smooth_iterations    # 迭代次數
        bpy.ops.object.modifier_apply(modifier=smooth_mod.name)

    # 7. 面三角化 (Triangulate)
    tri_mod = obj.modifiers.new(name="Auto_Triangulate", type='TRIANGULATE')
    bpy.ops.object.modifier_apply(modifier=tri_mod.name)

    # 8. 【新增】啟用平滑著色 (Smooth Shading)
    for p in obj.data.polygons:
        p.use_smooth = True
    obj.data.update()

    # 9. 主軸對齊 (長軸 X, 次軸 Y, 翹曲 +Z)
    align_orientation_pca(obj)

    # 10. 體積質心校正並歸零坐標
    bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_VOLUME', center='MEDIAN')
    obj.location = (0.0, 0.0, 0.0)

    # 11. PBR 材質設定
    mat = setup_surgical_material()
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)
    obj.active_material = mat


def export_single_usd(obj, export_filepath):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    bpy.ops.wm.usd_export(
        filepath=export_filepath,
        selected_objects_only=True,
        export_materials=True,
        export_uvmaps=True,
        evaluation_mode='RENDER'
    )


def cleanup_orphaned_memory():
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    gc.collect()


def main():
    args = parse_args()
    stl_dir = os.path.abspath(args.stl_dir)
    usd_dir = os.path.abspath(args.usd_dir)
    os.makedirs(usd_dir, exist_ok=True)

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    stl_files = sorted([f for f in os.listdir(stl_dir) if f.lower().endswith('.stl')])
    if not stl_files:
        print(f"[錯誤] 未找到 STL 檔案: {stl_dir}")
        sys.exit(1)

    print(f">> 偵測到 {len(stl_files)} 個 STL 檔案，執行平滑化與批次匯出...")
    import time
    for idx, file_name in enumerate(stl_files):
        start_time = time.time()
        base_name = os.path.splitext(file_name)[0]
        file_path = os.path.join(stl_dir, file_name)

        bpy.ops.wm.stl_import(filepath=file_path)
        obj = bpy.context.selected_objects[0]
        obj.name = base_name

        process_and_repair_mesh(obj, smooth_iterations=args.smooth_iterations)

        usd_export_path = os.path.join(usd_dir, f"{base_name}.usd")
        export_single_usd(obj, usd_export_path)
        print(f"[{idx + 1}/{len(stl_files)}] 已輸出平滑資產: {base_name}.usd")

        row = idx // args.cols
        col = idx % args.cols
        obj.location.x = col * args.grid_spacing
        obj.location.y = row * args.grid_spacing
        obj.location.z = 0.0

        cleanup_orphaned_memory()
        end_time = time.time()
        print(f">> {file_name}處理時間: {end_time - start_time} 秒\n")

    bpy.context.preferences.filepaths.save_version = 0
    inspect_blend_path = os.path.abspath(args.inspect_blend)
    bpy.ops.wm.save_mainfile(filepath=inspect_blend_path)

    print(f"\n>> 批次處理完畢！檢視檔儲存於: {inspect_blend_path}")
    subprocess.run(["blender", inspect_blend_path])


if __name__ == "__main__":
    main()
