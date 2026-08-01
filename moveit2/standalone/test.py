import os
import numpy as np
import open3d as o3d
from pxr import Usd, UsdGeom, Gf, Sdf

def extract_mesh_points(usd_path):
    stage = Usd.Stage.Open(usd_path, Usd.Stage.LoadAll)
    points_list = []
    
    for layer in stage.GetUsedLayers():
        temp_stage = Usd.Stage.Open(layer)
        for prim in temp_stage.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                if pts and len(pts) > 0:
                    points_list.append(np.array(pts))
                    
    if not points_list:
        raise ValueError(f"No valid geometry points found in {usd_path}.")
    return np.vstack(points_list)

def prepare_dataset(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd_down, pcd_fpfh

def align_and_save_assembly(source_usd_path, target_usd_path, output_usd_path):
    print("正在提取頂點資料...")
    src_pts = extract_mesh_points(source_usd_path)
    tgt_pts = extract_mesh_points(target_usd_path)

    # 1. 自動檢測尺寸比例 (Scale Detection)
    src_extent = max(np.max(src_pts, axis=0) - np.min(src_pts, axis=0))
    tgt_extent = max(np.max(tgt_pts, axis=0) - np.min(tgt_pts, axis=0))
    
    scale_factor = tgt_extent / src_extent
    print(f"[單位偵測] Target 邊界: {tgt_extent:.4f} | Source 邊界: {src_extent:.4f}")
    print(f"[自動縮放] 預計縮放倍率: {scale_factor:.6f}")

    # 預先進行單位等比例縮放進行配準
    src_pts_scaled = src_pts * scale_factor

    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(src_pts_scaled)

    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(tgt_pts)

    # 2. 特徵與配準
    bounding_box = tgt_pcd.get_axis_aligned_bounding_box()
    max_extent = max(bounding_box.get_extent())
    voxel_size = max_extent / 50.0

    src_down, src_fpfh = prepare_dataset(src_pcd, voxel_size)
    tgt_down, tgt_fpfh = prepare_dataset(tgt_pcd, voxel_size)

    print("正在執行 RANSAC 特徵匹配與空間對齊...")
    distance_threshold = voxel_size * 1.5
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(10000000, 0.999)
    )

    print("正在執行精細 ICP 對齊...")
    distance_threshold = voxel_size * 0.4
    icp_result = o3d.pipelines.registration.registration_icp(
        src_pcd, tgt_pcd, distance_threshold, ransac_result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True)
    )

    # 3. 組合包含 Scale + Rotation + Translation 的 4x4 矩陣
    T_icp = icp_result.transformation
    
    # 建立純 S (Scale) 矩陣
    S = np.identity(4)
    S[0,0] = scale_factor
    S[1,1] = scale_factor
    S[2,2] = scale_factor

    # 總變換矩陣 M = T_icp * S
    total_transform = T_icp @ S
    print("\n==== 計算出的總姿態變換矩陣 (含1000倍縮放) ====")
    print(total_transform)

    # 4. 寫入 USD 的 DefaultPrim / RootPrim XformOp
    stage = Usd.Stage.Open(source_usd_path, Usd.Stage.LoadAll)
    root_prim = stage.GetDefaultPrim()
    
    if not root_prim or not root_prim.IsValid():
        # 若無 Default Prim，抓第一個非 PseudoRoot 的 Prim (如 /Tooling_2)
        root_prim = [p for p in stage.GetPseudoRoot().GetChildren()][0]

    print(f"\n正在將 Transform 寫入 Root Prim: {root_prim.GetPath()} ...")

    xformable = UsdGeom.Xformable(root_prim)
    xformable.ClearXformOpOrder()
    
    # 將 NumPy 矩陣轉為 USD Gf.Matrix4d (列主序 Row-Major)
    gf_mat = Gf.Matrix4d(*total_transform.T.flat)
    xformable.AddTransformOp().Set(gf_mat)

    # 另存為新檔
    stage.Export(output_usd_path)
    print(f"成功將變換矩陣寫入並導出至: {output_usd_path}")

# 執行路徑
script_dir = os.path.dirname(os.path.abspath(__file__))
trg_tool = 'Tooling_1'
source_file = f"./moveit2/Tooling/Tooling/Tooling_URDF/{trg_tool}/urdf/{trg_tool}/{trg_tool}.usd"
target_file = "./moveit2/Tooling/2026_0401_5/tool_2.usd"
output_file = os.path.join(script_dir, f"{trg_tool}_aligned.usd")

align_and_save_assembly(source_file, target_file, output_file)
