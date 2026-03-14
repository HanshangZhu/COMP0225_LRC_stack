#!/usr/bin/env python3
"""Refine hand-drawn trajectory using scan-to-map ICP.
1. Load waypoints, interpolate to N scan poses
2. For each scan, transform wall points to map frame using interpolated pose
3. ICP refine against wall-only map
4. Output refined trajectory."""
import numpy as np
import open3d as o3d
from plyfile import PlyData
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import json, csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

MAP_PLY = "carto_3d_map.ply"
WAYPOINTS_JSON = "hand_drawn_trajectory.json"
BAG_PATH = "cartographer_tuning_bag_2_clean"
OUTPUT_CSV = "refined_trajectory.csv"
OUTPUT_PNG = "refined_trajectory.png"

# Filtering
MIN_Z_WALL = 0.15
MAX_Z_WALL = 1.5
MIN_XY = 0.3
VOXEL_MAP = 0.05
ICP_DIST = 1.0  # larger search radius since initial guess is rough

def load_wall_map(path):
    ply = PlyData.read(path)
    x, y, z = np.array(ply['vertex']['x']), np.array(ply['vertex']['y']), np.array(ply['vertex']['z'])
    pts = np.column_stack([x, y, z])
    mask = (z >= MIN_Z_WALL) & (z <= MAX_Z_WALL)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[mask])
    pcd = pcd.voxel_down_sample(VOXEL_MAP)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
    print(f"Map: {mask.sum()} wall pts -> {len(pcd.points)} after voxel")
    return pcd

def interpolate_waypoints(waypoints, n_poses):
    """Interpolate waypoints to n_poses evenly spaced points along the path."""
    wp = np.array(waypoints)
    # Compute cumulative arc length
    dists = np.sqrt(np.sum(np.diff(wp, axis=0)**2, axis=1))
    cumlen = np.concatenate([[0], np.cumsum(dists)])
    total = cumlen[-1]
    
    # Evenly space n_poses along the path
    target_dists = np.linspace(0, total, n_poses)
    poses_xy = np.column_stack([
        np.interp(target_dists, cumlen, wp[:, 0]),
        np.interp(target_dists, cumlen, wp[:, 1])
    ])
    
    # Compute heading from consecutive points
    dx = np.gradient(poses_xy[:, 0])
    dy = np.gradient(poses_xy[:, 1])
    headings = np.arctan2(dy, dx)
    
    return poses_xy, headings

def make_transform(x, y, yaw, z=0.0):
    """Create 4x4 transform from x, y, yaw."""
    T = np.eye(4)
    T[0, 0] = np.cos(yaw); T[0, 1] = -np.sin(yaw)
    T[1, 0] = np.sin(yaw); T[1, 1] = np.cos(yaw)
    T[0, 3] = x; T[1, 3] = y; T[2, 3] = z
    return T

def scan_wall_points(pts_list):
    """Extract wall points from a scan."""
    xs = np.array([p[0] for p in pts_list])
    ys = np.array([p[1] for p in pts_list])
    zs = np.array([p[2] for p in pts_list])
    mask = (zs >= MIN_Z_WALL) & (zs <= MAX_Z_WALL) & (xs**2 + ys**2 >= MIN_XY**2)
    if mask.sum() < 5:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack([xs[mask], ys[mask], zs[mask]]))
    return pcd

def main():
    # Load map
    map_pcd = load_wall_map(MAP_PLY)
    
    # Load waypoints
    with open(WAYPOINTS_JSON) as f:
        wp = json.load(f)['waypoints']
    print(f"Loaded {len(wp)} waypoints")
    
    # Count scans in bag
    reader = SequentialReader()
    reader.open(StorageOptions(uri=BAG_PATH, storage_id='sqlite3'), ConverterOptions('', ''))
    n_scans = 0
    scans = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if 'cloud' in topic.lower():
            msg = deserialize_message(data, PointCloud2)
            pts = list(pc2.read_points(msg, skip_nans=True))
            scans.append((ts, pts))
            n_scans += 1
    print(f"Loaded {n_scans} scans from bag")
    
    # Interpolate waypoints to scan count
    init_xy, init_yaw = interpolate_waypoints(wp, n_scans)
    print(f"Interpolated to {n_scans} poses")
    print(f"  Init extent: {init_xy[:,0].max()-init_xy[:,0].min():.2f}m x {init_xy[:,1].max()-init_xy[:,1].min():.2f}m")
    
    # ICP refine each scan
    refined_poses = []
    for i, (ts, pts) in enumerate(scans):
        scan_pcd = scan_wall_points(pts)
        x0, y0, yaw0 = init_xy[i, 0], init_xy[i, 1], init_yaw[i]
        
        if scan_pcd is None or len(scan_pcd.points) < 5:
            refined_poses.append((i, ts, x0, y0, 0.0, yaw0, 0.0, 0.0, 0))
            continue
        
        # Initial transform from hand-drawn pose
        T_init = make_transform(x0, y0, yaw0)
        
        # ICP refine
        result = o3d.pipelines.registration.registration_icp(
            scan_pcd, map_pcd,
            ICP_DIST,
            T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )
        
        T = result.transformation
        x_r, y_r, z_r = T[0, 3], T[1, 3], T[2, 3]
        yaw_r = np.arctan2(T[1, 0], T[0, 0])
        
        refined_poses.append((i, ts, x_r, y_r, z_r, yaw_r, 
                             result.fitness, result.inlier_rmse, len(scan_pcd.points)))
        
        if i % 100 == 0:
            dx = x_r - x0; dy = y_r - y0
            print(f"  Scan {i}: init=({x0:.2f},{y0:.2f}) -> ref=({x_r:.2f},{y_r:.2f}) "
                  f"delta=({dx:.2f},{dy:.2f}) fit={result.fitness:.3f} wall={len(scan_pcd.points)}")
    
    # Save CSV
    with open(OUTPUT_CSV, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['scan_idx', 'timestamp', 'x', 'y', 'z', 'yaw', 'fitness', 'rmse', 'n_wall'])
        for p in refined_poses:
            w.writerow(p)
    print(f"Saved {len(refined_poses)} poses to {OUTPUT_CSV}")
    
    # Plot
    init_x, init_y = init_xy[:, 0], init_xy[:, 1]
    ref_x = [p[2] for p in refined_poses]
    ref_y = [p[3] for p in refined_poses]
    
    # Load wall points for background
    ply = PlyData.read(MAP_PLY)
    mx, my, mz = np.array(ply['vertex']['x']), np.array(ply['vertex']['y']), np.array(ply['vertex']['z'])
    walls = (mz > 0.2) & (mz < 1.5)
    
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    for ax, title, px, py in [(axes[0], 'Hand-drawn (initial)', init_x, init_y),
                                (axes[1], 'ICP Refined', ref_x, ref_y)]:
        ax.scatter(mx[walls], my[walls], s=0.2, c='gray', alpha=0.3)
        ax.plot(px, py, 'r-', lw=1.5, alpha=0.8)
        ax.plot(px[0], py[0], 'go', ms=10, label='Start')
        ax.plot(px[-1], py[-1], 'r^', ms=10, label='End')
        ax.set_title(title, fontsize=14)
        ax.set_aspect('equal')
        ax.legend(); ax.grid(True, alpha=0.2)
    
    ext_r = [max(ref_x)-min(ref_x), max(ref_y)-min(ref_y)]
    gap = np.sqrt((ref_x[-1]-ref_x[0])**2 + (ref_y[-1]-ref_y[0])**2)
    path = sum(np.sqrt((ref_x[i+1]-ref_x[i])**2 + (ref_y[i+1]-ref_y[i])**2) for i in range(len(ref_x)-1))
    fig.suptitle(f'Refined: {ext_r[0]:.1f}m x {ext_r[1]:.1f}m, gap={gap:.2f}m, path={path:.1f}m', fontsize=13)
    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=150)
    print(f"Saved {OUTPUT_PNG}")

if __name__ == '__main__':
    main()
