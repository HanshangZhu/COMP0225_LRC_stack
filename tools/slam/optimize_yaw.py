#!/usr/bin/env python3
"""Optimize per-scan yaw by aligning wall points with Cartographer map.
XY positions come from hand-drawn trajectory (fixed).
Yaw is optimized per scan via 1-DOF search against map wall KD-tree."""
import numpy as np
import json
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.ndimage import uniform_filter1d
from plyfile import PlyData
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

MAP_PLY = "carto_3d_map.ply"
WAYPOINTS = "hand_drawn_trajectory.json"
BAG = "cartographer_tuning_bag_2_clean"
MIN_Z, MAX_Z, MIN_XY = 0.15, 1.5, 0.3

def main():
    # 1. Load map wall points (2D)
    ply = PlyData.read(MAP_PLY)
    mx = np.array(ply['vertex']['x'])
    my = np.array(ply['vertex']['y'])
    mz = np.array(ply['vertex']['z'])
    wm = (mz > MIN_Z) & (mz < MAX_Z)
    map_xy = np.column_stack([mx[wm], my[wm]])
    tree = cKDTree(map_xy)
    print(f"Map: {wm.sum()} wall pts, KD-tree built")

    # 2. Load scans
    reader = SequentialReader()
    reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('',''))
    scans = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if 'cloud' in topic.lower():
            msg = deserialize_message(data, PointCloud2)
            pts = list(pc2.read_points(msg, skip_nans=True))
            # Extract wall points in body frame
            wall = [(p[0], p[1]) for p in pts 
                    if p[2] >= MIN_Z and p[2] <= MAX_Z and (p[0]**2+p[1]**2) >= MIN_XY**2]
            scans.append(np.array(wall) if wall else np.zeros((0,2)))
    print(f"Loaded {len(scans)} scans")

    # 3. Interpolate hand-drawn XY
    with open(WAYPOINTS) as f:
        wp = np.array(json.load(f)['waypoints'])
    dists = np.sqrt(np.sum(np.diff(wp, axis=0)**2, axis=1))
    cumlen = np.concatenate([[0], np.cumsum(dists)])
    N = len(scans)
    target = np.linspace(0, cumlen[-1], N)
    px = np.interp(target, cumlen, wp[:,0])
    py = np.interp(target, cumlen, wp[:,1])
    
    # Initial heading from smoothed gradient
    dx = uniform_filter1d(np.gradient(px), 30)
    dy = uniform_filter1d(np.gradient(py), 30)
    init_yaw = np.arctan2(dy, dx)

    # 4. Per-scan yaw optimization
    yaw_angles = np.linspace(-np.pi, np.pi, 360)  # 1 degree resolution
    opt_yaw = np.zeros(N)
    
    for i in range(N):
        wall_body = scans[i]
        if len(wall_body) < 3:
            opt_yaw[i] = init_yaw[i]
            continue
        
        best_score = 1e9
        best_yaw = init_yaw[i]
        
        # Coarse search: try all angles
        for yaw in yaw_angles:
            c, s = np.cos(yaw), np.sin(yaw)
            gx = c * wall_body[:,0] - s * wall_body[:,1] + px[i]
            gy = s * wall_body[:,0] + c * wall_body[:,1] + py[i]
            projected = np.column_stack([gx, gy])
            dists_nn, _ = tree.query(projected)
            score = np.mean(dists_nn)  # mean distance to nearest map wall
            if score < best_score:
                best_score = score
                best_yaw = yaw
        
        # Fine search: ±2 degrees around best
        fine = np.linspace(best_yaw - np.radians(2), best_yaw + np.radians(2), 40)
        for yaw in fine:
            c, s = np.cos(yaw), np.sin(yaw)
            gx = c * wall_body[:,0] - s * wall_body[:,1] + px[i]
            gy = s * wall_body[:,0] + c * wall_body[:,1] + py[i]
            projected = np.column_stack([gx, gy])
            dists_nn, _ = tree.query(projected)
            score = np.mean(dists_nn)
            if score < best_score:
                best_score = score
                best_yaw = yaw
        
        opt_yaw[i] = best_yaw
        if i % 100 == 0:
            print(f"  Scan {i}: yaw={np.degrees(best_yaw):.1f}° score={best_score:.3f}m "
                  f"(init={np.degrees(init_yaw[i]):.1f}°) wall={len(wall_body)}")
    
    # 5. Smooth optimized yaw (temporal consistency)
    # Unwrap then smooth to avoid 2pi jumps
    opt_yaw_unwrap = np.unwrap(opt_yaw)
    opt_yaw_smooth = uniform_filter1d(opt_yaw_unwrap, 5)

    # 6. Project with optimized yaw
    all_wx, all_wy = [], []
    for i in range(N):
        wall_body = scans[i]
        if len(wall_body) == 0: continue
        c, s = np.cos(opt_yaw_smooth[i]), np.sin(opt_yaw_smooth[i])
        gx = c * wall_body[:,0] - s * wall_body[:,1] + px[i]
        gy = s * wall_body[:,0] + c * wall_body[:,1] + py[i]
        all_wx.extend(gx)
        all_wy.extend(gy)

    # 7. Plot
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    
    # Left: optimized yaw projection
    ax = axes[0]
    ax.scatter(mx[wm], my[wm], s=0.3, c='red', alpha=0.2, label='Carto map')
    ax.scatter(all_wx, all_wy, s=0.1, c='blue', alpha=0.4, label='Optimized yaw')
    ax.plot(px, py, 'g-', lw=1, alpha=0.3)
    ax.set_title('Optimized yaw + hand-drawn XY', fontsize=13)
    ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.2)
    
    # Right: yaw comparison
    ax = axes[1]
    ax.plot(np.degrees(init_yaw), 'b-', alpha=0.3, label='Path heading')
    ax.plot(np.degrees(opt_yaw), 'r.', ms=1, alpha=0.3, label='Optimized (raw)')
    ax.plot(np.degrees(opt_yaw_smooth), 'g-', lw=1.5, label='Optimized (smooth)')
    ax.set_xlabel('Scan index'); ax.set_ylabel('Yaw (deg)')
    ax.set_title('Yaw over time', fontsize=13)
    ax.legend(); ax.grid(True, alpha=0.2)
    
    plt.tight_layout()
    plt.savefig('optimized_yaw_result.png', dpi=150)
    print(f"Saved optimized_yaw_result.png")
    print(f"Projected {len(all_wx)} wall points")

    # Save poses
    np.savetxt('optimized_poses.csv', 
               np.column_stack([np.arange(N), px, py, np.degrees(opt_yaw_smooth)]),
               delimiter=',', header='scan,x,y,yaw_deg', comments='')
    print("Saved optimized_poses.csv")

if __name__ == '__main__':
    main()
