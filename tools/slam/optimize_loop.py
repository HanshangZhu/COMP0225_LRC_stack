#!/usr/bin/env python3
"""Optimized trajectory with loop closure drift correction.
1. Yaw optimization per scan (map alignment)
2. Linear loop closure correction (distribute endpoint gap backwards)
3. Re-optimize yaw with corrected XY"""
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
MIN_Z, MAX_Z, MIN_XY_SQ = 0.15, 1.5, 0.09

def optimize_yaw_scan(wall_body, x, y, tree, angles_coarse):
    if len(wall_body) < 3: return 0.0, 1e9
    best_yaw, best_s = 0.0, 1e9
    for a in angles_coarse:
        c, s = np.cos(a), np.sin(a)
        proj = np.column_stack([c*wall_body[:,0]-s*wall_body[:,1]+x, 
                               s*wall_body[:,0]+c*wall_body[:,1]+y])
        d, _ = tree.query(proj)
        sc = np.mean(d)
        if sc < best_s: best_s = sc; best_yaw = a
    for a in np.linspace(best_yaw-np.radians(3), best_yaw+np.radians(3), 60):
        c, s = np.cos(a), np.sin(a)
        proj = np.column_stack([c*wall_body[:,0]-s*wall_body[:,1]+x, 
                               s*wall_body[:,0]+c*wall_body[:,1]+y])
        d, _ = tree.query(proj)
        sc = np.mean(d)
        if sc < best_s: best_s = sc; best_yaw = a
    return best_yaw, best_s

def main():
    # Load map
    ply = PlyData.read(MAP_PLY)
    mx, my, mz = np.array(ply['vertex']['x']), np.array(ply['vertex']['y']), np.array(ply['vertex']['z'])
    wm = (mz > MIN_Z) & (mz < MAX_Z)
    tree = cKDTree(np.column_stack([mx[wm], my[wm]]))
    print(f"Map: {wm.sum()} wall pts")

    # Load scans
    reader = SequentialReader()
    reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('',''))
    scans = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if 'cloud' in topic.lower():
            msg = deserialize_message(data, PointCloud2)
            pts = list(pc2.read_points(msg, skip_nans=True))
            wall = [(p[0], p[1]) for p in pts 
                    if p[2] >= MIN_Z and p[2] <= MAX_Z and (p[0]**2+p[1]**2) >= MIN_XY_SQ]
            scans.append(np.array(wall) if wall else np.zeros((0,2)))
    N = len(scans)
    print(f"Scans: {N}")

    # Hand-drawn waypoints -> interpolated XY
    with open(WAYPOINTS) as f:
        wp = np.array(json.load(f)['waypoints'])
    dists_wp = np.sqrt(np.sum(np.diff(wp, axis=0)**2, axis=1))
    cumlen = np.concatenate([[0], np.cumsum(dists_wp)])
    target = np.linspace(0, cumlen[-1], N)
    px = np.interp(target, cumlen, wp[:,0])
    py = np.interp(target, cumlen, wp[:,1])

    # === Step 1: Linear loop closure correction ===
    # Only correct the RETURN LEG (WP 42-62), not the entire trajectory.
    # The first half (WP 0-41) already aligns well.
    # gap_vector points from WP 62 current position to WP 0 position (target)
    gap_x = px[0] - px[-1]  # vector from end -> start
    gap_y = py[0] - py[-1]
    print(f"Loop gap before correction: dx={-gap_x:.3f}m dy={-gap_y:.3f}m = {np.sqrt(gap_x**2+gap_y**2):.3f}m")
    
    # Find scan index corresponding to WP 42 (return leg start)
    # WP 42 is at cumlen[42] along the path
    t_wp42 = cumlen[42]
    scan_wp42 = int(np.round(t_wp42 / cumlen[-1] * (N-1)))
    print(f"Return leg starts at scan {scan_wp42} (WP 42 of 62)")
    
    # Apply ramp correction ONLY from scan_wp42 to end
    px_corrected = px.copy()
    py_corrected = py.copy()
    n_return = N - scan_wp42
    for i in range(scan_wp42, N):
        frac = (i - scan_wp42) / max(n_return - 1, 1)
        px_corrected[i] = px[i] + frac * gap_x
        py_corrected[i] = py[i] + frac * gap_y
    
    gap_after = np.sqrt((px_corrected[-1]-px_corrected[0])**2 + (py_corrected[-1]-py_corrected[0])**2)
    print(f"After correction: gap={gap_after:.4f}m")

    # === Step 2: Yaw optimization (Pass 1) ===
    print("Pass 1: Yaw optimization...")
    angles = np.linspace(-np.pi, np.pi, 360)
    yaw = np.zeros(N)
    for i in range(N):
        yaw[i], _ = optimize_yaw_scan(scans[i], px_corrected[i], py_corrected[i], tree, angles)
        if i % 200 == 0: print(f"  {i}/{N}")
    yaw = uniform_filter1d(np.unwrap(yaw), 7)

    # === Step 3: Small XY refinement (constrained to ±0.3m) ===
    print("Pass 2: Local XY refinement...")
    ox, oy = px_corrected.copy(), py_corrected.copy()
    for i in range(N):
        w = scans[i]
        if len(w) < 3: continue
        best_s = 1e9
        bx, by = ox[i], oy[i]
        # Small grid ±0.3m, 0.05m steps
        for dx in np.arange(-0.3, 0.31, 0.05):
            for dy in np.arange(-0.3, 0.31, 0.05):
                x, y = ox[i]+dx, oy[i]+dy
                c, s = np.cos(yaw[i]), np.sin(yaw[i])
                proj = np.column_stack([c*w[:,0]-s*w[:,1]+x, s*w[:,0]+c*w[:,1]+y])
                d, _ = tree.query(proj)
                sc = np.mean(d)
                if sc < best_s: best_s = sc; bx = x; by = y
        ox[i], oy[i] = bx, by
    # Heavy smoothing to prevent jitter
    ox = uniform_filter1d(ox, 15)
    oy = uniform_filter1d(oy, 15)
    
    # Enforce loop closure after smoothing — ramp only on return leg
    gap_x2 = ox[-1] - ox[0]
    gap_y2 = oy[-1] - oy[0]
    for i in range(scan_wp42, N):
        frac = (i - scan_wp42) / max(N - 1 - scan_wp42, 1)
        ox[i] -= frac * gap_x2
        oy[i] -= frac * gap_y2
    print(f"  After smoothing+closure: gap={np.sqrt((ox[-1]-ox[0])**2+(oy[-1]-oy[0])**2):.4f}m")

    # === Step 4: Re-optimize yaw ===
    print("Pass 3: Final yaw optimization...")
    for i in range(N):
        yaw[i], _ = optimize_yaw_scan(scans[i], ox[i], oy[i], tree, angles)
        if i % 200 == 0: print(f"  {i}/{N}")
    yaw = uniform_filter1d(np.unwrap(yaw), 7)

    # === Project ===
    all_wx, all_wy = [], []
    for i in range(N):
        w = scans[i]
        if len(w) == 0: continue
        c, s = np.cos(yaw[i]), np.sin(yaw[i])
        all_wx.extend(c*w[:,0]-s*w[:,1]+ox[i])
        all_wy.extend(s*w[:,0]+c*w[:,1]+oy[i])

    # === Plot ===
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    ax = axes[0]
    ax.scatter(mx[wm], my[wm], s=0.3, c='red', alpha=0.25, label='Carto map')
    ax.scatter(all_wx, all_wy, s=0.1, c='blue', alpha=0.4, label='Projected scans')
    ax.plot(ox, oy, 'g-', lw=1.5, alpha=0.7, label='Optimized path')
    ax.plot(ox[0], oy[0], 'go', ms=12, label='Start')
    ax.set_title('Loop-corrected + refined', fontsize=13)
    ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.2)

    ax = axes[1]
    ax.plot(ox, oy, 'g-', lw=2, label='Optimized')
    ax.plot(px, py, 'r--', lw=1, alpha=0.5, label='Hand-drawn')
    ax.plot(px_corrected, py_corrected, 'b:', lw=1, alpha=0.5, label='Loop-corrected')
    gap = np.sqrt((ox[-1]-ox[0])**2 + (oy[-1]-oy[0])**2)
    ext = [ox.max()-ox.min(), oy.max()-oy.min()]
    path_len = np.sum(np.sqrt(np.diff(ox)**2 + np.diff(oy)**2))
    ax.set_title(f'Path: {ext[0]:.1f}x{ext[1]:.1f}m, gap={gap:.3f}m, len={path_len:.1f}m')
    ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig('loop_corrected_result.png', dpi=150)
    print(f"Saved loop_corrected_result.png")

    np.savetxt('loop_corrected_poses.csv',
               np.column_stack([np.arange(N), ox, oy, np.degrees(yaw)]),
               delimiter=',', header='scan,x,y,yaw_deg', comments='')
    print("Saved loop_corrected_poses.csv")

if __name__ == '__main__':
    main()
