#!/usr/bin/env python3
"""Optimized trajectory with strong smoothness.
Pass 1: Yaw optimization per scan (360° search)
Pass 2: XY optimization with smoothness — optimize in windows, smooth heavily
Pass 3: Re-optimize yaw with refined XY"""
import numpy as np
import json
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.ndimage import uniform_filter1d
from scipy.optimize import minimize
from plyfile import PlyData
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

MAP_PLY = "carto_3d_map.ply"
WAYPOINTS = "hand_drawn_trajectory.json"
BAG = "cartographer_tuning_bag_2_clean"
MIN_Z, MAX_Z, MIN_XY_SQ = 0.15, 1.5, 0.09

def main():
    # Load map walls
    ply = PlyData.read(MAP_PLY)
    mx, my, mz = np.array(ply['vertex']['x']), np.array(ply['vertex']['y']), np.array(ply['vertex']['z'])
    wm = (mz > MIN_Z) & (mz < MAX_Z)
    map_xy = np.column_stack([mx[wm], my[wm]])
    tree = cKDTree(map_xy)
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

    # Initial XY
    with open(WAYPOINTS) as f:
        wp = np.array(json.load(f)['waypoints'])
    dists_wp = np.sqrt(np.sum(np.diff(wp, axis=0)**2, axis=1))
    cumlen = np.concatenate([[0], np.cumsum(dists_wp)])
    target = np.linspace(0, cumlen[-1], N)
    px = np.interp(target, cumlen, wp[:,0])
    py = np.interp(target, cumlen, wp[:,1])

    # === Pass 1: Yaw ===
    print("Pass 1: Yaw...")
    yaw = np.zeros(N)
    angles = np.linspace(-np.pi, np.pi, 360)
    for i in range(N):
        w = scans[i]
        if len(w) < 3: 
            dx = np.gradient(px); dy = np.gradient(py)
            yaw[i] = np.arctan2(uniform_filter1d(dy,30)[i], uniform_filter1d(dx,30)[i])
            continue
        best_s = 1e9
        for a in angles:
            c, s = np.cos(a), np.sin(a)
            proj = np.column_stack([c*w[:,0]-s*w[:,1]+px[i], s*w[:,0]+c*w[:,1]+py[i]])
            d, _ = tree.query(proj)
            sc = np.mean(d)
            if sc < best_s: best_s = sc; yaw[i] = a
        # Fine
        for a in np.linspace(yaw[i]-np.radians(3), yaw[i]+np.radians(3), 60):
            c, s = np.cos(a), np.sin(a)
            proj = np.column_stack([c*w[:,0]-s*w[:,1]+px[i], s*w[:,0]+c*w[:,1]+py[i]])
            d, _ = tree.query(proj)
            sc = np.mean(d)
            if sc < best_s: best_s = sc; yaw[i] = a
        if i % 200 == 0: print(f"  {i}")
    yaw = uniform_filter1d(np.unwrap(yaw), 7)

    # === Pass 2: XY with scipy.optimize (smooth cost) ===
    # Instead of per-scan grid search, optimize the WAYPOINTS directly
    # Then re-interpolate. This gives smooth paths by construction.
    print("Pass 2: Optimizing waypoint positions...")
    
    # For each waypoint, find which scans are nearest
    n_wp = len(wp)
    wp_scan_idx = []  # for each waypoint, list of scan indices
    for j in range(n_wp):
        t_wp = cumlen[j]
        # Scans within ±2 waypoints worth of arc length
        t_lo = cumlen[max(0, j-1)]
        t_hi = cumlen[min(n_wp-1, j+1)]
        idx = np.where((target >= t_lo) & (target <= t_hi))[0]
        wp_scan_idx.append(idx)
    
    def cost_fn(params):
        """Cost = mean scan-to-map distance + smoothness penalty."""
        wxp = params[:n_wp]
        wyp = params[n_wp:]
        # Interpolate to scan positions
        ipx = np.interp(target, cumlen, wxp)
        ipy = np.interp(target, cumlen, wyp)
        
        total_dist = 0.0
        count = 0
        for i in range(0, N, 3):  # every 3rd scan for speed
            w = scans[i]
            if len(w) < 3: continue
            c, s = np.cos(yaw[i]), np.sin(yaw[i])
            proj = np.column_stack([c*w[:,0]-s*w[:,1]+ipx[i], s*w[:,0]+c*w[:,1]+ipy[i]])
            d, _ = tree.query(proj)
            total_dist += np.mean(d)
            count += 1
        
        data_cost = total_dist / max(count, 1)
        
        # Smoothness: lightly penalize deviation from hand-drawn shape
        smooth_cost = np.mean((wxp - wp[:,0])**2 + (wyp - wp[:,1])**2)
        
        # Also penalize non-smooth path (neighboring waypoint distances)
        dx = np.diff(wxp); dy = np.diff(wyp)
        seg_lens = np.sqrt(dx**2 + dy**2)
        # Penalize if segments vary wildly in length  
        jerk_cost = np.std(seg_lens) / (np.mean(seg_lens) + 1e-6)
        
        return data_cost + 0.1 * smooth_cost + 0.05 * jerk_cost
    
    x0 = np.concatenate([wp[:,0], wp[:,1]])
    print(f"  Initial cost: {cost_fn(x0):.4f}")
    
    result = minimize(cost_fn, x0, method='Nelder-Mead',
                     options={'maxiter': 20000, 'xatol': 0.005, 'fatol': 0.0005, 'disp': True})
    
    opt_wpx = result.x[:n_wp]
    opt_wpy = result.x[n_wp:]
    ox = np.interp(target, cumlen, opt_wpx)
    oy = np.interp(target, cumlen, opt_wpy)
    print(f"  Final cost: {result.fun:.4f}")
    print(f"  Max shift: {np.max(np.sqrt((opt_wpx-wp[:,0])**2+(opt_wpy-wp[:,1])**2)):.2f}m")

    # === Pass 3: Re-optimize yaw ===
    print("Pass 3: Yaw re-optimization...")
    for i in range(N):
        w = scans[i]
        if len(w) < 3: continue
        best_s = 1e9
        for a in angles:
            c, s = np.cos(a), np.sin(a)
            proj = np.column_stack([c*w[:,0]-s*w[:,1]+ox[i], s*w[:,0]+c*w[:,1]+oy[i]])
            d, _ = tree.query(proj)
            sc = np.mean(d)
            if sc < best_s: best_s = sc; yaw[i] = a
        for a in np.linspace(yaw[i]-np.radians(3), yaw[i]+np.radians(3), 60):
            c, s = np.cos(a), np.sin(a)
            proj = np.column_stack([c*w[:,0]-s*w[:,1]+ox[i], s*w[:,0]+c*w[:,1]+oy[i]])
            d, _ = tree.query(proj)
            sc = np.mean(d)
            if sc < best_s: best_s = sc; yaw[i] = a
        if i % 200 == 0: print(f"  {i}")
    yaw = uniform_filter1d(np.unwrap(yaw), 7)

    # Project
    all_wx, all_wy = [], []
    for i in range(N):
        w = scans[i]
        if len(w) == 0: continue
        c, s = np.cos(yaw[i]), np.sin(yaw[i])
        all_wx.extend(c*w[:,0]-s*w[:,1]+ox[i])
        all_wy.extend(s*w[:,0]+c*w[:,1]+oy[i])

    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    ax = axes[0]
    ax.scatter(mx[wm], my[wm], s=0.3, c='red', alpha=0.2, label='Carto map')
    ax.scatter(all_wx, all_wy, s=0.1, c='blue', alpha=0.4, label='Optimized')
    ax.plot(ox, oy, 'g-', lw=1.5, alpha=0.7, label='Optimized path')
    ax.plot(ox[0], oy[0], 'go', ms=12)
    ax.set_title('Waypoint-optimized projection', fontsize=13)
    ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.2)

    ax = axes[1]
    ax.plot(ox, oy, 'g-o', ms=2, lw=1.5, label='Optimized')
    ax.plot(px, py, 'r--', lw=1, alpha=0.5, label='Hand-drawn')
    gap = np.sqrt((ox[-1]-ox[0])**2 + (oy[-1]-oy[0])**2)
    ext = [ox.max()-ox.min(), oy.max()-oy.min()]
    path_len = np.sum(np.sqrt(np.diff(ox)**2 + np.diff(oy)**2))
    ax.set_title(f'Path: {ext[0]:.1f}x{ext[1]:.1f}m, gap={gap:.2f}m, len={path_len:.1f}m')
    ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig('full_optimized_result.png', dpi=150)
    print("Saved full_optimized_result.png")
    np.savetxt('full_optimized_poses.csv',
               np.column_stack([np.arange(N), ox, oy, np.degrees(yaw)]),
               delimiter=',', header='scan,x,y,yaw_deg', comments='')

if __name__ == '__main__':
    main()
