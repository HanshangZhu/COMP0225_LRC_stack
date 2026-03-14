#!/usr/bin/env python3
"""Build occupancy grid from refined trajectory + projected scans.
Path A: Skip Carto's map, use our own trajectory as ground truth."""
import numpy as np
import json
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

BAG = "cartographer_tuning_bag_2_clean"
POSES = "loop_corrected_poses.csv"
RESOLUTION = 0.05  # meters per cell
MIN_Z, MAX_Z = 0.15, 1.5
MIN_RANGE_SQ = 0.09  # 0.3m

def main():
    # Load poses
    data = np.genfromtxt(POSES, delimiter=',', skip_header=1)
    px, py, yaw_deg = data[:,1], data[:,2], data[:,3]
    # Extra smoothing
    px = uniform_filter1d(px, 30)
    py = uniform_filter1d(py, 30)
    yaw = np.radians(uniform_filter1d(yaw_deg, 7))
    N = len(px)

    # Load scans
    reader = SequentialReader()
    reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('',''))
    scans_all = []  # all points, not just walls
    while reader.has_next():
        topic, data_raw, ts = reader.read_next()
        if 'cloud' in topic.lower():
            msg = deserialize_message(data_raw, PointCloud2)
            scans_all.append(list(pc2.read_points(msg, skip_nans=True)))
    print(f"Loaded {len(scans_all)} scans, {N} poses")

    # Project all points into global frame
    wall_pts = []
    all_pts = []
    for i in range(min(N, len(scans_all))):
        c, s = np.cos(yaw[i]), np.sin(yaw[i])
        for p in scans_all[i]:
            r2 = p[0]**2 + p[1]**2
            if r2 < MIN_RANGE_SQ: continue
            gx = c*p[0] - s*p[1] + px[i]
            gy = s*p[0] + c*p[1] + py[i]
            all_pts.append((gx, gy, p[2]))
            if p[2] >= MIN_Z and p[2] <= MAX_Z:
                wall_pts.append((gx, gy))

    all_pts = np.array(all_pts)
    wall_pts = np.array(wall_pts)
    print(f"Total projected: {len(all_pts)}, wall: {len(wall_pts)}")

    # Stats
    print(f"Wall X: [{wall_pts[:,0].min():.1f}, {wall_pts[:,0].max():.1f}]")
    print(f"Wall Y: [{wall_pts[:,1].min():.1f}, {wall_pts[:,1].max():.1f}]")

    # Build occupancy grid
    margin = 2.0
    x_min = wall_pts[:,0].min() - margin
    x_max = wall_pts[:,0].max() + margin
    y_min = wall_pts[:,1].min() - margin
    y_max = wall_pts[:,1].max() + margin
    
    nx = int((x_max - x_min) / RESOLUTION) + 1
    ny = int((y_max - y_min) / RESOLUTION) + 1
    
    # Hit count grid
    hits = np.zeros((ny, nx), dtype=np.float32)
    for wx, wy in wall_pts:
        ci = int((wx - x_min) / RESOLUTION)
        ri = int((wy - y_min) / RESOLUTION)
        if 0 <= ci < nx and 0 <= ri < ny:
            hits[ri, ci] += 1

    # Raytrace for free space
    free = np.zeros((ny, nx), dtype=np.float32)
    # Sample every 5th scan for speed
    for i in range(0, min(N, len(scans_all)), 5):
        cx = int((px[i] - x_min) / RESOLUTION)
        cy = int((py[i] - y_min) / RESOLUTION)
        c, s = np.cos(yaw[i]), np.sin(yaw[i])
        for p in scans_all[i]:
            r2 = p[0]**2 + p[1]**2
            if r2 < MIN_RANGE_SQ: continue
            gx = c*p[0] - s*p[1] + px[i]
            gy = s*p[0] + c*p[1] + py[i]
            ex = int((gx - x_min) / RESOLUTION)
            ey = int((gy - y_min) / RESOLUTION)
            # Bresenham-ish ray from robot to endpoint
            dx = abs(ex - cx); dy = abs(ey - cy)
            sx = 1 if cx < ex else -1
            sy = 1 if cy < ey else -1
            err = dx - dy
            rx, ry = cx, cy
            steps = 0
            while steps < 500:
                if 0 <= rx < nx and 0 <= ry < ny:
                    free[ry, rx] += 1
                if rx == ex and ry == ey: break
                e2 = 2 * err
                if e2 > -dy: err -= dy; rx += sx
                if e2 < dx: err += dx; ry += sy
                steps += 1

    # Occupancy: log-odds
    occ = np.full((ny, nx), 0.5)  # unknown
    total = hits + free
    mask = total > 0
    occ[mask] = hits[mask] / total[mask]
    
    # Threshold
    occupied = occ > 0.65
    free_space = occ < 0.35
    unknown = ~occupied & ~free_space

    # Plot
    fig, axes = plt.subplots(1, 3, figsize=(20, 7))
    
    # 1. Wall point density
    ax = axes[0]
    h = ax.imshow(np.log1p(hits), origin='lower', cmap='hot',
                  extent=[x_min, x_max, y_min, y_max])
    ax.plot(px, py, 'g-', lw=1, alpha=0.5)
    ax.set_title('Wall point density (log)', fontsize=13)
    ax.set_aspect('equal')
    plt.colorbar(h, ax=ax, shrink=0.7)

    # 2. Occupancy grid
    ax = axes[1]
    grid_img = np.ones((ny, nx, 3))
    grid_img[occupied] = [0, 0, 0]  # black = occupied
    grid_img[free_space] = [1, 1, 1]  # white = free
    grid_img[unknown] = [0.5, 0.5, 0.5]  # gray = unknown
    ax.imshow(grid_img, origin='lower', extent=[x_min, x_max, y_min, y_max])
    ax.plot(px, py, 'g-', lw=1.5, alpha=0.5)
    ax.set_title('Occupancy grid (projected scans)', fontsize=13)
    ax.set_aspect('equal')

    # 3. Scatter comparison
    ax = axes[2]
    ax.scatter(wall_pts[::3,0], wall_pts[::3,1], s=0.1, c='blue', alpha=0.3, label='Projected walls')
    ax.plot(px, py, 'g-', lw=1.5, alpha=0.5, label='Trajectory')
    ax.set_title('All projected wall points', fontsize=13)
    ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig('projected_map.png', dpi=150)
    print("Saved projected_map.png")

    # Save as PGM-style text (for ROS)
    # OccupancyGrid format: 0=free, 100=occupied, -1=unknown
    ros_grid = np.full((ny, nx), -1, dtype=np.int8)
    ros_grid[free_space] = 0
    ros_grid[occupied] = 100
    np.save('projected_occupancy.npy', ros_grid)
    print(f"Saved projected_occupancy.npy ({nx}x{ny} cells, res={RESOLUTION}m)")
    print(f"Origin: ({x_min:.2f}, {y_min:.2f})")

if __name__ == '__main__':
    main()
