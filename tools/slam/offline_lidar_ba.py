#!/usr/bin/env python3
"""
offline_lidar_ba.py — Offline LiDAR Bundle Adjustment (BALM2-style)

Reads Point-LIO output (registered scans + odometry) from a ROS2 bag,
performs plane-feature-based bundle adjustment on the scan poses,
and compares raw vs optimized trajectories.

Algorithm (following BALM2 paper):
1. Extract keyframe scans + poses from bag
2. Voxelize points from all scans using initial poses
3. For each voxel, fit a plane via PCA — keep if eigenvalue ratio passes threshold
4. Build cost: sum of point-to-plane distances for all points in valid voxels
5. Optimize all scan poses (6-DOF) to minimize this cost
6. Optionally add ground constraint: penalize Z deviation and roll/pitch

Usage:
    python3 offline_lidar_ba.py <bag_path> [--output results.png]
"""

import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import time

# ROS2 bag reading
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ─── Bag reading helpers ───

def read_bag_messages(bag_path, topic):
    """Yield (timestamp_ns, deserialized_msg) for a topic."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        print(f"  Topic {topic} not found in bag. Available: {list(type_map.keys())}")
        return

    msg_type = get_message(type_map[topic])

    while reader.has_next():
        t, data, stamp = reader.read_next()
        if t == topic:
            msg = deserialize_message(data, msg_type)
            yield stamp, msg


def cloud_msg_to_numpy(cloud_msg):
    """Convert PointCloud2 to Nx3 numpy array (x,y,z only)."""
    import struct
    
    # Find x, y, z field offsets
    field_map = {}
    for f in cloud_msg.fields:
        field_map[f.name] = (f.offset, f.datatype)
    
    if 'x' not in field_map:
        return np.zeros((0, 3))
    
    points = []
    data = bytes(cloud_msg.data)
    point_step = cloud_msg.point_step
    
    x_off = field_map['x'][0]
    y_off = field_map['y'][0]
    z_off = field_map['z'][0]
    
    n_points = len(data) // point_step
    
    # Fast batch unpack
    arr = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        arr[i, 0] = struct.unpack_from('f', data, base + x_off)[0]
        arr[i, 1] = struct.unpack_from('f', data, base + y_off)[0]
        arr[i, 2] = struct.unpack_from('f', data, base + z_off)[0]
    
    # Remove NaN/inf
    valid = np.isfinite(arr).all(axis=1)
    return arr[valid]


def odom_to_pose(odom_msg):
    """Extract (position, quaternion) from Odometry message."""
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


# ─── Pose parameterization ───

def pose_to_params(pos, quat):
    """Pose → 6-DOF params [tx, ty, tz, rx, ry, rz] (Rodrigues)."""
    rotvec = R.from_quat(quat).as_rotvec()
    return np.concatenate([pos, rotvec])


def params_to_pose(params):
    """6-DOF params → (4x4 matrix)."""
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(params[3:6]).as_matrix()
    T[:3, 3] = params[0:3]
    return T


def transform_points(pts, T):
    """Apply 4x4 transform to Nx3 points."""
    ones = np.ones((pts.shape[0], 1))
    pts_h = np.hstack([pts, ones])
    return (T @ pts_h.T).T[:, :3]


# ─── Voxel plane detection ───

def find_plane_voxels(all_points_global, voxel_size=1.0, min_points=20, eigen_ratio=0.1):
    """
    Voxelize all points, fit planes in each voxel via PCA.
    Returns list of (centroid, normal, point_indices) for valid planes.
    """
    # Discretize to voxel grid
    voxel_keys = np.floor(all_points_global / voxel_size).astype(np.int32)
    
    # Group points by voxel
    from collections import defaultdict
    voxel_map = defaultdict(list)
    for i, key in enumerate(voxel_keys):
        voxel_map[tuple(key)].append(i)
    
    planes = []
    for key, indices in voxel_map.items():
        if len(indices) < min_points:
            continue
        
        pts = all_points_global[indices]
        centroid = pts.mean(axis=0)
        
        # PCA
        cov = np.cov((pts - centroid).T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        # eigenvalues sorted ascending — smallest corresponds to plane normal
        if eigenvalues[1] < 1e-10:
            continue
        
        ratio = eigenvalues[0] / eigenvalues[1]
        
        if ratio < eigen_ratio:
            # Valid plane
            normal = eigenvectors[:, 0]  # eigenvector of smallest eigenvalue
            planes.append({
                'centroid': centroid,
                'normal': normal,
                'indices': np.array(indices),
                'eigenvalues': eigenvalues
            })
    
    return planes


# ─── Bundle Adjustment cost function ───

def ba_cost(params_flat, scan_points_local, scan_sizes, planes, 
            ground_weight=10.0, z0=0.0):
    """
    BALM2-style cost: sum of squared point-to-plane distances.
    Plus ground constraint: penalize Z deviation and roll/pitch.
    
    params_flat: [6*N] — 6-DOF for each of N scans (scan 0 is fixed)
    """
    n_scans = len(scan_sizes)
    
    # Reconstruct poses
    poses = []
    for i in range(n_scans):
        p = params_flat[i*6:(i+1)*6]
        poses.append(params_to_pose(p))
    
    # Transform all local points to global using current poses
    all_global = np.zeros_like(scan_points_local)
    offset = 0
    for i, size in enumerate(scan_sizes):
        if size > 0:
            local_pts = scan_points_local[offset:offset+size]
            all_global[offset:offset+size] = transform_points(local_pts, poses[i])
        offset += size
    
    # Point-to-plane cost
    cost = 0.0
    for plane in planes:
        pts = all_global[plane['indices']]
        centroid = pts.mean(axis=0)
        
        # Re-fit plane on current points
        centered = pts - centroid
        cov = centered.T @ centered / len(pts)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normal = eigenvectors[:, 0]
        
        # Point-to-plane distances
        dists = centered @ normal
        cost += np.sum(dists ** 2)
    
    # Ground constraint: penalize Z and roll/pitch for each scan
    for i in range(n_scans):
        z = params_flat[i*6 + 2]  # tz
        rx = params_flat[i*6 + 3]  # roll (Rodrigues x)
        ry = params_flat[i*6 + 4]  # pitch (Rodrigues y)
        cost += ground_weight * (z - z0) ** 2
        cost += ground_weight * rx ** 2
        cost += ground_weight * ry ** 2
    
    return cost


# ─── Main ───

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Offline LiDAR Bundle Adjustment')
    parser.add_argument('bag_path', help='Path to ROS2 bag')
    parser.add_argument('--cloud-topic', default='/registered_scan', help='Point cloud topic')
    parser.add_argument('--odom-topic', default='/state_estimation', help='Odometry topic')
    parser.add_argument('--output', default='ba_trajectory.png', help='Output plot path')
    parser.add_argument('--keyframe-dist', type=float, default=0.5, help='Min distance between keyframes (m)')
    parser.add_argument('--max-keyframes', type=int, default=100, help='Max keyframes to use')
    parser.add_argument('--voxel-size', type=float, default=1.0, help='Voxel size for plane detection (m)')
    parser.add_argument('--downsample', type=float, default=0.1, help='Point cloud downsample leaf size (m)')
    parser.add_argument('--ground-weight', type=float, default=10.0, help='Ground constraint weight')
    parser.add_argument('--max-iter', type=int, default=50, help='Max optimization iterations')
    args = parser.parse_args()
    
    print("=" * 60)
    print("  Offline LiDAR Bundle Adjustment (BALM2-style)")
    print("=" * 60)
    
    # ── Step 1: Extract keyframes ──
    print(f"\n1. Reading bag: {args.bag_path}")
    
    # Read all odom first
    odom_poses = []
    odom_times = []
    for stamp, msg in read_bag_messages(args.bag_path, args.odom_topic):
        pos, quat = odom_to_pose(msg)
        odom_poses.append((pos, quat))
        odom_times.append(stamp)
    
    print(f"   {len(odom_poses)} odom messages")
    
    # Read clouds and pair with nearest odom
    keyframes = []  # (pos, quat, cloud_local)
    last_kf_pos = None
    cloud_count = 0
    
    for stamp, msg in read_bag_messages(args.bag_path, args.cloud_topic):
        cloud_count += 1
        
        # Find nearest odom by timestamp
        if not odom_times:
            continue
        idx = np.searchsorted(odom_times, stamp)
        idx = min(idx, len(odom_poses) - 1)
        pos, quat = odom_poses[idx]
        
        # Keyframe selection by distance
        if last_kf_pos is not None:
            dist = np.linalg.norm(pos - last_kf_pos)
            if dist < args.keyframe_dist:
                continue
        
        # Extract point cloud
        pts = cloud_msg_to_numpy(msg)
        if len(pts) < 50:
            continue
        
        # Downsample (simple voxel grid)
        if args.downsample > 0:
            grid_keys = np.floor(pts / args.downsample).astype(np.int32)
            _, unique_idx = np.unique(grid_keys, axis=0, return_index=True)
            pts = pts[unique_idx]
        
        keyframes.append({
            'pos': pos.copy(),
            'quat': quat.copy(),
            'cloud': pts,
            'time': stamp
        })
        last_kf_pos = pos.copy()
        
        if len(keyframes) >= args.max_keyframes:
            break
    
    print(f"   {cloud_count} clouds → {len(keyframes)} keyframes selected")
    
    if len(keyframes) < 3:
        print("   Not enough keyframes for BA. Exiting.")
        return
    
    # ── Step 2: Build global point array ──
    print(f"\n2. Building global point cloud ({len(keyframes)} scans)")
    
    scan_points_local = []
    scan_sizes = []
    initial_params = []
    
    for kf in keyframes:
        scan_points_local.append(kf['cloud'])
        scan_sizes.append(len(kf['cloud']))
        initial_params.append(pose_to_params(kf['pos'], kf['quat']))
    
    scan_points_local = np.vstack(scan_points_local)
    initial_params = np.concatenate(initial_params)
    
    total_points = len(scan_points_local)
    print(f"   Total points: {total_points}")
    
    # Transform to global for initial plane detection
    all_global = np.zeros_like(scan_points_local)
    offset = 0
    for i, size in enumerate(scan_sizes):
        T = params_to_pose(initial_params[i*6:(i+1)*6])
        all_global[offset:offset+size] = transform_points(
            scan_points_local[offset:offset+size], T)
        offset += size
    
    # ── Step 3: Find plane features ──
    print(f"\n3. Finding plane features (voxel_size={args.voxel_size}m)")
    planes = find_plane_voxels(all_global, 
                               voxel_size=args.voxel_size, 
                               min_points=20, 
                               eigen_ratio=0.1)
    
    total_plane_points = sum(len(p['indices']) for p in planes)
    print(f"   {len(planes)} plane voxels found, covering {total_plane_points}/{total_points} points")
    
    if len(planes) < 10:
        print("   Too few planes. Try larger voxel_size or looser eigen_ratio.")
        # Try with more relaxed parameters
        planes = find_plane_voxels(all_global, voxel_size=2.0, min_points=10, eigen_ratio=0.2)
        total_plane_points = sum(len(p['indices']) for p in planes)
        print(f"   Retry with voxel=2.0: {len(planes)} planes, {total_plane_points} points")
    
    # ── Step 4: Optimize ──
    print(f"\n4. Running bundle adjustment (max_iter={args.max_iter})")
    
    z0 = initial_params[2]  # Initial Z of first scan as ground reference
    
    # Fix first scan
    x0 = initial_params.copy()
    fixed_first = x0[:6].copy()
    
    t0 = time.time()
    
    result = minimize(
        ba_cost,
        x0,
        args=(scan_points_local, scan_sizes, planes, args.ground_weight, z0),
        method='L-BFGS-B',
        options={'maxiter': args.max_iter, 'disp': True, 'ftol': 1e-10}
    )
    
    elapsed = time.time() - t0
    print(f"   Optimization: {elapsed:.1f}s, success={result.success}")
    print(f"   Cost: {ba_cost(initial_params, scan_points_local, scan_sizes, planes, args.ground_weight, z0):.2f}"
          f" → {result.fun:.2f}")
    
    # Force first scan to stay fixed
    optimized_params = result.x.copy()
    optimized_params[:6] = fixed_first
    
    # ── Step 5: Extract trajectories ──
    print(f"\n5. Extracting trajectories")
    
    n = len(keyframes)
    raw_traj = np.zeros((n, 3))
    opt_traj = np.zeros((n, 3))
    
    for i in range(n):
        raw_traj[i] = initial_params[i*6:i*6+3]
        opt_traj[i] = optimized_params[i*6:i*6+3]
    
    # Metrics
    raw_gap = np.linalg.norm(raw_traj[-1, :2] - raw_traj[0, :2])
    opt_gap = np.linalg.norm(opt_traj[-1, :2] - opt_traj[0, :2])
    raw_zdrift = abs(raw_traj[-1, 2] - raw_traj[0, 2])
    opt_zdrift = abs(opt_traj[-1, 2] - opt_traj[0, 2])
    
    raw_dist = np.sum(np.linalg.norm(np.diff(raw_traj, axis=0), axis=1))
    opt_dist = np.sum(np.linalg.norm(np.diff(opt_traj, axis=0), axis=1))
    
    print(f"\n{'='*60}")
    print(f"  {'Metric':<20} {'Raw':>12} {'BA-Optimized':>12} {'Change':>10}")
    print(f"  {'-'*54}")
    print(f"  {'Loop gap (XY)':<20} {raw_gap:>10.2f}m {opt_gap:>10.2f}m {(opt_gap-raw_gap)/raw_gap*100:>+8.1f}%")
    print(f"  {'Z drift':<20} {raw_zdrift:>10.2f}m {opt_zdrift:>10.2f}m {(opt_zdrift-raw_zdrift)/(raw_zdrift+1e-6)*100:>+8.1f}%")
    print(f"  {'Distance':<20} {raw_dist:>10.2f}m {opt_dist:>10.2f}m {(opt_dist-raw_dist)/raw_dist*100:>+8.1f}%")
    print(f"{'='*60}")
    
    # ── Step 6: Plot ──
    print(f"\n6. Plotting to {args.output}")
    
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # XY trajectory
    ax = axes[0]
    ax.plot(raw_traj[:, 0], raw_traj[:, 1], 'r-', alpha=0.7, label=f'Raw (gap={raw_gap:.1f}m)')
    ax.plot(opt_traj[:, 0], opt_traj[:, 1], 'b-', alpha=0.7, label=f'BA (gap={opt_gap:.1f}m)')
    ax.plot(raw_traj[0, 0], raw_traj[0, 1], 'go', ms=10, label='Start')
    ax.plot(raw_traj[-1, 0], raw_traj[-1, 1], 'rs', ms=10, label='End (raw)')
    ax.plot(opt_traj[-1, 0], opt_traj[-1, 1], 'bs', ms=10, label='End (BA)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('XY Trajectory'); ax.legend(fontsize=8)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    
    # Z over time
    ax = axes[1]
    t = np.arange(n)
    ax.plot(t, raw_traj[:, 2], 'r-', label=f'Raw (drift={raw_zdrift:.2f}m)')
    ax.plot(t, opt_traj[:, 2], 'b-', label=f'BA (drift={opt_zdrift:.2f}m)')
    ax.axhline(y=z0, color='g', linestyle='--', alpha=0.5, label='Ground')
    ax.set_xlabel('Keyframe #'); ax.set_ylabel('Z (m)')
    ax.set_title('Z / Altitude'); ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Position corrections
    ax = axes[2]
    corrections = opt_traj - raw_traj
    ax.plot(t, corrections[:, 0], label='dX')
    ax.plot(t, corrections[:, 1], label='dY')
    ax.plot(t, corrections[:, 2], label='dZ')
    ax.set_xlabel('Keyframe #'); ax.set_ylabel('Correction (m)')
    ax.set_title('BA Corrections'); ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"   Plot saved: {args.output}")


if __name__ == '__main__':
    main()
