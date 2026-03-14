#!/usr/bin/env python3
"""
reconstruct_scene.py — Full LiDAR+IMU offline scene reconstruction

Reads raw /utlidar/cloud + /utlidar/imu from a Go2 bag,
applies body-frame transforms, builds pose chain via IMU dead-reckoning + ICP,
detects loop closures, optimizes the pose graph, then reconstructs the scene.

Outputs:
  - reconstructed_scene.ply  (full point cloud)
  - reconstruction_result.png (trajectory + top-down map)

Usage:
  python3 reconstruct_scene.py cartographer_tuning_bag_2
"""

import sys, os, time, yaml
import numpy as np
from collections import defaultdict
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import minimize

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations
from transforms3d.quaternions import quat2mat

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ═══════════════════════════════════════════════════════════════
# 1. BAG READING + BODY-FRAME TRANSFORMS
# ═══════════════════════════════════════════════════════════════

def stamp_ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

def load_calib():
    calib = {
        'acc_bias_x': 0.0, 'acc_bias_y': 0.0, 'acc_bias_z': 0.0,
        'ang_bias_x': 0.0, 'ang_bias_y': 0.0, 'ang_bias_z': 0.0,
        'ang_z2x_proj': 0.15, 'ang_z2y_proj': -0.28,
    }
    for d in [os.path.expanduser('~/COMP0225_LRC_stack'), os.path.expanduser('~/Desktop')]:
        p = os.path.join(d, 'imu_calib_data.yaml')
        if os.path.exists(p):
            with open(p) as f:
                calib.update(yaml.safe_load(f))
            print(f"  Loaded calibration from {p}")
            break
    return calib

def cloud_msg_to_xyz(cloud_msg):
    """Fast PointCloud2 → Nx3 numpy using struct, skipping pc2.read_points_list."""
    import struct as st
    field_map = {f.name: f.offset for f in cloud_msg.fields}
    if 'x' not in field_map:
        return np.zeros((0, 3), dtype=np.float32)
    data = bytes(cloud_msg.data)
    ps = cloud_msg.point_step
    n = len(data) // ps
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    xo, yo, zo = field_map['x'], field_map['y'], field_map['z']
    # Batch unpack using numpy fromiter with memoryview
    arr = np.frombuffer(data, dtype=np.uint8).reshape(n, ps)
    x = np.frombuffer(arr[:, xo:xo+4].tobytes(), dtype=np.float32)
    y = np.frombuffer(arr[:, yo:yo+4].tobytes(), dtype=np.float32)
    z = np.frombuffer(arr[:, zo:zo+4].tobytes(), dtype=np.float32)
    xyz = np.column_stack([x, y, z])
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]

def transform_cloud_to_body(cloud_msg, body2cloud_mat, cam_offset, fbox):
    xyz = cloud_msg_to_xyz(cloud_msg)
    if len(xyz) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    xyz = xyz @ body2cloud_mat.T
    xyz[:, 2] -= cam_offset
    keep = ~(
        (xyz[:, 0] > fbox[0]) & (xyz[:, 0] < fbox[1]) &
        (xyz[:, 1] > fbox[2]) & (xyz[:, 1] < fbox[3]) &
        (xyz[:, 2] > fbox[4]) & (xyz[:, 2] < fbox[5])
    )
    return xyz[keep].astype(np.float32)

def transform_imu_to_body(imu_msg, calib):
    theta = 15.1 * np.pi / 180.0
    gx, gy, gz = imu_msg.angular_velocity.x, -imu_msg.angular_velocity.y, -imu_msg.angular_velocity.z
    gx2 = np.cos(theta)*gx - np.sin(theta)*gz
    gy2 = gy
    gz2 = np.sin(theta)*gx + np.cos(theta)*gz
    gx2 -= calib['ang_bias_x']; gy2 -= calib['ang_bias_y']; gz2 -= calib['ang_bias_z']
    gx2 += calib['ang_z2x_proj']*gz2; gy2 += calib['ang_z2y_proj']*gz2

    ax, ay, az = imu_msg.linear_acceleration.x, -imu_msg.linear_acceleration.y, -imu_msg.linear_acceleration.z
    ax2 = np.cos(theta)*ax - np.sin(theta)*az
    ay2 = ay
    az2 = np.sin(theta)*ax + np.cos(theta)*az
    ax2 -= calib['acc_bias_x']; ay2 -= calib['acc_bias_y']; az2 -= calib['acc_bias_z']
    return np.array([gx2, gy2, gz2]), np.array([ax2, ay2, az2])

def read_bag(bag_path, calib, body2cloud_mat, cam_offset, fbox, subsample=3):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    clouds, imus = [], []
    cloud_count = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == '/utlidar/cloud':
            cloud_count += 1
            if cloud_count % subsample != 0:
                continue
            msg = deserialize_message(data, PointCloud2)
            xyz = transform_cloud_to_body(msg, body2cloud_mat, cam_offset, fbox)
            if len(xyz) > 50:
                clouds.append((stamp_ns(msg), xyz))
        elif topic == '/utlidar/imu':
            msg = deserialize_message(data, Imu)
            gyro, acc = transform_imu_to_body(msg, calib)
            imus.append((stamp_ns(msg), gyro, acc))
    clouds.sort(key=lambda x: x[0])
    imus.sort(key=lambda x: x[0])
    print(f"  {cloud_count} total clouds → {len(clouds)} after subsample={subsample}")
    print(f"  {len(imus)} IMU readings")
    return clouds, imus


# ═══════════════════════════════════════════════════════════════
# 2. IMU DEAD RECKONING
# ═══════════════════════════════════════════════════════════════

def imu_dead_reckon(clouds, imus):
    n = len(clouds)
    poses = [np.eye(4)]
    imu_idx = 0
    for i in range(1, n):
        t0, t1 = clouds[i-1][0], clouds[i][0]
        omega_sum = np.zeros(3)
        while imu_idx < len(imus) and imus[imu_idx][0] < t0:
            imu_idx += 1
        j, prev_t = imu_idx, t0
        while j < len(imus) and imus[j][0] <= t1:
            dt = (imus[j][0] - prev_t) * 1e-9
            if 0 < dt < 0.1:
                omega_sum += imus[j][1] * dt
            prev_t = imus[j][0]; j += 1
        angle = np.linalg.norm(omega_sum)
        dR = Rot.from_rotvec(omega_sum).as_matrix() if angle > 1e-8 else np.eye(3)
        dT = np.eye(4); dT[:3, :3] = dR
        poses.append(poses[-1] @ dT)
    return poses


# ═══════════════════════════════════════════════════════════════
# 3. ICP
# ═══════════════════════════════════════════════════════════════

def downsample_voxel(pts, leaf=0.1):
    if len(pts) == 0: return pts
    keys = np.floor(pts / leaf).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]

def icp(source, target, init_T=np.eye(4), max_iter=30, tol=1e-5, max_dist=1.0):
    """Point-to-point ICP. Returns (4x4 transform, fitness, mean_error)."""
    src = source.copy()
    T = init_T.copy()
    tree = KDTree(target)
    prev_err = float('inf')
    for _ in range(max_iter):
        src_t = (T[:3, :3] @ src.T).T + T[:3, 3]
        dists, indices = tree.query(src_t, distance_upper_bound=max_dist)
        valid = dists < max_dist
        if valid.sum() < 10: break
        sv, tv = src_t[valid], target[indices[valid]]
        sc, tc = sv.mean(0), tv.mean(0)
        H = (sv - sc).T @ (tv - tc)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0: Vt[-1] *= -1; R = Vt.T @ U.T
        t = tc - R @ sc
        dT = np.eye(4); dT[:3, :3] = R; dT[:3, 3] = t
        T = dT @ T
        err = np.mean(np.linalg.norm(sv - tv, axis=1))
        if abs(prev_err - err) < tol: break
        prev_err = err
    src_f = (T[:3, :3] @ source.T).T + T[:3, 3]
    df, _ = tree.query(src_f, distance_upper_bound=max_dist)
    fitness = (df < max_dist).sum() / len(source)
    return T, fitness, prev_err

def build_icp_poses(clouds, imu_poses, leaf=0.15, max_dist=1.0):
    n = len(clouds)
    poses = [np.eye(4)]
    for i in range(1, n):
        src = downsample_voxel(clouds[i][1], leaf)
        tgt = downsample_voxel(clouds[i-1][1], leaf)
        T_rel_imu = np.linalg.inv(imu_poses[i-1]) @ imu_poses[i]
        tgt_g = (poses[-1][:3, :3] @ tgt.T).T + poses[-1][:3, 3]
        init_g = poses[-1] @ T_rel_imu
        T_icp, fitness, _ = icp(src, tgt_g, init_T=init_g, max_iter=40, max_dist=max_dist)
        if fitness < 0.1: T_icp = init_g
        poses.append(T_icp)
        if i % 50 == 0 or i == n-1:
            print(f"    ICP {i}/{n-1}: fitness={fitness:.3f}")
    return poses


# ═══════════════════════════════════════════════════════════════
# 4. LOOP CLOSURE DETECTION
# ═══════════════════════════════════════════════════════════════

def detect_loop_closures(clouds_local, poses, leaf=0.15, min_gap=30,
                          proximity_m=3.0, min_fitness=0.4, max_dist=0.8):
    """
    For each scan, check if any earlier scan (separated by min_gap)
    is within proximity_m. If so, ICP them — if fitness > threshold,
    record as loop closure.
    Returns list of (i, j, T_j_from_i, fitness).
    """
    n = len(poses)
    positions = np.array([p[:3, 3] for p in poses])
    closures = []

    # Build position tree over all scans
    pos_tree = KDTree(positions[:, :2])  # XY only

    for i in range(min_gap, n):
        # Find candidates: earlier scans within proximity
        nearby = pos_tree.query_ball_point(positions[i, :2], proximity_m)
        for j in nearby:
            if j >= i - min_gap:
                continue  # Too close in sequence, not a real loop

            # ICP between scan i and scan j (both in their global frames)
            src = downsample_voxel(clouds_local[i], leaf)
            tgt = downsample_voxel(clouds_local[j], leaf)

            # Transform both to global
            src_g = (poses[i][:3, :3] @ src.T).T + poses[i][:3, 3]
            tgt_g = (poses[j][:3, :3] @ tgt.T).T + poses[j][:3, 3]

            T_icp, fitness, err = icp(src_g, tgt_g, init_T=np.eye(4),
                                       max_iter=50, max_dist=max_dist)

            if fitness > min_fitness:
                # T_icp transforms src_global to align with tgt_global
                # This means: pose[j] ≈ T_icp @ pose[i]
                closures.append((i, j, fitness, err))

        if i % 50 == 0:
            print(f"    Loop check {i}/{n-1}: {len(closures)} closures found so far")

    print(f"  Total loop closures: {len(closures)}")
    return closures


# ═══════════════════════════════════════════════════════════════
# 5. POSE GRAPH OPTIMIZATION
# ═══════════════════════════════════════════════════════════════

def pose_to_6dof(T):
    return np.concatenate([T[:3, 3], Rot.from_matrix(T[:3, :3]).as_rotvec()])

def sixdof_to_pose(p):
    T = np.eye(4)
    T[:3, :3] = Rot.from_rotvec(p[3:6]).as_matrix()
    T[:3, 3] = p[0:3]
    return T

def transform_pts(pts, T):
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


def pose_graph_cost(params, n, odom_edges, loop_edges, odom_weight=1.0,
                     loop_weight=10.0, ground_weight=3.0, z0=0.0):
    """
    Pose graph cost:
      - Odometry edges: (i, i+1) with measured relative transform
      - Loop closure edges: (i, j) constraint that poses should align
      - Ground constraint: Z=z0, roll=pitch=0
    """
    cost = 0.0

    # Odometry edges
    for i, j, T_meas in odom_edges:
        Ti = sixdof_to_pose(params[i*6:(i+1)*6])
        Tj = sixdof_to_pose(params[j*6:(j+1)*6])
        T_pred = np.linalg.inv(Ti) @ Tj
        # Residual in translation
        dt = T_pred[:3, 3] - T_meas[:3, 3]
        cost += odom_weight * np.sum(dt**2)
        # Residual in rotation (log map)
        dR = T_meas[:3, :3].T @ T_pred[:3, :3]
        rv = Rot.from_matrix(dR).as_rotvec()
        cost += odom_weight * np.sum(rv**2)

    # Loop closure edges: penalize distance between poses[i] and poses[j]
    for i, j, fitness, _ in loop_edges:
        Ti = sixdof_to_pose(params[i*6:(i+1)*6])
        Tj = sixdof_to_pose(params[j*6:(j+1)*6])
        # The scans should see the same thing, so transform scan i by Ti
        # and scan j by Tj — they should overlap.
        # Simple constraint: positions should be close (weighted by fitness)
        w = loop_weight * fitness
        dt = Ti[:3, 3] - Tj[:3, 3]
        cost += w * np.sum(dt**2)
        # Rotation should be similar
        dR = Ti[:3, :3].T @ Tj[:3, :3]
        rv = Rot.from_matrix(dR).as_rotvec()
        cost += w * 0.5 * np.sum(rv**2)

    # Ground constraint
    for i in range(n):
        cost += ground_weight * (params[i*6+2] - z0)**2      # Z
        cost += ground_weight * params[i*6+3]**2              # roll
        cost += ground_weight * params[i*6+4]**2              # pitch

    return cost


def optimize_pose_graph(poses, loop_closures, max_iter=100):
    """Optimize poses with odometry + loop closure constraints."""
    n = len(poses)

    # Build odometry edges from consecutive pose pairs
    odom_edges = []
    for i in range(n - 1):
        T_rel = np.linalg.inv(poses[i]) @ poses[i+1]
        odom_edges.append((i, i+1, T_rel))

    params0 = np.concatenate([pose_to_6dof(p) for p in poses])
    z0 = params0[2]
    fixed = params0[:6].copy()

    c0 = pose_graph_cost(params0, n, odom_edges, loop_closures, z0=z0)
    print(f"  Initial PGO cost: {c0:.1f}")

    result = minimize(
        pose_graph_cost, params0,
        args=(n, odom_edges, loop_closures, 1.0, 10.0, 3.0, z0),
        method='L-BFGS-B',
        options={'maxiter': max_iter, 'disp': True, 'ftol': 1e-12})

    opt = result.x.copy()
    opt[:6] = fixed  # Pin first scan

    cf = pose_graph_cost(opt, n, odom_edges, loop_closures, z0=z0)
    print(f"  Final PGO cost: {cf:.1f} ({c0:.1f} → {cf:.1f})")

    return [sixdof_to_pose(opt[i*6:(i+1)*6]) for i in range(n)]


# ═══════════════════════════════════════════════════════════════
# 6. RECONSTRUCT + VISUALIZE
# ═══════════════════════════════════════════════════════════════

def write_ply(path, points, colors=None):
    n = len(points)
    with open(path, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if colors is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(n):
            line = f"{points[i,0]:.4f} {points[i,1]:.4f} {points[i,2]:.4f}"
            if colors is not None:
                line += f" {colors[i,0]} {colors[i,1]} {colors[i,2]}"
            f.write(line + "\n")


def reconstruct_and_render(clouds_local, poses_before, poses_after,
                            loop_closures, output_dir):
    """Build global cloud + render comparison views."""
    # Build global cloud with optimized poses
    all_pts = []
    n = len(poses_after)
    for i in range(n):
        pts_g = transform_pts(clouds_local[i], poses_after[i])
        all_pts.append(pts_g)
    all_pts = np.vstack(all_pts)

    # Height filter
    z_mask = (all_pts[:, 2] > -0.5) & (all_pts[:, 2] < 2.0)
    pts_f = all_pts[z_mask]

    # Write PLY
    ply_path = os.path.join(output_dir, 'reconstructed_scene.ply')
    write_ply(ply_path, pts_f)
    print(f"  PLY saved: {ply_path} ({len(pts_f)} points)")

    # ── Plots ──
    traj_b = np.array([p[:3, 3] for p in poses_before])
    traj_a = np.array([p[:3, 3] for p in poses_after])

    fig, axes = plt.subplots(1, 3, figsize=(21, 7))

    # 1. Trajectory comparison
    ax = axes[0]
    ax.plot(traj_b[:, 0], traj_b[:, 1], 'r-', lw=1, alpha=0.6, label='Before LC')
    ax.plot(traj_a[:, 0], traj_a[:, 1], 'b-', lw=1.5, alpha=0.8, label='After LC')
    ax.plot(traj_a[0, 0], traj_a[0, 1], 'go', ms=10, label='Start')
    ax.plot(traj_a[-1, 0], traj_a[-1, 1], 'bs', ms=8, label='End')
    # Draw loop closure lines
    for i, j, fit, _ in loop_closures:
        ax.plot([traj_a[i, 0], traj_a[j, 0]], [traj_a[i, 1], traj_a[j, 1]],
                'g-', lw=0.5, alpha=0.3)
    gap_b = np.linalg.norm(traj_b[-1, :2] - traj_b[0, :2])
    gap_a = np.linalg.norm(traj_a[-1, :2] - traj_a[0, :2])
    ax.set_title(f'Trajectory (gap: {gap_b:.2f}m → {gap_a:.2f}m)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

    # 2. Top-down density
    ax = axes[1]
    res = 0.05
    x_min, x_max = pts_f[:, 0].min() - 0.5, pts_f[:, 0].max() + 0.5
    y_min, y_max = pts_f[:, 1].min() - 0.5, pts_f[:, 1].max() + 0.5
    nx = int((x_max - x_min) / res)
    ny = int((y_max - y_min) / res)
    grid = np.zeros((ny, nx), dtype=np.float32)
    ix = ((pts_f[:, 0] - x_min) / res).astype(int).clip(0, nx-1)
    iy = ((pts_f[:, 1] - y_min) / res).astype(int).clip(0, ny-1)
    np.add.at(grid, (iy, ix), 1)
    cap = np.percentile(grid[grid > 0], 95) if (grid > 0).any() else 1
    grid = np.clip(grid, 0, cap)
    ax.imshow(grid, origin='lower', extent=[x_min, x_max, y_min, y_max],
              cmap='hot', interpolation='nearest')
    ax.plot(traj_a[:, 0], traj_a[:, 1], 'c-', lw=0.8, alpha=0.6)
    ax.set_title(f'Top-down ({x_max-x_min:.1f}m × {y_max-y_min:.1f}m)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')

    # 3. Z comparison
    ax = axes[2]
    ax.plot(traj_b[:, 2], 'r-', alpha=0.6, label=f'Before (drift={abs(traj_b[-1,2]-traj_b[0,2]):.2f}m)')
    ax.plot(traj_a[:, 2], 'b-', alpha=0.8, label=f'After (drift={abs(traj_a[-1,2]-traj_a[0,2]):.2f}m)')
    ax.set_title('Z altitude'); ax.set_xlabel('Scan #'); ax.set_ylabel('Z (m)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    img_path = os.path.join(output_dir, 'reconstruction_result.png')
    plt.savefig(img_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Plot saved: {img_path}")
    return img_path


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_path> [--subsample N] [--skip-ba]")
        sys.exit(1)

    bag_path = sys.argv[1]
    subsample = 3
    skip_ba = '--skip-ba' in sys.argv
    for i, a in enumerate(sys.argv):
        if a == '--subsample' and i+1 < len(sys.argv):
            subsample = int(sys.argv[i+1])

    output_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 60)
    print("  LiDAR+IMU Scene Reconstruction (with Loop Closure)")
    print("=" * 60)

    # Setup transforms
    calib = load_calib()
    cam_offset = 0.046825
    quat_cloud = tf_transformations.quaternion_from_euler(0, 2.87820258505555555556, 0)
    body2cloud_mat = quat2mat([quat_cloud[3], quat_cloud[0], quat_cloud[1], quat_cloud[2]])
    fbox = (-0.7, -0.1, -0.3, 0.3, -0.6 - cam_offset, 0 - cam_offset)

    # ── Step 1: Read bag ──
    print(f"\n1. Reading bag: {bag_path}")
    t0 = time.time()
    clouds, imus = read_bag(bag_path, calib, body2cloud_mat, cam_offset, fbox, subsample)
    print(f"  Done in {time.time()-t0:.1f}s")
    if len(clouds) < 5:
        print("Not enough clouds!"); return

    # ── Step 2: IMU dead reckoning ──
    print(f"\n2. IMU dead reckoning ({len(clouds)} scans)")
    t0 = time.time()
    imu_poses = imu_dead_reckon(clouds, imus)
    print(f"  Done in {time.time()-t0:.1f}s")

    # ── Step 3: ICP scan-to-scan ──
    print(f"\n3. ICP scan-to-scan registration")
    t0 = time.time()
    icp_poses = build_icp_poses(clouds, imu_poses, leaf=0.15, max_dist=1.0)
    print(f"  Done in {time.time()-t0:.1f}s")

    clouds_local = [c[1] for c in clouds]

    # ── Step 4: Loop closure detection ──
    print(f"\n4. Detecting loop closures")
    t0 = time.time()
    loop_closures = detect_loop_closures(
        clouds_local, icp_poses,
        leaf=0.15, min_gap=30, proximity_m=3.0,
        min_fitness=0.4, max_dist=0.8)
    print(f"  Done in {time.time()-t0:.1f}s")

    # ── Step 5: Pose graph optimization ──
    print(f"\n5. Pose graph optimization ({len(loop_closures)} loop constraints)")
    t0 = time.time()
    if len(loop_closures) > 0:
        opt_poses = optimize_pose_graph(icp_poses, loop_closures, max_iter=100)
    else:
        print("  No loop closures found — using ICP poses as-is")
        opt_poses = icp_poses
    print(f"  Done in {time.time()-t0:.1f}s")

    # ── Step 6: Reconstruct + Visualize ──
    print(f"\n6. Reconstructing scene")
    clouds_out = [downsample_voxel(c, 0.05) for c in clouds_local]
    img_path = reconstruct_and_render(clouds_out, icp_poses, opt_poses,
                                       loop_closures, output_dir)

    # Summary
    traj_b = np.array([p[:3, 3] for p in icp_poses])
    traj_a = np.array([p[:3, 3] for p in opt_poses])
    gap_b = np.linalg.norm(traj_b[-1, :2] - traj_b[0, :2])
    gap_a = np.linalg.norm(traj_a[-1, :2] - traj_a[0, :2])
    ext_x = traj_a[:, 0].max() - traj_a[:, 0].min()
    ext_y = traj_a[:, 1].max() - traj_a[:, 1].min()
    path_len = np.sum(np.linalg.norm(np.diff(traj_a, axis=0), axis=1))

    print(f"\n{'='*60}")
    print(f"  Arena extent:      {ext_x:.1f}m × {ext_y:.1f}m")
    print(f"  Path length:       {path_len:.1f}m")
    print(f"  Loop gap (XY):     {gap_b:.2f}m → {gap_a:.2f}m")
    print(f"  Z drift:           {abs(traj_b[-1,2]-traj_b[0,2]):.2f}m → {abs(traj_a[-1,2]-traj_a[0,2]):.2f}m")
    print(f"  Loop closures:     {len(loop_closures)}")
    print(f"  Scans used:        {len(clouds)}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
