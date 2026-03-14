#!/usr/bin/env python3
"""
offline_ba_pgo.py — Chained LiDAR BA → Pose Graph Optimization

Pipeline:
  Stage 1 (BA):  Plane-feature bundle adjustment + ground constraint → fixes Z-drift
  Stage 2 (PGO): Distance-based loop detection + ICP → closes XY loops

Usage:
    python3 offline_ba_pgo.py data/hallway_run/ --output data/ba_pgo_result.png
"""

import sys, os, argparse, time
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
from scipy.optimize import minimize
from collections import defaultdict
import struct

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# ═══════════════════════════════════════════════════════════════
# Bag reading
# ═══════════════════════════════════════════════════════════════

def read_bag_messages(bag_path, topic):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        return
    msg_type = get_message(topic_types[topic])
    while reader.has_next():
        t, data, stamp = reader.read_next()
        if t == topic:
            yield stamp, deserialize_message(data, msg_type)


def cloud_to_numpy(msg):
    fields = {f.name: f.offset for f in msg.fields}
    if 'x' not in fields:
        return np.zeros((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    pts = np.zeros((n, 3), dtype=np.float32)
    for i, f in enumerate(['x', 'y', 'z']):
        off = fields[f]
        pts[:, i] = np.frombuffer(raw[:, off:off+4].tobytes(), dtype=np.float32)
    valid = np.isfinite(pts).all(axis=1)
    pts = pts[valid]
    dist = np.linalg.norm(pts, axis=1)
    return pts[(dist > 0.2) & (dist < 30.0)]


def odom_to_pose6(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
    return np.array([p.x, p.y, p.z, rpy[0], rpy[1], rpy[2]])


# ═══════════════════════════════════════════════════════════════
# Pose helpers
# ═══════════════════════════════════════════════════════════════

def pose6_to_T(p):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', p[3:6]).as_matrix()
    T[:3, 3] = p[:3]
    return T

def T_to_pose6(T):
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
    return np.array([T[0,3], T[1,3], T[2,3], rpy[0], rpy[1], rpy[2]])

def pose6_to_rotvec(p):
    rv = Rotation.from_euler('xyz', p[3:6]).as_rotvec()
    return np.concatenate([p[:3], rv])

def rotvec_to_T(rv):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rv[3:6]).as_matrix()
    T[:3, 3] = rv[:3]
    return T

def transform_pts(pts, T):
    return (T[:3, :3] @ pts.T).T + T[:3, 3]

def downsample_voxel(pts, leaf=0.1):
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


# ═══════════════════════════════════════════════════════════════
# Stage 1: LiDAR Bundle Adjustment (BALM2-style)
# ═══════════════════════════════════════════════════════════════

def find_planes(all_pts, voxel_size=1.5, min_pts=20, eigen_ratio=0.1):
    keys = np.floor(all_pts / voxel_size).astype(np.int32)
    voxel_map = defaultdict(list)
    for i, k in enumerate(keys):
        voxel_map[tuple(k)].append(i)
    
    planes = []
    for key, indices in voxel_map.items():
        if len(indices) < min_pts:
            continue
        pts = all_pts[indices]
        centroid = pts.mean(axis=0)
        cov = np.cov((pts - centroid).T)
        evals, evecs = np.linalg.eigh(cov)
        if evals[1] < 1e-10:
            continue
        if evals[0] / evals[1] < eigen_ratio:
            planes.append({'centroid': centroid, 'normal': evecs[:,0],
                          'indices': np.array(indices)})
    return planes


def ba_cost(params, local_pts, sizes, planes, gw, z0):
    n = len(sizes)
    poses = [rotvec_to_T(params[i*6:(i+1)*6]) for i in range(n)]
    
    all_g = np.zeros_like(local_pts)
    off = 0
    for i, s in enumerate(sizes):
        if s > 0:
            all_g[off:off+s] = transform_pts(local_pts[off:off+s], poses[i])
        off += s
    
    cost = 0.0
    for plane in planes:
        pts = all_g[plane['indices']]
        c = pts.mean(axis=0)
        centered = pts - c
        cov = centered.T @ centered / len(pts)
        _, evecs = np.linalg.eigh(cov)
        n_vec = evecs[:, 0]
        cost += np.sum((centered @ n_vec) ** 2)
    
    for i in range(n):
        z = params[i*6 + 2]
        rx, ry = params[i*6 + 3], params[i*6 + 4]
        cost += gw * ((z - z0)**2 + rx**2 + ry**2)
    
    return cost


def run_ba(keyframes, voxel_size=1.5, ground_weight=20.0, max_iter=30):
    """Run BA on keyframes. Returns BA-corrected pose6 list."""
    print("\n  ── Stage 1: LiDAR Bundle Adjustment ──")
    
    local_pts_list = []
    sizes = []
    params = []
    
    for kf in keyframes:
        pts = downsample_voxel(kf['cloud'], leaf=0.1)
        local_pts_list.append(pts)
        sizes.append(len(pts))
        params.append(pose6_to_rotvec(kf['pose']))
    
    local_pts = np.vstack(local_pts_list) if local_pts_list else np.zeros((0,3))
    params = np.concatenate(params)
    z0 = params[2]
    
    # Transform to global for plane detection
    all_g = np.zeros_like(local_pts)
    off = 0
    for i, s in enumerate(sizes):
        T = rotvec_to_T(params[i*6:(i+1)*6])
        all_g[off:off+s] = transform_pts(local_pts[off:off+s], T)
        off += s
    
    planes = find_planes(all_g, voxel_size=voxel_size)
    plane_pts = sum(len(p['indices']) for p in planes)
    print(f"    {len(planes)} plane voxels, {plane_pts}/{len(local_pts)} points")
    
    if len(planes) < 5:
        planes = find_planes(all_g, voxel_size=2.5, min_pts=10, eigen_ratio=0.2)
        plane_pts = sum(len(p['indices']) for p in planes)
        print(f"    Retry relaxed: {len(planes)} planes, {plane_pts} points")
    
    fixed = params[:6].copy()
    c0 = ba_cost(params, local_pts, sizes, planes, ground_weight, z0)
    
    result = minimize(ba_cost, params,
                      args=(local_pts, sizes, planes, ground_weight, z0),
                      method='L-BFGS-B',
                      options={'maxiter': max_iter, 'disp': False, 'ftol': 1e-10})
    
    opt = result.x.copy()
    opt[:6] = fixed  # Keep first scan fixed
    c1 = ba_cost(opt, local_pts, sizes, planes, ground_weight, z0)
    print(f"    BA cost: {c0:.1f} → {c1:.1f} ({(1-c1/c0)*100:.0f}% reduction)")
    
    # Convert back to pose6
    ba_poses = []
    for i in range(len(keyframes)):
        T = rotvec_to_T(opt[i*6:(i+1)*6])
        ba_poses.append(T_to_pose6(T))
    
    z_raw = abs(keyframes[-1]['pose'][2] - keyframes[0]['pose'][2])
    z_ba = abs(ba_poses[-1][2] - ba_poses[0][2])
    print(f"    Z-drift: {z_raw:.2f}m → {z_ba:.2f}m")
    
    return ba_poses


# ═══════════════════════════════════════════════════════════════
# Stage 2: PGO with distance-based loop detection + ICP
# ═══════════════════════════════════════════════════════════════

def simple_icp(source, target, max_iter=50, tol=1e-5, max_dist=2.0):
    if len(source) < 10 or len(target) < 10:
        return np.eye(4), 1e6
    src = source[:2000].copy() if len(source) > 2000 else source.copy()
    tgt = target[:5000] if len(target) > 5000 else target
    tree = KDTree(tgt)
    T = np.eye(4)
    
    for _ in range(max_iter):
        dists, indices = tree.query(src)
        mask = dists < max_dist
        if mask.sum() < 10:
            break
        sm, tm = src[mask], tgt[indices[mask]]
        sc, tc = sm.mean(0), tm.mean(0)
        H = (sm - sc).T @ (tm - tc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = tc - R @ sc
        src = (R @ src.T).T + t
        dT = np.eye(4); dT[:3,:3] = R; dT[:3,3] = t
        T = dT @ T
        if np.linalg.norm(t) < tol:
            break
    
    d2, _ = tree.query(src)
    inl = d2 < max_dist
    fitness = d2[inl].mean() if inl.sum() > 0 else 1e6
    return T, fitness


def find_distance_loops(poses, min_gap=15, dist_thresh=3.0):
    """Find loop closure candidates by distance."""
    loops = []
    positions = np.array([p[:3] for p in poses])
    
    for i in range(min_gap, len(poses)):
        for j in range(0, i - min_gap):
            d = np.linalg.norm(positions[i, :2] - positions[j, :2])
            if d < dist_thresh:
                loops.append((j, i, d))
    
    # Deduplicate — keep best per query
    seen = set()
    unique = []
    for j, i, d in sorted(loops, key=lambda x: x[2]):
        if i not in seen:
            unique.append((j, i, d))
            seen.add(i)
    return unique


def optimize_pgo(poses, odom_edges, loop_edges, n_iter=20):
    """3-DOF (x, y, yaw) pose graph optimization."""
    n = len(poses)
    if n < 2 or not loop_edges:
        return poses
    
    state = np.zeros(n * 3)
    for i, p in enumerate(poses):
        state[i*3:i*3+3] = [p[0], p[1], p[5]]
    
    for _ in range(n_iter):
        H = lil_matrix((n*3, n*3))
        b = np.zeros(n*3)
        for d in range(3):
            H[d,d] += 1e6
        
        all_edges = [(i,j,r,100.0) for i,j,r in odom_edges] + \
                     [(i,j,r,w) for i,j,r,w in loop_edges]
        
        for i, j, rel, w in all_edges:
            xi, yi, thi = state[i*3:i*3+3]
            xj, yj, thj = state[j*3:j*3+3]
            c, s = np.cos(thi), np.sin(thi)
            dx_w, dy_w = xj-xi, yj-yi
            ex = c*dx_w + s*dy_w - rel[0]
            ey = -s*dx_w + c*dy_w - rel[1]
            eth = ((thj-thi-rel[5]) + np.pi) % (2*np.pi) - np.pi
            e = np.array([ex, ey, eth])
            
            e_sq = np.dot(e, e)
            rw = w / (1.0 + e_sq)
            
            J = np.zeros((3, 6))
            J[0,0]=-c; J[0,1]=-s; J[0,2]=-s*dx_w+c*dy_w; J[0,3]=c; J[0,4]=s
            J[1,0]=s; J[1,1]=-c; J[1,2]=-c*dx_w-s*dy_w; J[1,3]=-s; J[1,4]=c
            J[2,2]=-1; J[2,5]=1
            
            W = rw * np.eye(3)
            idx = [i*3, i*3+1, i*3+2, j*3, j*3+1, j*3+2]
            JtW = J.T @ W
            Hl = JtW @ J; bl = JtW @ e
            for a in range(6):
                for bb in range(6):
                    H[idx[a], idx[bb]] += Hl[a, bb]
                b[idx[a]] += bl[a]
        
        try:
            dx = spsolve(H.tocsr(), -b)
        except:
            break
        state += dx
        if np.linalg.norm(dx) < 1e-6:
            break
    
    result = []
    for i, p in enumerate(poses):
        c = p.copy()
        c[0], c[1], c[5] = state[i*3:i*3+3]
        result.append(c)
    return result


def run_pgo(keyframes, ba_poses, icp_fitness=0.5, loop_dist=3.0, loop_gap=15):
    """Run PGO on BA-corrected poses with distance-based loops."""
    print("\n  ── Stage 2: Pose Graph Optimization ──")
    
    # Build odom edges from BA-corrected sequential poses
    odom_edges = []
    for i in range(len(ba_poses) - 1):
        Ti = pose6_to_T(ba_poses[i])
        Tj = pose6_to_T(ba_poses[i+1])
        rel = T_to_pose6(np.linalg.inv(Ti) @ Tj)
        odom_edges.append((i, i+1, rel))
    
    # Find loop candidates
    loop_cands = find_distance_loops(ba_poses, min_gap=loop_gap, dist_thresh=loop_dist)
    print(f"    {len(loop_cands)} distance-based loop candidates")
    
    # Verify with ICP
    loop_edges = []
    for j, i, d in loop_cands:
        # Build target submap around j
        tgt_list = []
        for off in range(-3, 4):
            idx = j + off
            if 0 <= idx < len(keyframes):
                Ti_j = pose6_to_T(ba_poses[j])
                Ti_k = pose6_to_T(ba_poses[idx])
                T_rel = np.linalg.inv(Ti_j) @ Ti_k
                tgt_list.append(transform_pts(keyframes[idx]['cloud'], T_rel))
        
        if not tgt_list:
            continue
        tgt = np.vstack(tgt_list)
        
        Ti_j = pose6_to_T(ba_poses[j])
        Ti_i = pose6_to_T(ba_poses[i])
        T_init = np.linalg.inv(Ti_j) @ Ti_i
        src_init = transform_pts(keyframes[i]['cloud'], T_init)
        
        T_icp, fitness = simple_icp(src_init, tgt)
        
        if fitness < icp_fitness:
            T_final = T_icp @ T_init
            rel = T_to_pose6(T_final)
            loop_edges.append((j, i, rel, 1.0))
            print(f"    ✅ Loop: {j} ↔ {i} (fitness={fitness:.3f}, dist={d:.1f}m)")
        else:
            print(f"    ✗ Reject: {j} ↔ {i} (fitness={fitness:.3f})")
    
    print(f"    {len(loop_edges)} loops verified")
    
    if loop_edges:
        final_poses = optimize_pgo(ba_poses, odom_edges, loop_edges)
        print("    PGO optimization complete")
    else:
        final_poses = ba_poses
        print("    No loops verified — using BA poses only")
    
    return final_poses, loop_edges


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Chained BA → PGO')
    parser.add_argument('bag_path')
    parser.add_argument('--cloud-topic', default='/registered_scan')
    parser.add_argument('--odom-topic', default='/state_estimation')
    parser.add_argument('--output', default='data/ba_pgo_result.png')
    parser.add_argument('--kf-dist', type=float, default=0.5)
    parser.add_argument('--max-kf', type=int, default=100)
    parser.add_argument('--voxel-size', type=float, default=1.5)
    parser.add_argument('--ground-weight', type=float, default=20.0)
    parser.add_argument('--loop-dist', type=float, default=3.0)
    parser.add_argument('--icp-fitness', type=float, default=0.5)
    args = parser.parse_args()
    
    print("=" * 60)
    print("  Offline BA → PGO Pipeline")
    print("=" * 60)
    
    # ── Read data ──
    print("\n1. Reading bag...")
    odom_times, odom_poses = [], []
    for stamp, msg in read_bag_messages(args.bag_path, args.odom_topic):
        odom_times.append(stamp)
        odom_poses.append(odom_to_pose6(msg))
    odom_times = np.array(odom_times)
    print(f"   {len(odom_poses)} odom messages")
    
    keyframes = []
    last_pos = None
    cloud_count = 0
    for stamp, msg in read_bag_messages(args.bag_path, args.cloud_topic):
        cloud_count += 1
        idx = min(np.searchsorted(odom_times, stamp), len(odom_poses)-1)
        pose = odom_poses[idx]
        
        if last_pos is not None and np.linalg.norm(pose[:3] - last_pos) < args.kf_dist:
            continue
        
        pts = cloud_to_numpy(msg)
        if len(pts) < 50:
            continue
        
        keyframes.append({'pose': pose.copy(), 'cloud': pts, 'time': stamp})
        last_pos = pose[:3].copy()
        if len(keyframes) >= args.max_kf:
            break
    
    print(f"   {cloud_count} clouds → {len(keyframes)} keyframes")
    
    if len(keyframes) < 5:
        print("   Not enough keyframes!")
        return
    
    # ── Run pipeline ──
    raw_poses = [kf['pose'].copy() for kf in keyframes]
    
    # Stage 1: BA
    ba_poses = run_ba(keyframes, voxel_size=args.voxel_size,
                      ground_weight=args.ground_weight)
    
    # Stage 2: PGO
    final_poses, loop_edges = run_pgo(keyframes, ba_poses,
                                       icp_fitness=args.icp_fitness,
                                       loop_dist=args.loop_dist)
    
    # ── Results ──
    raw_arr = np.array(raw_poses)
    ba_arr = np.array(ba_poses)
    fin_arr = np.array(final_poses)
    
    metrics = {}
    for name, arr in [('Raw', raw_arr), ('BA', ba_arr), ('BA+PGO', fin_arr)]:
        metrics[name] = {
            'gap': np.linalg.norm(arr[-1,:2] - arr[0,:2]),
            'z': abs(arr[-1,2] - arr[0,2]),
            'dist': np.sum(np.linalg.norm(np.diff(arr[:,:3], axis=0), axis=1))
        }
    
    print(f"\n{'='*60}")
    print(f"  {'Metric':<18} {'Raw':>10} {'BA':>10} {'BA+PGO':>10}")
    print(f"  {'-'*48}")
    print(f"  {'Loop gap XY (m)':<18} {metrics['Raw']['gap']:>8.2f}m {metrics['BA']['gap']:>8.2f}m {metrics['BA+PGO']['gap']:>8.2f}m")
    print(f"  {'Z drift (m)':<18} {metrics['Raw']['z']:>8.2f}m {metrics['BA']['z']:>8.2f}m {metrics['BA+PGO']['z']:>8.2f}m")
    print(f"  {'Distance (m)':<18} {metrics['Raw']['dist']:>8.2f}m {metrics['BA']['dist']:>8.2f}m {metrics['BA+PGO']['dist']:>8.2f}m")
    print(f"  {'Loops verified':<18} {'':>10} {'':>10} {len(loop_edges):>10}")
    print(f"{'='*60}")
    
    # ── Plot ──
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    ax = axes[0]
    ax.plot(raw_arr[:,0], raw_arr[:,1], 'r-', alpha=0.6, lw=1, label=f"Raw (gap={metrics['Raw']['gap']:.1f}m)")
    ax.plot(ba_arr[:,0], ba_arr[:,1], 'orange', alpha=0.6, lw=1, label=f"BA (gap={metrics['BA']['gap']:.1f}m)")
    ax.plot(fin_arr[:,0], fin_arr[:,1], 'b-', lw=1.5, label=f"BA+PGO (gap={metrics['BA+PGO']['gap']:.1f}m)")
    ax.plot(raw_arr[0,0], raw_arr[0,1], 'go', ms=10, label='Start')
    for li, ci, _, _ in loop_edges:
        ax.plot([fin_arr[li,0], fin_arr[ci,0]], [fin_arr[li,1], fin_arr[ci,1]],
                'g-', lw=2, alpha=0.5)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('XY Trajectory Comparison'); ax.legend(fontsize=7)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    
    ax = axes[1]
    t = np.arange(len(keyframes))
    ax.plot(t, raw_arr[:,2], 'r-', label=f"Raw (z={metrics['Raw']['z']:.2f}m)")
    ax.plot(t, ba_arr[:,2], 'orange', label=f"BA (z={metrics['BA']['z']:.2f}m)")
    ax.plot(t, fin_arr[:,2], 'b-', label=f"BA+PGO (z={metrics['BA+PGO']['z']:.2f}m)")
    ax.axhline(y=raw_arr[0,2], color='g', ls='--', alpha=0.5, label='Ground')
    ax.set_xlabel('Keyframe #'); ax.set_ylabel('Z (m)')
    ax.set_title('Z / Altitude'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    
    ax = axes[2]
    bars = list(metrics.keys())
    gaps = [metrics[k]['gap'] for k in bars]
    zs = [metrics[k]['z'] for k in bars]
    x = np.arange(len(bars))
    w = 0.35
    ax.bar(x - w/2, gaps, w, label='Loop gap XY', color='steelblue', alpha=0.8)
    ax.bar(x + w/2, zs, w, label='Z drift', color='coral', alpha=0.8)
    ax.set_xticks(x); ax.set_xticklabels(bars)
    ax.set_ylabel('Error (m)'); ax.set_title('Summary')
    ax.legend(); ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved: {args.output}")


if __name__ == '__main__':
    t0 = time.time()
    main()
    print(f"  Total time: {time.time()-t0:.1f}s")
