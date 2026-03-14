#!/usr/bin/env python3
"""Offline Scan-Context PGO — mirrors the C++ sc_pgo_node but reads bag directly.

Usage:
  python3 offline_pgo.py data/hallway_run/
  python3 offline_pgo.py data/hallway_run/ --odom-topic /state_estimation --cloud-topic /registered_scan
"""
import sys, os, argparse, time
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve

# ══════════════════════════════════════════════════════════════════
# 1. Bag Reader
# ══════════════════════════════════════════════════════════════════
def read_bag(bag_path, odom_topic, cloud_topic):
    """Read odom + cloud from bag, return sorted by timestamp."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))

    odom_msgs = []  # (timestamp_ns, Odometry)
    cloud_msgs = [] # (timestamp_ns, PointCloud2)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            odom_msgs.append((ts, msg))
        elif topic == cloud_topic:
            msg = deserialize_message(data, PointCloud2)
            cloud_msgs.append((ts, msg))

    print(f"  Read {len(odom_msgs)} odom, {len(cloud_msgs)} cloud msgs")
    return odom_msgs, cloud_msgs


def pc2_to_numpy(msg):
    """Convert PointCloud2 to Nx3 numpy array (XYZ only)."""
    import struct
    # Find xyz field offsets
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if 'x' not in fields: return np.zeros((0, 3))

    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    if n == 0: return np.zeros((0, 3))

    xs = np.frombuffer(msg.data, dtype=np.float32,
                       offset=fields['x'][0], count=n)
    # Need to handle stride
    points = np.zeros((n, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, point_step)
    for i, f in enumerate(['x', 'y', 'z']):
        off = fields[f][0]
        points[:, i] = np.frombuffer(raw[:, off:off+4].tobytes(),
                                      dtype=np.float32)

    # Filter NaN and far points
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    dist = np.linalg.norm(points, axis=1)
    points = points[(dist > 0.2) & (dist < 30.0)]
    return points


def odom_to_pose(msg):
    """Extract (x,y,z, roll,pitch,yaw) from Odometry."""
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    r = Rotation.from_quat([o.x, o.y, o.z, o.w])
    rpy = r.as_euler('xyz')
    return np.array([p.x, p.y, p.z, rpy[0], rpy[1], rpy[2]])


# ══════════════════════════════════════════════════════════════════
# 2. Scan Context
# ══════════════════════════════════════════════════════════════════
class ScanContext:
    def __init__(self, num_rings=20, num_sectors=60, max_radius=20.0,
                 dist_threshold=0.3, exclude_recent=25, candidates_num=10):
        self.NR = num_rings
        self.NS = num_sectors
        self.max_r = max_radius
        self.thresh = dist_threshold
        self.exclude = exclude_recent
        self.K = candidates_num
        self.descriptors = []
        self.ring_keys = []

    def make_descriptor(self, points):
        """Build ring-sector height descriptor from Nx3 points."""
        desc = np.zeros((self.NR, self.NS))
        if len(points) == 0:
            return desc
        xy = points[:, :2]
        r = np.linalg.norm(xy, axis=1)
        theta = np.arctan2(xy[:, 1], xy[:, 0]) + np.pi  # [0, 2pi]

        r_idx = np.clip((r / self.max_r * self.NR).astype(int), 0, self.NR - 1)
        s_idx = np.clip((theta / (2 * np.pi) * self.NS).astype(int), 0, self.NS - 1)

        for i in range(len(points)):
            ri, si = r_idx[i], s_idx[i]
            desc[ri, si] = max(desc[ri, si], points[i, 2])  # max height
        return desc

    def make_ring_key(self, desc):
        """Ring key = mean of each ring."""
        return desc.mean(axis=1)

    def add(self, points):
        desc = self.make_descriptor(points)
        self.descriptors.append(desc)
        self.ring_keys.append(self.make_ring_key(desc))

    def _sc_distance(self, a, b):
        """Column-shifted cosine distance between two SC descriptors."""
        min_dist = 1e9
        for shift in range(self.NS):
            b_shifted = np.roll(b, shift, axis=1)
            # Column-wise cosine similarity
            cos_sims = []
            for c in range(self.NS):
                ca, cb = a[:, c], b_shifted[:, c]
                na, nb = np.linalg.norm(ca), np.linalg.norm(cb)
                if na < 1e-6 or nb < 1e-6:
                    cos_sims.append(0.0)
                else:
                    cos_sims.append(np.dot(ca, cb) / (na * nb))
            dist = 1.0 - np.mean(cos_sims)
            min_dist = min(min_dist, dist)
        return min_dist

    def detect_loop(self, query_idx=None):
        """Find loop closure for the latest (or specified) keyframe."""
        if query_idx is None:
            query_idx = len(self.descriptors) - 1
        if query_idx < self.exclude + 2:
            return -1, -1, 1.0

        query_rk = self.ring_keys[query_idx]
        query_desc = self.descriptors[query_idx]

        # Ring key search (top-K candidates by L1 distance)
        search_range = query_idx - self.exclude
        if search_range <= 0:
            return -1, -1, 1.0

        rk_dists = []
        for i in range(search_range):
            d = np.linalg.norm(query_rk - self.ring_keys[i], ord=1)
            rk_dists.append((d, i))
        rk_dists.sort()
        candidates = [idx for _, idx in rk_dists[:self.K]]

        # Full SC distance for candidates
        best_idx = -1
        best_dist = self.thresh
        for idx in candidates:
            d = self._sc_distance(query_desc, self.descriptors[idx])
            if d < best_dist:
                best_dist = d
                best_idx = idx

        if best_idx >= 0:
            return best_idx, query_idx, best_dist
        return -1, -1, 1.0


# ══════════════════════════════════════════════════════════════════
# 3. Simple ICP (point-to-point using KDTree)
# ══════════════════════════════════════════════════════════════════
def simple_icp(source, target, max_iterations=50, tolerance=1e-5,
               max_corr_dist=2.0):
    """Simple point-to-point ICP. Returns (T_4x4, fitness_score)."""
    if len(source) < 10 or len(target) < 10:
        return np.eye(4), 1e6

    # Subsample for speed
    if len(source) > 2000:
        idx = np.random.choice(len(source), 2000, replace=False)
        src = source[idx].copy()
    else:
        src = source.copy()

    if len(target) > 5000:
        idx = np.random.choice(len(target), 5000, replace=False)
        tgt = target[idx]
    else:
        tgt = target

    tree = KDTree(tgt)
    T = np.eye(4)

    for it in range(max_iterations):
        # Find correspondences
        dists, indices = tree.query(src)
        mask = dists < max_corr_dist
        if mask.sum() < 10:
            break

        src_matched = src[mask]
        tgt_matched = tgt[indices[mask]]

        # Compute centroid
        src_c = src_matched.mean(axis=0)
        tgt_c = tgt_matched.mean(axis=0)

        # SVD for rotation
        H = (src_matched - src_c).T @ (tgt_matched - tgt_c)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = tgt_c - R @ src_c

        # Apply
        src = (R @ src.T).T + t
        dT = np.eye(4)
        dT[:3, :3] = R
        dT[:3, 3] = t
        T = dT @ T

        # Check convergence
        if np.linalg.norm(t) < tolerance and \
           np.abs(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))) < tolerance:
            break

    # Fitness score (mean inlier distance)
    dists, _ = tree.query(src)
    inlier_mask = dists < max_corr_dist
    if inlier_mask.sum() > 0:
        fitness = dists[inlier_mask].mean()
    else:
        fitness = 1e6
    return T, fitness


# ══════════════════════════════════════════════════════════════════
# 4. Pose Graph Optimization (simple Gauss-Newton on SE(2) for XY)
# ══════════════════════════════════════════════════════════════════
def pose_to_T(pose6):
    """pose6 = [x,y,z,r,p,y] -> 4x4 transform."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', pose6[3:6]).as_matrix()
    T[:3, 3] = pose6[:3]
    return T

def T_to_pose(T):
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
    return np.array([T[0, 3], T[1, 3], T[2, 3], rpy[0], rpy[1], rpy[2]])

def optimize_pose_graph(poses, odom_edges, loop_edges, n_iterations=20):
    """
    Simple pose graph optimization.
    poses: list of [x,y,z,r,p,y]
    odom_edges: list of (i, j, relative_pose6)
    loop_edges: list of (i, j, relative_pose6, weight)
    
    Uses iterative linearization on the full 6-DOF state.
    For simplicity, optimizes XY + yaw (3-DOF) which is what matters for 2D mapping.
    """
    n = len(poses)
    if n < 2 or len(loop_edges) == 0:
        return poses  # nothing to optimize

    # Work in 3-DOF: x, y, yaw
    state = np.zeros(n * 3)
    for i, p in enumerate(poses):
        state[i*3] = p[0]      # x
        state[i*3+1] = p[1]    # y
        state[i*3+2] = p[5]    # yaw

    odom_weight = 100.0  # high confidence in odom
    loop_weight = 1.0    # moderate confidence in loops

    for iteration in range(n_iterations):
        H = lil_matrix((n*3, n*3))
        b = np.zeros(n*3)

        # Prior on first pose
        for d in range(3):
            H[d, d] += 1e6
            # b[d] += 0 (anchor at origin)

        # Odometry edges
        for i, j, rel in odom_edges:
            # Expected relative: R_i^T @ (t_j - t_i)
            xi, yi, thi = state[i*3], state[i*3+1], state[i*3+2]
            xj, yj, thj = state[j*3], state[j*3+1], state[j*3+2]

            c, s = np.cos(thi), np.sin(thi)
            dx_w = xj - xi
            dy_w = yj - yi
            dx_local = c * dx_w + s * dy_w
            dy_local = -s * dx_w + c * dy_w
            dth = thj - thi
            # Wrap angle
            dth = (dth + np.pi) % (2*np.pi) - np.pi

            # Error
            ex = dx_local - rel[0]
            ey = dy_local - rel[1]
            eth = dth - rel[5]
            eth = (eth + np.pi) % (2*np.pi) - np.pi
            e = np.array([ex, ey, eth])

            # Jacobian (numerical for simplicity)
            J = np.zeros((3, 6))
            # d(error)/d(xi, yi, thi)
            J[0, 0] = -c;  J[0, 1] = -s;  J[0, 2] = -s*dx_w + c*dy_w
            J[1, 0] = s;   J[1, 1] = -c;  J[1, 2] = -c*dx_w - s*dy_w
            J[2, 2] = -1.0
            # d(error)/d(xj, yj, thj)
            J[0, 3] = c;   J[0, 4] = s
            J[1, 3] = -s;  J[1, 4] = c
            J[2, 5] = 1.0

            W = odom_weight * np.eye(3)
            idx = [i*3, i*3+1, i*3+2, j*3, j*3+1, j*3+2]
            JtW = J.T @ W
            Hlocal = JtW @ J
            blocal = JtW @ e
            for a in range(6):
                for bb in range(6):
                    H[idx[a], idx[bb]] += Hlocal[a, bb]
                b[idx[a]] += blocal[a]

        # Loop closure edges
        for i, j, rel, w in loop_edges:
            xi, yi, thi = state[i*3], state[i*3+1], state[i*3+2]
            xj, yj, thj = state[j*3], state[j*3+1], state[j*3+2]

            c, s = np.cos(thi), np.sin(thi)
            dx_w = xj - xi
            dy_w = yj - yi
            dx_local = c * dx_w + s * dy_w
            dy_local = -s * dx_w + c * dy_w
            dth = thj - thi
            dth = (dth + np.pi) % (2*np.pi) - np.pi

            ex = dx_local - rel[0]
            ey = dy_local - rel[1]
            eth = dth - rel[5]
            eth = (eth + np.pi) % (2*np.pi) - np.pi
            e = np.array([ex, ey, eth])

            J = np.zeros((3, 6))
            J[0, 0] = -c;  J[0, 1] = -s;  J[0, 2] = -s*dx_w + c*dy_w
            J[1, 0] = s;   J[1, 1] = -c;  J[1, 2] = -c*dx_w - s*dy_w
            J[2, 2] = -1.0
            J[0, 3] = c;   J[0, 4] = s
            J[1, 3] = -s;  J[1, 4] = c
            J[2, 5] = 1.0

            # Robust weight (Cauchy kernel)
            e_sq = np.dot(e, e)
            robust_w = w / (1.0 + e_sq)
            W = robust_w * np.eye(3)

            idx = [i*3, i*3+1, i*3+2, j*3, j*3+1, j*3+2]
            JtW = J.T @ W
            Hlocal = JtW @ J
            blocal = JtW @ e
            for a in range(6):
                for bb in range(6):
                    H[idx[a], idx[bb]] += Hlocal[a, bb]
                b[idx[a]] += blocal[a]

        # Solve
        H_csr = H.tocsr()
        try:
            dx = spsolve(H_csr, -b)
        except Exception:
            break
        state += dx

        if np.linalg.norm(dx) < 1e-6:
            break

    # Reconstruct full poses
    result = []
    for i, p in enumerate(poses):
        corrected = p.copy()
        corrected[0] = state[i*3]
        corrected[1] = state[i*3+1]
        corrected[5] = state[i*3+2]
        # Keep z, roll, pitch from original (not optimized in 2D)
        result.append(corrected)
    return result


# ══════════════════════════════════════════════════════════════════
# 5. Main Pipeline
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description='Offline SC-PGO')
    parser.add_argument('bag_path', help='Path to rosbag2')
    parser.add_argument('--odom-topic', default='/state_estimation')
    parser.add_argument('--cloud-topic', default='/registered_scan')
    parser.add_argument('--kf-gap', type=float, default=1.0, help='Keyframe distance gap (m)')
    parser.add_argument('--sc-threshold', type=float, default=0.3)
    parser.add_argument('--sc-exclude', type=int, default=25)
    parser.add_argument('--icp-fitness', type=float, default=0.5)
    parser.add_argument('--output', default='pgo_comparison.png')
    args = parser.parse_args()

    print("=" * 60)
    print("  Offline Scan-Context PGO")
    print(f"  Bag:         {args.bag_path}")
    print(f"  Odom topic:  {args.odom_topic}")
    print(f"  Cloud topic: {args.cloud_topic}")
    print("=" * 60)

    # ── Step 1: Read bag ──
    print("\n[1/6] Reading bag...")
    odom_msgs, cloud_msgs = read_bag(args.bag_path, args.odom_topic, args.cloud_topic)
    if not cloud_msgs:
        print("ERROR: No cloud messages found!")
        sys.exit(1)

    # Build odom lookup (sorted by timestamp)
    odom_ts = np.array([ts for ts, _ in odom_msgs])

    # ── Step 2: Pair cloud with nearest odom ──
    print("[2/6] Pairing clouds with nearest odom...")
    paired = []
    for cloud_ts_ns, cloud_msg in cloud_msgs:
        idx = np.searchsorted(odom_ts, cloud_ts_ns)
        idx = min(idx, len(odom_ts) - 1)
        # Check which neighbor is closer
        if idx > 0 and abs(odom_ts[idx-1] - cloud_ts_ns) < abs(odom_ts[idx] - cloud_ts_ns):
            idx = idx - 1
        paired.append((cloud_ts_ns, odom_msgs[idx][1], cloud_msg))
    print(f"  Paired {len(paired)} cloud-odom samples")

    # ── Step 3: Keyframe selection ──
    print("[3/6] Selecting keyframes...")
    sc = ScanContext(
        num_rings=20, num_sectors=60, max_radius=20.0,
        dist_threshold=args.sc_threshold,
        exclude_recent=args.sc_exclude, candidates_num=10)

    keyframes = []  # (pose6, points)
    kf_poses = []
    prev_pose = None

    for ts, odom_msg, cloud_msg in paired:
        pose = odom_to_pose(odom_msg)
        if prev_pose is not None:
            d = np.linalg.norm(pose[:3] - prev_pose[:3])
            if d < args.kf_gap:
                continue

        points = pc2_to_numpy(cloud_msg)
        if len(points) < 50:
            continue

        keyframes.append((pose, points))
        kf_poses.append(pose)
        sc.add(points)
        prev_pose = pose

    print(f"  Selected {len(keyframes)} keyframes from {len(paired)} clouds")

    # ── Step 4: Loop detection ──
    print("[4/6] Detecting loops (Scan Context)...")
    loops = []
    for i in range(args.sc_exclude + 2, len(keyframes)):
        loop_idx, query_idx, dist = sc.detect_loop(i)
        if loop_idx >= 0:
            loops.append((loop_idx, query_idx, dist))
            print(f"  Loop detected: {loop_idx} ↔ {query_idx} (SC dist={dist:.3f})")
    print(f"  Found {len(loops)} loop candidates")

    # ── Step 5: ICP verification ──
    print("[5/6] Verifying loops with ICP...")
    odom_edges = []
    loop_edges = []

    # Build odom edges
    for i in range(len(keyframes) - 1):
        Ti = pose_to_T(keyframes[i][0])
        Tj = pose_to_T(keyframes[i+1][0])
        T_rel = np.linalg.inv(Ti) @ Tj
        rel_pose = T_to_pose(T_rel)
        odom_edges.append((i, i+1, rel_pose))

    # Verify loops with ICP
    verified_loops = 0
    for loop_idx, curr_idx, sc_dist in loops:
        # Transform current cloud to loop frame for ICP
        src_pts = keyframes[curr_idx][1]
        # Build submap around loop candidate
        tgt_pts_list = []
        for offset in range(-5, 6):
            idx = loop_idx + offset
            if 0 <= idx < len(keyframes):
                # Transform to loop frame
                Ti = pose_to_T(keyframes[loop_idx][0])
                Tj = pose_to_T(keyframes[idx][0])
                T_rel = np.linalg.inv(Ti) @ Tj
                pts_transformed = (T_rel[:3, :3] @ keyframes[idx][1].T).T + T_rel[:3, 3]
                tgt_pts_list.append(pts_transformed)

        if not tgt_pts_list:
            continue
        tgt_pts = np.vstack(tgt_pts_list)

        # Initial guess: relative pose from odom
        Ti = pose_to_T(keyframes[loop_idx][0])
        Tc = pose_to_T(keyframes[curr_idx][0])
        T_init = np.linalg.inv(Ti) @ Tc
        src_init = (T_init[:3, :3] @ src_pts.T).T + T_init[:3, 3]

        T_icp, fitness = simple_icp(src_init, tgt_pts, max_iterations=50,
                                     max_corr_dist=2.0)

        if fitness < args.icp_fitness:
            T_final = T_icp @ T_init
            rel_pose = T_to_pose(T_final)
            loop_edges.append((loop_idx, curr_idx, rel_pose, 1.0))
            verified_loops += 1
            print(f"  ✅ Loop verified: {loop_idx} ↔ {curr_idx} (ICP fitness={fitness:.3f})")
        else:
            print(f"  ✗ Loop rejected: {loop_idx} ↔ {curr_idx} (ICP fitness={fitness:.3f})")

    print(f"  {verified_loops} loops verified out of {len(loops)} candidates")

    # ── Step 6: Pose Graph Optimization ──
    print("[6/6] Optimizing pose graph...")
    raw_poses = [kf[0].copy() for kf in keyframes]

    if verified_loops > 0:
        corrected_poses = optimize_pose_graph(raw_poses, odom_edges, loop_edges)
        print(f"  PGO complete with {verified_loops} loop constraints")
    else:
        corrected_poses = raw_poses
        print("  No loops — corrected = raw")

    # ── Results ──
    raw_arr = np.array(raw_poses)
    corr_arr = np.array(corrected_poses)

    raw_gap = np.linalg.norm(raw_arr[-1, :2] - raw_arr[0, :2])
    corr_gap = np.linalg.norm(corr_arr[-1, :2] - corr_arr[0, :2])
    raw_dist = np.sum(np.linalg.norm(np.diff(raw_arr[:, :2], axis=0), axis=1))
    corr_dist = np.sum(np.linalg.norm(np.diff(corr_arr[:, :2], axis=0), axis=1))

    print("\n" + "=" * 60)
    print("  Results")
    print("=" * 60)
    print(f"  {'':20s} {'Raw (Point-LIO)':>18s}  {'PGO Corrected':>18s}")
    print(f"  {'Keyframes':20s} {len(raw_poses):>18d}  {len(corrected_poses):>18d}")
    print(f"  {'Distance (m)':20s} {raw_dist:>18.1f}  {corr_dist:>18.1f}")
    print(f"  {'Loop gap XY (m)':20s} {raw_gap:>18.2f}  {corr_gap:>18.2f}")
    print(f"  {'Z drift (m)':20s} {abs(raw_arr[-1,2]-raw_arr[0,2]):>18.2f}  {abs(corr_arr[-1,2]-corr_arr[0,2]):>18.2f}")
    print(f"  {'Loops detected':20s} {len(loops):>18d}")
    print(f"  {'Loops verified':20s} {verified_loops:>18d}")
    print(f"  {'Improvement':20s} {'':>18s}  {(1-corr_gap/raw_gap)*100 if raw_gap > 0 else 0:>17.1f}%")

    # ── Plot ──
    print(f"\n  Plotting to {args.output}...")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(20, 6))

    # 1. Raw trajectory
    ax = axes[0]
    ax.plot(raw_arr[:, 0], raw_arr[:, 1], 'r-', linewidth=1.5, label='Point-LIO (raw)')
    ax.plot(raw_arr[0, 0], raw_arr[0, 1], 'go', markersize=10, label='Start')
    ax.plot(raw_arr[-1, 0], raw_arr[-1, 1], 'rs', markersize=10, label=f'End (gap={raw_gap:.2f}m)')
    ax.set_title(f'Point-LIO Raw\n(gap={raw_gap:.2f}m, zdrift={abs(raw_arr[-1,2]-raw_arr[0,2]):.2f}m)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

    # 2. Corrected trajectory
    ax = axes[1]
    ax.plot(corr_arr[:, 0], corr_arr[:, 1], 'b-', linewidth=1.5, label='SC-PGO Corrected')
    ax.plot(corr_arr[0, 0], corr_arr[0, 1], 'go', markersize=10, label='Start')
    ax.plot(corr_arr[-1, 0], corr_arr[-1, 1], 'bs', markersize=10, label=f'End (gap={corr_gap:.2f}m)')
    # Draw loop edges
    for li, ci, _, _ in loop_edges:
        ax.plot([corr_arr[li, 0], corr_arr[ci, 0]],
                [corr_arr[li, 1], corr_arr[ci, 1]], 'g-', linewidth=2, alpha=0.7)
    ax.set_title(f'SC-PGO Corrected\n(gap={corr_gap:.2f}m, {verified_loops} loops)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

    # 3. Overlay
    ax = axes[2]
    ax.plot(raw_arr[:, 0], raw_arr[:, 1], 'r-', linewidth=1, alpha=0.5, label='Raw')
    ax.plot(corr_arr[:, 0], corr_arr[:, 1], 'b-', linewidth=1.5, label='Corrected')
    for li, ci, _, _ in loop_edges:
        ax.plot([corr_arr[li, 0], corr_arr[ci, 0]],
                [corr_arr[li, 1], corr_arr[ci, 1]], 'g-', linewidth=2, alpha=0.5)
    ax.set_title('Overlay Comparison')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"  Saved: {args.output}")
    plt.close()


if __name__ == '__main__':
    t0 = time.time()
    main()
    print(f"\n  Total time: {time.time()-t0:.1f}s")
