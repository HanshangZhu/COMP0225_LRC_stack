#!/usr/bin/env python3
"""
extract_poses_from_tf_bag.py — Extract map→base_link poses from a recorded TF bag,
match to cloud timestamps, export for BALM2.

Usage:
  python3 extract_poses_from_tf_bag.py <tf_bag_dir> <cloud_bag_dir> <output_dir>
"""

import sys, os
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from transforms3d.quaternions import quat2mat


def main():
    if len(sys.argv) < 4:
        print("Usage: python3 extract_poses_from_tf_bag.py <tf_bag> <cloud_bag> <output_dir>")
        sys.exit(1)

    tf_bag = sys.argv[1]
    cloud_bag = sys.argv[2]
    output_dir = sys.argv[3]
    os.makedirs(output_dir, exist_ok=True)

    # 1. Read all TF transforms (map → base_link)
    print(f"Reading TF from {tf_bag}...")
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=tf_bag, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    tf_poses = []  # (timestamp_ns, 4x4 matrix)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in ['/tf', '/tf_static']:
            msg = deserialize_message(data, TFMessage)
            for tf in msg.transforms:
                if tf.header.frame_id == 'map' and tf.child_frame_id == 'base_link':
                    t = tf.transform.translation
                    q = tf.transform.rotation
                    R = quat2mat([q.w, q.x, q.y, q.z])
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[0, 3] = t.x
                    T[1, 3] = t.y
                    T[2, 3] = t.z
                    stamp_ns = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
                    tf_poses.append((stamp_ns, T))

    print(f"  {len(tf_poses)} map→base_link transforms")

    if not tf_poses:
        # Try odom → base_link as fallback
        print("  No map→base_link found, trying odom→base_link...")
        reader2 = SequentialReader()
        reader2.open(
            StorageOptions(uri=tf_bag, storage_id='sqlite3'),
            ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
        while reader2.has_next():
            topic, data, t_ns = reader2.read_next()
            if topic in ['/tf', '/tf_static']:
                msg = deserialize_message(data, TFMessage)
                for tf in msg.transforms:
                    if tf.child_frame_id == 'base_link':
                        t = tf.transform.translation
                        q = tf.transform.rotation
                        R = quat2mat([q.w, q.x, q.y, q.z])
                        T = np.eye(4)
                        T[:3, :3] = R
                        T[0, 3] = t.x
                        T[1, 3] = t.y
                        T[2, 3] = t.z
                        stamp_ns = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
                        tf_poses.append((stamp_ns, T))
        print(f"  {len(tf_poses)} *→base_link transforms")

    if not tf_poses:
        print("ERROR: No TF poses found!")
        sys.exit(1)

    tf_poses.sort(key=lambda x: x[0])

    # 2. Read cloud timestamps from original bag
    print(f"\nReading cloud timestamps from {cloud_bag}...")
    reader3 = SequentialReader()
    reader3.open(
        StorageOptions(uri=cloud_bag, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    cloud_stamps = []
    while reader3.has_next():
        topic, data, t_ns = reader3.read_next()
        if topic in ['/utlidar/transformed_cloud', '/utlidar/cloud']:
            msg = deserialize_message(data, PointCloud2)
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            cloud_stamps.append(stamp_ns)

    print(f"  {len(cloud_stamps)} cloud timestamps")

    # 3. Match cloud timestamps to nearest TF pose
    tf_stamps = np.array([t[0] for t in tf_poses])
    matched_poses = []
    for cs in cloud_stamps:
        idx = np.argmin(np.abs(tf_stamps - cs))
        dt_ms = abs(tf_stamps[idx] - cs) / 1e6
        if dt_ms < 200:  # within 200ms
            matched_poses.append(tf_poses[idx][1])
        else:
            print(f"  WARNING: closest TF is {dt_ms:.0f}ms away for cloud stamp")

    print(f"  Matched {len(matched_poses)} / {len(cloud_stamps)} clouds to TF poses")

    # 4. Write BALM2 format
    print(f"\nWriting to {output_dir}...")
    with open(os.path.join(output_dir, 'alidarPose.csv'), 'w') as f:
        for idx, T in enumerate(matched_poses):
            Tt = T.T.copy()
            Tt[3, 3] = float(idx)
            for r in range(4):
                f.write(','.join(str(Tt[r, c]) for c in range(4)))
                f.write('\n')

    print(f"  {len(matched_poses)} poses saved")

    # Summary
    if matched_poses:
        positions = np.array([T[:3, 3] for T in matched_poses])
        extent = positions.max(axis=0) - positions.min(axis=0)
        gap = np.linalg.norm(positions[-1, :2] - positions[0, :2])
        print(f"\n  Arena extent: {extent[0]:.1f}m × {extent[1]:.1f}m")
        print(f"  Loop gap: {gap:.2f}m")
        print(f"  Z range: {extent[2]:.2f}m")


if __name__ == '__main__':
    main()
