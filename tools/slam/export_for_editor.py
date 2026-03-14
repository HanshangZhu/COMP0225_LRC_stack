#!/usr/bin/env python3
"""Export scan data + map + poses to simple binary files for C++ editor."""
import numpy as np
import struct
from scipy.ndimage import uniform_filter1d
from plyfile import PlyData
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

MAP_PLY = "carto_3d_map.ply"
BAG = "cartographer_tuning_bag_2_clean"
POSES = "yaw_bias_corrected_poses.csv"
MIN_Z, MAX_Z, MIN_R2 = 0.15, 1.5, 0.09

# Map wall points
ply = PlyData.read(MAP_PLY)
mx, my, mz = np.array(ply['vertex']['x']), np.array(ply['vertex']['y']), np.array(ply['vertex']['z'])
wm = (mz > MIN_Z) & (mz < MAX_Z)
map_xy = np.column_stack([mx[wm], my[wm]]).astype(np.float32)
map_xy.tofile("editor_map.bin")
print(f"Map: {len(map_xy)} wall pts -> editor_map.bin")

# Poses
data = np.genfromtxt(POSES, delimiter=',', skip_header=1)
px = uniform_filter1d(data[:,1], 30).astype(np.float32)
py = uniform_filter1d(data[:,2], 30).astype(np.float32)
yaw = np.radians(uniform_filter1d(data[:,3], 7)).astype(np.float32)
N = len(px)
poses = np.column_stack([px, py, yaw])
poses.tofile("editor_poses.bin")
print(f"Poses: {N} -> editor_poses.bin")

# Scans (body-frame wall points)
reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('',''))
scans = []
while reader.has_next():
    topic, d, ts = reader.read_next()
    if 'cloud' in topic.lower():
        msg = deserialize_message(d, PointCloud2)
        pts = list(pc2.read_points(msg, skip_nans=True))
        wall = [(p[0], p[1]) for p in pts
                if p[2] >= MIN_Z and p[2] <= MAX_Z and (p[0]**2+p[1]**2) >= MIN_R2]
        scans.append(np.array(wall, dtype=np.float32) if wall else np.zeros((0,2), dtype=np.float32))

# Write scans: [N_scans][for each: n_pts, x0,y0, x1,y1, ...]
with open("editor_scans.bin", "wb") as f:
    f.write(struct.pack('i', len(scans)))
    total_pts = 0
    for s in scans:
        f.write(struct.pack('i', len(s)))
        if len(s) > 0:
            s.tofile(f)
            total_pts += len(s)
    print(f"Scans: {len(scans)}, total {total_pts} body-frame wall pts -> editor_scans.bin")
