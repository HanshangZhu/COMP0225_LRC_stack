#!/usr/bin/env python3
"""
Pre-process a raw Go2 bag into a clean bag for Cartographer tuning.

Applies the same transforms as transform_everything.py
(15.1° pitch rotation, axis flips, IMU bias correction, body-box filter)
but offline — guaranteeing sorted, monotonic timestamps.

Usage:
  python3 preprocess_bag.py <input_bag_dir> [output_bag_dir]

Output bag contains:
  /utlidar/transformed_cloud    (PointCloud2, body frame)
  /utlidar/transformed_raw_imu  (Imu, body frame)
"""
import sys, os, yaml
import numpy as np

if not hasattr(np, "float"):
    np.float = float

from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Time as TimeMsg
import sensor_msgs_py.point_cloud2 as pc2
import tf_transformations
from transforms3d.quaternions import quat2mat


def stamp_to_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec

def ns_to_stamp(ns):
    t = TimeMsg()
    t.sec = int(ns // 10**9)
    t.nanosec = int(ns % 10**9)
    return t


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
            print(f"Loaded calibration from {p}")
            break
    return calib


def transform_imu(msg, calib, body2imu_rot):
    """Apply same transforms as transform_everything.py imu_callback."""
    theta = 15.1 / 180.0 * np.pi

    # Angular velocity: negate y,z then pitch-rotate
    x = msg.angular_velocity.x
    y = -msg.angular_velocity.y
    z = -msg.angular_velocity.z
    x2 = np.cos(theta) * x - np.sin(theta) * z
    y2 = y
    z2 = np.sin(theta) * x + np.cos(theta) * z
    x2 -= calib['ang_bias_x']
    y2 -= calib['ang_bias_y']
    z2 -= calib['ang_bias_z']
    x2 += calib['ang_z2x_proj'] * z2
    y2 += calib['ang_z2y_proj'] * z2

    # Linear acceleration: negate y,z then pitch-rotate
    ax = msg.linear_acceleration.x
    ay = -msg.linear_acceleration.y
    az = -msg.linear_acceleration.z
    ax2 = np.cos(theta) * ax - np.sin(theta) * az
    ay2 = ay
    az2 = np.sin(theta) * ax + np.cos(theta) * az

    # Orientation
    rot = [body2imu_rot[0], body2imu_rot[1], body2imu_rot[2], body2imu_rot[3]]
    ori = tf_transformations.quaternion_multiply(
        rot, [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    out = Imu()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = 'body'
    out.orientation.x = ori[0]
    out.orientation.y = ori[1]
    out.orientation.z = ori[2]
    out.orientation.w = ori[3]
    out.angular_velocity = Vector3(x=x2, y=y2, z=z2)
    out.linear_acceleration = Vector3(
        x=ax2 - calib['acc_bias_x'],
        y=ay2 - calib['acc_bias_y'],
        z=az2 - calib['acc_bias_z'])
    return out


def transform_cloud(msg, body2cloud_mat, cam_offset, filter_box):
    """Apply same transforms as transform_everything.py cloud_callback."""
    points = np.array(pc2.read_points_list(msg))
    points[:, 0:3] = points[:, 0:3] @ body2cloud_mat.T
    points[:, 2] -= cam_offset

    # Filter body box
    keep = ~(
        (points[:, 0] > filter_box['x_min']) & (points[:, 0] < filter_box['x_max']) &
        (points[:, 1] > filter_box['y_min']) & (points[:, 1] < filter_box['y_max']) &
        (points[:, 2] > filter_box['z_min']) & (points[:, 2] < filter_box['z_max'])
    )
    filtered = points[keep].tolist()
    for i in range(len(filtered)):
        filtered[i][4] = int(filtered[i][4])

    out = pc2.create_cloud(msg.header, msg.fields, filtered)
    out.header.stamp = msg.header.stamp
    out.header.frame_id = 'body'
    out.is_dense = msg.is_dense
    return out


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <input_bag_dir> [output_bag_dir]")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_bag = sys.argv[2] if len(sys.argv) > 2 else input_bag.rstrip('/') + '_clean'

    print(f"Input:  {input_bag}")
    print(f"Output: {output_bag}")

    # --- Setup transforms (matching transform_everything.py) ---
    calib = load_calib()
    cam_offset = 0.046825

    # body2cloud rotation
    quat_cloud = tf_transformations.quaternion_from_euler(0, 2.87820258505555555556, 0)
    body2cloud_mat = quat2mat(np.array([quat_cloud[3], quat_cloud[0], quat_cloud[1], quat_cloud[2]]))

    # body2imu rotation
    body2imu_rot = tf_transformations.quaternion_from_euler(0, 2.87820258505555555556, 3.14159265358)

    filter_box = {
        'x_min': -0.7, 'x_max': -0.1,
        'y_min': -0.3, 'y_max': 0.3,
        'z_min': -0.6 - cam_offset, 'z_max': 0 - cam_offset,
    }

    # --- Read all messages ---
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=input_bag, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print(f"Topics in bag: {topic_types}")

    messages = []  # (timestamp_ns, topic, serialized_bytes)
    imu_count = 0
    cloud_count = 0

    while reader.has_next():
        topic, data, t_ns = reader.read_next()

        if topic == '/utlidar/imu':
            msg = deserialize_message(data, Imu)
            transformed = transform_imu(msg, calib, body2imu_rot)
            raw_ns = stamp_to_ns(transformed.header.stamp)
            messages.append((raw_ns, '/utlidar/transformed_raw_imu', serialize_message(transformed), 'sensor_msgs/msg/Imu'))
            imu_count += 1

        elif topic == '/utlidar/cloud':
            msg = deserialize_message(data, PointCloud2)
            transformed = transform_cloud(msg, body2cloud_mat, cam_offset, filter_box)
            raw_ns = stamp_to_ns(transformed.header.stamp)
            messages.append((raw_ns, '/utlidar/transformed_cloud', serialize_message(transformed), 'sensor_msgs/msg/PointCloud2'))
            cloud_count += 1

    print(f"Read {imu_count} IMU + {cloud_count} cloud messages")

    # --- Sort by timestamp (the key step!) ---
    messages.sort(key=lambda x: x[0])
    print(f"Sorted {len(messages)} messages by timestamp")

    # Verify monotonic
    for i in range(1, len(messages)):
        if messages[i][0] < messages[i-1][0]:
            print(f"WARNING: non-monotonic at index {i}")

    # --- Write clean bag ---
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_bag, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    from rosbag2_py import TopicMetadata
    writer.create_topic(TopicMetadata(
        name='/utlidar/transformed_cloud',
        type='sensor_msgs/msg/PointCloud2',
        serialization_format='cdr'))
    writer.create_topic(TopicMetadata(
        name='/utlidar/transformed_raw_imu',
        type='sensor_msgs/msg/Imu',
        serialization_format='cdr'))

    for t_ns, topic, data, _ in messages:
        writer.write(topic, data, t_ns)

    del writer  # flush
    print(f"Done! Clean bag written to: {output_bag}")
    print(f"Replay with:")
    print(f"  ros2 bag play {output_bag} --rate 1.0")


if __name__ == '__main__':
    main()
