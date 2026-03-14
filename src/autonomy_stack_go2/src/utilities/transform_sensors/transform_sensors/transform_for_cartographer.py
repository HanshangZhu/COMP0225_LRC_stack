#!/usr/bin/env python3
"""
Minimal QoS bridge + static TF publisher for Cartographer.

Instead of manually rotating point clouds and IMU data (error-prone),
this node:
  1. Bridges BEST_EFFORT → RELIABLE QoS for both cloud and IMU
  2. Publishes static TFs from body → sensor frames
  3. Forwards raw data with only timestamp monotonicity enforcement

Cartographer handles all frame transformations internally using TF.
This is Cartographer's INTENDED usage pattern.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# UTLidar mounting on Go2
PITCH_RAD = 15.1 / 180.0 * np.pi  # 15.1° forward tilt
BODY2CLOUD_EULER_Y = np.pi - PITCH_RAD  # ~2.878 rad (from original CMU code)


def quat_from_euler(roll, pitch, yaw):
    """Quaternion [x, y, z, w] from Euler angles (ZYX convention)."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


class CartographerBridge(Node):
    def __init__(self):
        super().__init__('carto_bridge')

        # ── Publishers (RELIABLE for Cartographer) ──
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50)
        self.imu_pub = self.create_publisher(Imu, '/carto/imu', reliable_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/carto/cloud', reliable_qos)

        # ── Subscribers (BEST_EFFORT to match Go2 DDS) ──
        self.imu_sub = self.create_subscription(
            Imu, '/utlidar/imu', self._imu_cb, qos_profile=qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self._cloud_cb, qos_profile=qos_profile_sensor_data)

        # ── Monotonic timestamp trackers ──
        self._last_imu_ns = 0
        self._last_cloud_ns = 0
        self._imu_drop_count = 0

        # ── Publish static TFs: body → sensor frames ──
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        self.get_logger().info('Cartographer QoS bridge ready')

    def _publish_static_tfs(self):
        """Publish body → utlidar_lidar and body → utlidar_imu static TFs.

        These are the EXACT transforms from the original CMU code:
        - body → utlidar_lidar: euler(0, π-15.1°, 0)
        - body → utlidar_imu:  euler(0, π-15.1°, π)
        """
        # body → LiDAR frame
        t_lidar = TransformStamped()
        t_lidar.header.stamp = self.get_clock().now().to_msg()
        t_lidar.header.frame_id = 'body'
        t_lidar.child_frame_id = 'utlidar_lidar'
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.046825  # cam_offset
        q = quat_from_euler(0, BODY2CLOUD_EULER_Y, 0)
        t_lidar.transform.rotation.x = q[0]
        t_lidar.transform.rotation.y = q[1]
        t_lidar.transform.rotation.z = q[2]
        t_lidar.transform.rotation.w = q[3]

        # body → IMU frame
        t_imu = TransformStamped()
        t_imu.header.stamp = self.get_clock().now().to_msg()
        t_imu.header.frame_id = 'body'
        t_imu.child_frame_id = 'utlidar_imu'
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.046825
        q = quat_from_euler(0, BODY2CLOUD_EULER_Y, np.pi)
        t_imu.transform.rotation.x = q[0]
        t_imu.transform.rotation.y = q[1]
        t_imu.transform.rotation.z = q[2]
        t_imu.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform([t_lidar, t_imu])
        self.get_logger().info(
            f'Published static TFs: body→utlidar_lidar, body→utlidar_imu '
            f'(pitch={15.1}°)')

    def _cloud_cb(self, msg: PointCloud2):
        """Forward cloud with monotonic timestamp + correct frame_id."""
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= self._last_cloud_ns:
            return
        self._last_cloud_ns = stamp_ns

        msg.header.frame_id = 'utlidar_lidar'
        self.cloud_pub.publish(msg)

    def _imu_cb(self, msg: Imu):
        """Forward IMU with monotonic timestamp + correct frame_id."""
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= self._last_imu_ns:
            self._imu_drop_count += 1
            if self._imu_drop_count % 50 == 1:
                self.get_logger().warn(
                    f'Dropped {self._imu_drop_count} out-of-order IMU msgs')
            return
        self._last_imu_ns = stamp_ns

        msg.header.frame_id = 'utlidar_imu'
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
