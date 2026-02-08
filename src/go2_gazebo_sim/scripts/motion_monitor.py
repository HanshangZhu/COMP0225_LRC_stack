#!/usr/bin/env python3
"""
Monitor robot motion and wall stops in Gazebo.  # module docstring
"""
import math  # math utilities
from typing import Optional  # typing helpers

import rclpy  # ROS2 Python API
from rclpy.node import Node  # ROS2 Node base class
from nav_msgs.msg import Odometry  # odometry message
from std_msgs.msg import Int8  # stop flag message


class MotionMonitor(Node):  # node definition
    def __init__(self):  # init
        super().__init__("motion_monitor")  # node name

        # Parameters  # section label
        self.declare_parameter("odom_topic", "/odom/ground_truth")  # odom topic
        self.declare_parameter("stop_topic", "/stop")  # wall stop topic
        self.declare_parameter("report_rate", 1.0)  # Hz
        self.declare_parameter("move_threshold", 0.05)  # meters

        self.odom_topic = self.get_parameter("odom_topic").value  # odom topic value
        self.stop_topic = self.get_parameter("stop_topic").value  # stop topic value
        self.report_rate = self.get_parameter("report_rate").value  # report rate value
        self.move_threshold = self.get_parameter("move_threshold").value  # movement threshold

        # State  # section label
        self.last_x: Optional[float] = None  # last x position
        self.last_y: Optional[float] = None  # last y position
        self.last_stop: Optional[int] = None  # last stop flag
        self.last_dist = 0.0  # last interval distance

        # Subscribers  # section label
        self.odom_sub = self.create_subscription(  # odom subscription
            Odometry, self.odom_topic, self.odom_callback, 10  # type/topic/callback
        )
        self.stop_sub = self.create_subscription(  # stop subscription
            Int8, self.stop_topic, self.stop_callback, 10  # type/topic/callback
        )

        # Timer  # section label
        self.timer = self.create_timer(1.0 / self.report_rate, self.report)  # periodic report

        self.get_logger().info("Motion monitor started")  # startup log

    def odom_callback(self, msg: Odometry):  # odom handler
        x = msg.pose.pose.position.x  # current x
        y = msg.pose.pose.position.y  # current y
        if self.last_x is not None and self.last_y is not None:  # if we have last
            self.last_dist = math.hypot(x - self.last_x, y - self.last_y)  # delta distance
        self.last_x = x  # store x
        self.last_y = y  # store y

    def stop_callback(self, msg: Int8):  # stop handler
        self.last_stop = int(msg.data)  # store stop flag

    def report(self):  # periodic report
        if self.last_x is None or self.last_y is None:  # no odom yet
            self.get_logger().info("Waiting for /odom...")  # log
            return  # skip
        moved = self.last_dist >= self.move_threshold  # movement check
        stop = self.last_stop if self.last_stop is not None else 0  # stop flag default
        status = "MOVING" if moved else "STILL"  # status string
        wall = "STOP=1" if stop == 1 else "STOP=0"  # wall status
        self.get_logger().info(  # log report
            f"pos=({self.last_x:.2f},{self.last_y:.2f}) dist={self.last_dist:.2f} {status} {wall}"
        )


def main(args=None):  # main entry
    rclpy.init(args=args)  # init ROS
    node = MotionMonitor()  # create node
    try:  # run loop
        rclpy.spin(node)  # spin
    except KeyboardInterrupt:  # allow ctrl-c
        pass  # ignore
    node.destroy_node()  # cleanup
    rclpy.shutdown()  # shutdown ROS


if __name__ == "__main__":  # script entry
    main()  # run main
