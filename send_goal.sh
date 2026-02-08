#!/bin/bash
# Send a goal to the end of the L-corridor (9, 5)

source /opt/ros/humble/setup.bash
source /home/hz/cmu_exploration_ws/install/setup.bash

echo "Sending goal to (9.0, 5.0)..."
ros2 topic pub /way_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'odom'}, point: {x: 9.0, y: 5.0, z: 0.0}}" --once
