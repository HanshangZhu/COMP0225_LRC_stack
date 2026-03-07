# COMP0225_QRC_DEMO

This folder contains a ROS2-style bringup package that duplicates the key Gazebo and real-robot launch stacks plus their file dependencies.

Package path:
- `COMP0225_QRC_DEMO/src/comp0225_qrc_demo_bringup`

## Build (Standalone Workspace)

```bash
cd /home/hz/COMP0225_LRC_stack/COMP0225_QRC_DEMO
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/hz/COMP0225_LRC_stack/install/setup.bash
source install/setup.bash
```

## Launch (Gazebo Demo)

```bash
ros2 launch comp0225_qrc_demo_bringup gazebo_demo.launch.py
```

Go2W is the default Gazebo model. You can switch variants:

```bash
ros2 launch comp0225_qrc_demo_bringup gazebo_demo.launch.py robot_variant:=go2w
ros2 launch comp0225_qrc_demo_bringup gazebo_demo.launch.py robot_variant:=go2
```

Stable single-robot frontier run (headless, go2w, no FAST-LIO dependency):

```bash
ros2 launch comp0225_qrc_demo_bringup gazebo_demo.launch.py \
  robot_variant:=go2w gui:=false rviz:=false use_slam:=false
```

Optional: enable FAST-LIO in parallel while still using simulation ground truth as the relayed state source:

```bash
ros2 launch comp0225_qrc_demo_bringup gazebo_demo.launch.py \
  robot_variant:=go2w use_slam:=true slam_odom_topic:=/odom/ground_truth
```

## Launch (Real Robot Demo)

```bash
ros2 launch comp0225_qrc_demo_bringup real_robot_demo.launch.py
```

## Notes

- The launch files are copied into this package and patched to reference local copied configs/assets where possible.
- Runtime node executables still come from their original ROS2 packages (`local_planner`, `terrain_analysis`, `point_lio_unilidar`, `far_planner`, etc.), so source your main stack overlay first (`/home/hz/COMP0225_LRC_stack/install/setup.bash`).
