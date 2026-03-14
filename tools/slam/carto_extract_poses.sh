#!/bin/bash
# carto_extract_poses.sh — Run Cartographer on clean bag, record TF, extract poses
# Usage: ./carto_extract_poses.sh

set -e

BAG_DIR="$HOME/COMP0225_LRC_stack/cartographer_tuning_bag_2_clean"
CARTO_CFG="$HOME/COMP0225_LRC_stack/src/go2_real_bringup/config/cartographer"
TF_BAG_DIR="/tmp/carto_tf_recording"
RATE=1.0

echo "=== Cartographer Pose Extraction ==="
echo "  Bag: $BAG_DIR"
echo "  Rate: $RATE"

# Kill stale
pkill -9 -f cartographer 2>/dev/null || true
pkill -9 -f transform_everything 2>/dev/null || true
sleep 1

# 1. Start Cartographer (clean bag has pre-transformed data)
echo ""
echo "1. Starting Cartographer..."
ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$CARTO_CFG" \
  -configuration_basename go2w_3d_mapping.lua \
  --ros-args \
  -r points2:=/utlidar/transformed_cloud \
  -r imu:=/utlidar/transformed_raw_imu \
  -p use_sim_time:=false &
CARTO_PID=$!
sleep 3

# 2. Record TF
echo "2. Recording TF..."
rm -rf "$TF_BAG_DIR"
ros2 bag record /tf /tf_static -o "$TF_BAG_DIR" &
REC_PID=$!
sleep 1

# 3. Play clean bag
echo "3. Playing clean bag at rate=$RATE..."
ros2 bag play "$BAG_DIR" --rate $RATE \
  --topics /utlidar/transformed_cloud /utlidar/transformed_raw_imu

echo "4. Bag complete. Waiting for Cartographer to finish processing..."
sleep 5

# Kill everything
kill $REC_PID 2>/dev/null || true
kill $CARTO_PID 2>/dev/null || true
sleep 2

echo ""
echo "5. TF recorded to: $TF_BAG_DIR"
echo "   Extract poses with: python3 extract_poses_from_tf_bag.py $TF_BAG_DIR"
echo "=== Done ==="
