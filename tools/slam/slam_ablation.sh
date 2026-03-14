#!/bin/bash
# slam_ablation.sh — Automated SLAM frontend parameter sweep
#
# Replays data/hallway_run/ through Point-LIO with different configs,
# records trajectory, and compares metrics.
#
# Usage: ./slam_ablation.sh
#        ./slam_ablation.sh --skip-existing   # skip runs that already have results
set -euo pipefail

BAG_PATH="data/hallway_run/"
RESULTS_DIR="data/ablation_results"
TRANSFORM_SRC="src/autonomy_stack_go2/src/utilities/transform_sensors/transform_sensors/transform_everything.py"
POINTLIO_CFG="src/autonomy_stack_go2/src/slam/point_lio_unilidar/config/utlidar.yaml"
RESULTS_CSV="$RESULTS_DIR/results.csv"

mkdir -p "$RESULTS_DIR"

# ── Backup originals ──
cp "$TRANSFORM_SRC" "$TRANSFORM_SRC.bak"
cp "$POINTLIO_CFG" "$POINTLIO_CFG.bak"

cleanup() {
  echo "Restoring originals..."
  cp "$TRANSFORM_SRC.bak" "$TRANSFORM_SRC"
  cp "$POINTLIO_CFG.bak" "$POINTLIO_CFG"
  rm -f "$TRANSFORM_SRC.bak" "$POINTLIO_CFG.bak"
  killall -9 pointlio_mapping rviz2 transform_everything 2>/dev/null || true
}
trap cleanup EXIT

# ── Helper: set pitch angle in transform_everything.py ──
set_pitch() {
  local angle="$1"
  sed -i "s/theta = [0-9.]* \/ 180/theta = $angle \/ 180/" "$TRANSFORM_SRC"
  echo "    pitch = ${angle}°"
}

# ── Helper: set Point-LIO YAML params ──
# Usage: set_pointlio_param "param_name" "value"
set_pointlio_param() {
  local param="$1"
  local value="$2"
  # Match YAML line: spaces + param: old_value
  sed -i "s/\(${param}:\s*\).*/\1${value}/" "$POINTLIO_CFG"
}

# ── Helper: run one experiment ──
# Usage: run_experiment "run_name" 
run_experiment() {
  local name="$1"
  local odom_bag="$RESULTS_DIR/${name}"

  if [[ "${SKIP_EXISTING:-}" == "1" ]] && [[ -d "$odom_bag" ]]; then
    echo "  [SKIP] $name (already exists)"
    return 0
  fi

  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "  Running: $name"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

  # Clean previous
  rm -rf "$odom_bag"
  killall -9 pointlio_mapping rviz2 transform_everything 2>/dev/null || true
  sleep 2

  # Launch Point-LIO (also starts transform_everything + rviz)
  ros2 launch point_lio_unilidar mapping_utlidar.launch > /dev/null 2>&1 &
  local PLIO_PID=$!
  sleep 6

  # Record output
  ros2 bag record /state_estimation -o "$odom_bag" > /dev/null 2>&1 &
  local REC_PID=$!
  sleep 1

  # Play ONLY raw topics — transform_everything will re-transform with current pitch
  ros2 bag play "$BAG_PATH" --rate 1.0 \
    --topics /utlidar/cloud /utlidar/imu \
    > /dev/null 2>&1

  sleep 5
  kill $REC_PID 2>/dev/null; wait $REC_PID 2>/dev/null || true
  sleep 1
  kill $PLIO_PID 2>/dev/null; wait $PLIO_PID 2>/dev/null || true
  sleep 1

  # Extract metrics
  local stats
  stats=$(python3 plot_trajectory.py "$odom_bag" \
    --topics /state_estimation \
    --names "$name" \
    --output "$RESULTS_DIR/${name}.png" 2>&1)

  # Parse: "    Duration:  71.3s" etc — field after colon, strip units
  local duration dist gap zdrift
  duration=$(echo "$stats" | grep "Duration:" | sed 's/.*Duration:\s*//' | sed 's/s.*//')
  dist=$(echo "$stats" | grep "Distance:" | sed 's/.*Distance:\s*//' | sed 's/m.*//')
  gap=$(echo "$stats" | grep "Loop gap:" | sed 's/.*Loop gap:\s*//' | sed 's/m.*//')
  zdrift=$(echo "$stats" | grep "Z drift:" | sed 's/.*Z drift:\s*//' | sed 's/m.*//')

  echo "  → gap=${gap}m  z_drift=${zdrift}m  dist=${dist}m"
  echo "${name},${gap},${zdrift},${dist},${duration}" >> "$RESULTS_CSV"
}

# ── Main ──
[[ "${1:-}" == "--skip-existing" ]] && SKIP_EXISTING=1

echo "╔══════════════════════════════════════════════════════╗"
echo "║          SLAM Frontend Ablation Study                ║"
echo "║  Bag: $BAG_PATH                          ║"
echo "╚══════════════════════════════════════════════════════╝"

# Init CSV
echo "name,loop_gap_m,z_drift_m,distance_m,duration_s" > "$RESULTS_CSV"

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Group A: Hybrid baseline + pitch sweep
# Fixed: gravity_align=false, local covs (b=0.001, output=50/100, gyr_input=0.1)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
reset_to_hybrid() {
  cp "$POINTLIO_CFG.bak" "$POINTLIO_CFG"
  set_pointlio_param "gravity_align" "false"
}

# Run A1: Hybrid baseline (pitch=15.1, gravity_align=false + local covs)
reset_to_hybrid
set_pitch 15.1
run_experiment "A1_hybrid_p15.1"

# Run A2-A5: Pitch sweep
for pitch in 13.0 14.0 16.0 17.0; do
  reset_to_hybrid
  set_pitch "$pitch"
  run_experiment "A${pitch/./_}_pitch_p${pitch}"
done

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Group B: Bias cov sweep (pitch=15.1, gravity_align=false)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
for bcov in 0.00001 0.0001 0.01; do
  reset_to_hybrid
  set_pitch 15.1
  set_pointlio_param "b_acc_cov" "$bcov"
  set_pointlio_param "b_gyr_cov" "$bcov"
  echo "    b_acc/gyr_cov = $bcov"
  run_experiment "B_bcov_${bcov}"
done

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Group C: Output cov sweep (pitch=15.1, gravity_align=false, local bias)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
for acc_out in 50 200 500; do
  gyr_out=$((acc_out * 2))
  reset_to_hybrid
  set_pitch 15.1
  set_pointlio_param "acc_cov_output" "${acc_out}.0"
  set_pointlio_param "gyr_cov_output" "${gyr_out}.0"
  echo "    acc/gyr_cov_output = $acc_out/$gyr_out"
  run_experiment "C_outcov_${acc_out}_${gyr_out}"
done

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Final results
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║                     RESULTS                          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Results CSV: $RESULTS_CSV"
echo ""

# Print table
printf "%-30s %10s %10s %10s\n" "Run" "Gap(m)" "Z-drift" "Dist(m)"
printf "%-30s %10s %10s %10s\n" "---" "------" "-------" "-------"
while IFS=, read -r name gap zdrift dist dur; do
  [[ "$name" == "name" ]] && continue
  printf "%-30s %10s %10s %10s\n" "$name" "$gap" "$zdrift" "$dist"
done < "$RESULTS_CSV"

# Find best by combined metric (gap + zdrift)
echo ""
echo "Best runs by loop gap:"
sort -t, -k2 -n "$RESULTS_CSV" | head -4
echo ""
echo "Best runs by Z-drift:"
sort -t, -k3 -n "$RESULTS_CSV" | head -4

# Generate comparison plot
python3 -c "
import csv, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

rows = []
with open('$RESULTS_CSV') as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

if not rows:
    print('No results to plot')
    sys.exit(0)

names = [r['name'] for r in rows]
gaps = [float(r['loop_gap_m']) for r in rows]
zdrifts = [float(r['z_drift_m']) for r in rows]

fig, axes = plt.subplots(1, 2, figsize=(16, 6))

x = np.arange(len(names))
ax = axes[0]
ax.barh(x, gaps, color='steelblue', alpha=0.8)
ax.set_yticks(x); ax.set_yticklabels(names, fontsize=8)
ax.set_xlabel('Loop Gap XY (m)')
ax.set_title('Loop Closure Gap (lower = better)')
ax.invert_yaxis()
ax.grid(True, alpha=0.3, axis='x')

ax = axes[1]
ax.barh(x, zdrifts, color='coral', alpha=0.8)
ax.set_yticks(x); ax.set_yticklabels(names, fontsize=8)
ax.set_xlabel('Z Drift (m)')
ax.set_title('Z Drift (lower = better)')
ax.invert_yaxis()
ax.grid(True, alpha=0.3, axis='x')

plt.tight_layout()
plt.savefig('$RESULTS_DIR/comparison.png', dpi=150, bbox_inches='tight')
print('Comparison plot saved: $RESULTS_DIR/comparison.png')
"

echo ""
echo "Done! All results in $RESULTS_DIR/"
