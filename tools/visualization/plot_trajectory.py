#!/usr/bin/env python3
"""Extract and plot trajectories from rosbag2 for SLAM comparison.

Usage:
  # Plot Point-LIO trajectory from recorded bag:
  python3 plot_trajectory.py data/hallway_run/ --topic /state_estimation

  # Compare two runs (live recording mode — subscribe to topics):
  python3 plot_trajectory.py --live --topics /state_estimation /corrected_odom --duration 90
"""
import sys
import os
import argparse
import numpy as np

def extract_from_bag(bag_path, topic):
    """Extract XY trajectory from a rosbag2 Odometry topic."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    xs, ys, zs, ts = [], [], [], []
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, Odometry)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
            zs.append(msg.pose.pose.position.z)
            ts.append(timestamp / 1e9)

    print(f"  {topic}: {len(xs)} poses extracted")
    return np.array(xs), np.array(ys), np.array(zs), np.array(ts)


def plot_comparison(trajectories, output_path="trajectory_comparison.png"):
    """Plot XY trajectories for comparison."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    colors = ['#e74c3c', '#2ecc71', '#3498db', '#9b59b6']

    # Left: XY trajectory
    ax = axes[0]
    for i, (name, xs, ys, zs, ts) in enumerate(trajectories):
        color = colors[i % len(colors)]
        # Subsample for cleaner plot
        step = max(1, len(xs) // 500)
        ax.plot(xs[::step], ys[::step], '-', color=color, alpha=0.7, linewidth=1.5, label=name)
        ax.plot(xs[0], ys[0], 'o', color=color, markersize=10, zorder=5)
        ax.plot(xs[-1], ys[-1], 's', color=color, markersize=10, zorder=5)

        # Annotate start/end gap
        gap = np.sqrt((xs[-1] - xs[0])**2 + (ys[-1] - ys[0])**2)
        ax.annotate(f'{name}\nloop gap: {gap:.2f}m',
                    xy=(xs[-1], ys[-1]), fontsize=8,
                    textcoords="offset points", xytext=(10, 10))

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Trajectory Comparison')
    ax.legend(loc='upper left')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Right: Z drift over time
    ax2 = axes[1]
    for i, (name, xs, ys, zs, ts) in enumerate(trajectories):
        color = colors[i % len(colors)]
        t_rel = ts - ts[0]
        step = max(1, len(t_rel) // 500)
        ax2.plot(t_rel[::step], zs[::step], '-', color=color, alpha=0.7, linewidth=1.5, label=name)

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('Altitude / Z-Drift Over Time')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n  Plot saved to: {output_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='SLAM trajectory comparison')
    parser.add_argument('bag_path', nargs='?', default=None, help='Path to rosbag2 directory')
    parser.add_argument('--topics', nargs='+', default=['/state_estimation'],
                        help='Odometry topics to extract')
    parser.add_argument('--names', nargs='+', default=None,
                        help='Display names for each topic')
    parser.add_argument('--output', default='trajectory_comparison.png',
                        help='Output plot path')
    args = parser.parse_args()

    if not args.bag_path:
        # Default to hallway_run
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.bag_path = os.path.join(script_dir, 'data', 'hallway_run')

    if not os.path.isdir(args.bag_path):
        print(f"ERROR: Bag not found: {args.bag_path}")
        sys.exit(1)

    names = args.names or args.topics
    if len(names) < len(args.topics):
        names.extend(args.topics[len(names):])

    print(f"Extracting from: {args.bag_path}")
    trajectories = []
    for topic, name in zip(args.topics, names):
        print(f"  Reading {topic}...")
        xs, ys, zs, ts = extract_from_bag(args.bag_path, topic)
        if len(xs) > 0:
            trajectories.append((name, xs, ys, zs, ts))
        else:
            print(f"  WARNING: No messages found on {topic}")

    if trajectories:
        plot_comparison(trajectories, args.output)

        # Print stats
        print("\n  === Trajectory Stats ===")
        for name, xs, ys, zs, ts in trajectories:
            duration = ts[-1] - ts[0]
            distance = np.sum(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2))
            loop_gap = np.sqrt((xs[-1] - xs[0])**2 + (ys[-1] - ys[0])**2)
            z_drift = abs(zs[-1] - zs[0])
            print(f"  {name}:")
            print(f"    Duration:  {duration:.1f}s")
            print(f"    Distance:  {distance:.1f}m")
            print(f"    Loop gap:  {loop_gap:.2f}m  (start→end XY distance)")
            print(f"    Z drift:   {z_drift:.3f}m")
    else:
        print("No trajectories extracted!")


if __name__ == '__main__':
    main()
