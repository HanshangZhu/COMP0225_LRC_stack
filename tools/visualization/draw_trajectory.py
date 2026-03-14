#!/usr/bin/env python3
"""Interactive trajectory drawing on the Cartographer occupancy grid.
Click points to define waypoints, press Enter to save."""
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # interactive backend
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from plyfile import PlyData
import json, sys

MAP_PLY = "carto_3d_map.ply"
OUTPUT = "hand_drawn_trajectory.json"

def main():
    # Load wall points from PLY
    ply = PlyData.read(MAP_PLY)
    x = np.array(ply['vertex']['x'])
    y = np.array(ply['vertex']['y'])
    z = np.array(ply['vertex']['z'])
    walls = (z > 0.2) & (z < 1.5)
    
    print(f"Loaded {walls.sum()} wall points from {MAP_PLY}")
    print(f"Map extent: X=[{x[walls].min():.1f}, {x[walls].max():.1f}], "
          f"Y=[{y[walls].min():.1f}, {y[walls].max():.1f}]")
    
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.scatter(x[walls], y[walls], s=0.3, c='gray', alpha=0.4)
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_title('LEFT-CLICK to add waypoints\n'
                 'RIGHT-CLICK to undo last\n'
                 'Press ENTER or close window to save',
                 fontsize=13)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)
    
    waypoints = []
    line, = ax.plot([], [], 'r-o', lw=2, ms=8, alpha=0.8, picker=5)
    start_marker, = ax.plot([], [], 'go', ms=15, zorder=10, label='Start')
    
    def update_plot():
        if waypoints:
            wx = [w[0] for w in waypoints]
            wy = [w[1] for w in waypoints]
            line.set_data(wx, wy)
            start_marker.set_data([wx[0]], [wy[0]])
        else:
            line.set_data([], [])
            start_marker.set_data([], [])
        fig.canvas.draw_idle()
    
    def on_click(event):
        if event.inaxes != ax:
            return
        if event.button == 1:  # left click = add point
            waypoints.append((event.xdata, event.ydata))
            print(f"  Added ({event.xdata:.2f}, {event.ydata:.2f}) — {len(waypoints)} points")
        elif event.button == 3:  # right click = undo
            if waypoints:
                removed = waypoints.pop()
                print(f"  Removed ({removed[0]:.2f}, {removed[1]:.2f}) — {len(waypoints)} points")
        update_plot()
    
    def on_key(event):
        if event.key == 'enter':
            save_and_close()
    
    def save_and_close():
        if waypoints:
            data = {
                'waypoints': waypoints,
                'description': 'Hand-drawn trajectory waypoints on Cartographer map'
            }
            with open(OUTPUT, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"\nSaved {len(waypoints)} waypoints to {OUTPUT}")
        plt.close()
    
    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    plt.tight_layout()
    plt.show()
    
    # Save on close if not already saved
    if waypoints:
        data = {
            'waypoints': waypoints,
            'description': 'Hand-drawn trajectory waypoints on Cartographer map'
        }
        with open(OUTPUT, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved {len(waypoints)} waypoints to {OUTPUT}")

if __name__ == '__main__':
    main()
