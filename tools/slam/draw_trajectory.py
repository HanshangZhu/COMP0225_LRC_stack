#!/usr/bin/env python3
"""Interactive trajectory drawing on the Cartographer 3D map.
Left-click to add waypoints, right-click to undo, Enter/close to save."""
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from plyfile import PlyData
import json

MAP_PLY = "carto_3d_map.ply"
OUTPUT = "hand_drawn_trajectory.json"

def main():
    ply = PlyData.read(MAP_PLY)
    x = np.array(ply['vertex']['x'])
    y = np.array(ply['vertex']['y'])
    z = np.array(ply['vertex']['z'])
    
    # Wall points only for clearer visualization
    walls = (z > 0.2) & (z < 1.5)
    print(f"Loaded {walls.sum()} wall points from {MAP_PLY}")
    print(f"Map extent: X=[{x[walls].min():.1f}, {x[walls].max():.1f}], "
          f"Y=[{y[walls].min():.1f}, {y[walls].max():.1f}]")

    fig, ax = plt.subplots(figsize=(12, 12))
    ax.scatter(x[walls], y[walls], s=0.3, c='gray', alpha=0.4)
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_title('LEFT-CLICK: add waypoint | RIGHT-CLICK: undo | ENTER: save & close',
                 fontsize=12)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)

    waypoints = []
    line, = ax.plot([], [], 'r-o', lw=2, ms=8, alpha=0.8)
    start_mk, = ax.plot([], [], 'go', ms=15, zorder=10)

    def redraw():
        if waypoints:
            wx, wy = zip(*waypoints)
            line.set_data(wx, wy)
            start_mk.set_data([wx[0]], [wy[0]])
        else:
            line.set_data([], [])
            start_mk.set_data([], [])
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax: return
        if event.button == 1:
            waypoints.append((event.xdata, event.ydata))
            print(f"  + ({event.xdata:.2f}, {event.ydata:.2f})  [{len(waypoints)} pts]")
        elif event.button == 3 and waypoints:
            r = waypoints.pop()
            print(f"  - ({r[0]:.2f}, {r[1]:.2f})  [{len(waypoints)} pts]")
        redraw()

    def on_key(event):
        if event.key == 'enter':
            save()
            plt.close()

    def save():
        if not waypoints: return
        with open(OUTPUT, 'w') as f:
            json.dump({'waypoints': waypoints}, f, indent=2)
        print(f"\nSaved {len(waypoints)} waypoints to {OUTPUT}")

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.tight_layout()
    plt.show()
    save()  # also save on window close

if __name__ == '__main__':
    main()
