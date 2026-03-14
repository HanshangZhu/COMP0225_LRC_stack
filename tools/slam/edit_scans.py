#!/usr/bin/env python3
"""Interactive scan pose editor.
- Shows projected wall points colored by scan index
- Click a region → highlights which scans contribute
- Arrow keys: nudge selected scans (dx, dy)
- R/T: rotate selected scans (dθ)
- S: save corrected poses
- Scroll or +/-: change selection radius
"""
import numpy as np
import json
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.ndimage import uniform_filter1d
from scipy.spatial import cKDTree
from plyfile import PlyData
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

MAP_PLY = "carto_3d_map.ply"
BAG = "cartographer_tuning_bag_2_clean"
POSES = "yaw_bias_corrected_poses.csv"
MIN_Z, MAX_Z, MIN_R2 = 0.15, 1.5, 0.09
NUDGE_M = 0.05       # meters per arrow key
NUDGE_DEG = 0.5      # degrees per R/T key

class ScanEditor:
    def __init__(self):
        # Load map
        ply = PlyData.read(MAP_PLY)
        mx, my, mz = np.array(ply['vertex']['x']), np.array(ply['vertex']['y']), np.array(ply['vertex']['z'])
        wm = (mz > MIN_Z) & (mz < MAX_Z)
        self.map_x = mx[wm]
        self.map_y = my[wm]

        # Load poses
        data = np.genfromtxt(POSES, delimiter=',', skip_header=1)
        self.px = uniform_filter1d(data[:,1], 30).copy()
        self.py = uniform_filter1d(data[:,2], 30).copy()
        self.yaw = np.radians(uniform_filter1d(data[:,3], 7)).copy()
        self.N = len(self.px)

        # Load scans (wall points in body frame)
        reader = SequentialReader()
        reader.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('',''))
        self.scans = []
        while reader.has_next():
            topic, d, ts = reader.read_next()
            if 'cloud' in topic.lower():
                msg = deserialize_message(d, PointCloud2)
                pts = list(pc2.read_points(msg, skip_nans=True))
                wall = [(p[0], p[1]) for p in pts
                        if p[2] >= MIN_Z and p[2] <= MAX_Z and (p[0]**2+p[1]**2) >= MIN_R2]
                self.scans.append(np.array(wall) if wall else np.zeros((0,2)))

        # Project all points
        self.project_all()

        # State
        self.selection_radius = 0.5
        self.selected_scans = set()
        self.click_center = None
        self.circle_patch = None
        self.history = []  # undo stack

    def project_all(self):
        """Project all scans to global frame."""
        pts_x, pts_y, pts_idx = [], [], []
        for i in range(min(self.N, len(self.scans))):
            w = self.scans[i]
            if len(w) == 0: continue
            c, s = np.cos(self.yaw[i]), np.sin(self.yaw[i])
            gx = c*w[:,0] - s*w[:,1] + self.px[i]
            gy = s*w[:,0] + c*w[:,1] + self.py[i]
            pts_x.extend(gx)
            pts_y.extend(gy)
            pts_idx.extend([i]*len(gx))
        self.pts_x = np.array(pts_x)
        self.pts_y = np.array(pts_y)
        self.pts_idx = np.array(pts_idx)
        # Build KD-tree for click selection
        self.pts_tree = cKDTree(np.column_stack([self.pts_x, self.pts_y]))

    def project_scans(self, scan_indices):
        """Re-project only specific scans."""
        for i in scan_indices:
            w = self.scans[i]
            if len(w) == 0: continue
            c, s = np.cos(self.yaw[i]), np.sin(self.yaw[i])
            gx = c*w[:,0] - s*w[:,1] + self.px[i]
            gy = s*w[:,0] + c*w[:,1] + self.py[i]
            mask = self.pts_idx == i
            idx = np.where(mask)[0]
            if len(idx) == len(gx):
                self.pts_x[idx] = gx
                self.pts_y[idx] = gy

    def draw(self):
        self.ax.clear()
        # Map walls
        self.ax.scatter(self.map_x, self.map_y, s=0.3, c='red', alpha=0.15)
        # Projected wall points
        colors = self.pts_idx / self.N
        sc = self.ax.scatter(self.pts_x, self.pts_y, s=0.2, c=colors,
                            cmap='rainbow', alpha=0.4, vmin=0, vmax=1)
        # Trajectory
        self.ax.plot(self.px, self.py, 'k-', lw=0.5, alpha=0.2)
        # Selection circle
        if self.click_center is not None:
            circ = Circle(self.click_center, self.selection_radius,
                         fill=False, ec='white', lw=2, ls='--')
            self.ax.add_patch(circ)
            # Highlight selected scan points
            for si in self.selected_scans:
                mask = self.pts_idx == si
                self.ax.scatter(self.pts_x[mask], self.pts_y[mask],
                              s=2, c='lime', alpha=0.8)
        # Info text
        info = f"Radius: {self.selection_radius:.2f}m | Selected: {len(self.selected_scans)} scans"
        if self.selected_scans:
            lo, hi = min(self.selected_scans), max(self.selected_scans)
            info += f" (#{lo}-{hi})"
        info += "\nClick=select | Arrows=nudge XY | R/T=rotate | Z=undo | S=save | +/-=radius"
        self.ax.set_title(info, fontsize=10)
        self.ax.set_aspect('equal')
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax: return
        if event.button == 1:  # left click
            cx, cy = event.xdata, event.ydata
            self.click_center = (cx, cy)
            # Find points within radius
            idx = self.pts_tree.query_ball_point([cx, cy], self.selection_radius)
            self.selected_scans = set(self.pts_idx[idx].astype(int))
            print(f"Selected {len(self.selected_scans)} scans near ({cx:.2f}, {cy:.2f})")
            self.draw()

    def on_key(self, event):
        if not self.selected_scans and event.key not in ('s', 'z', 'plus', 'minus', 'equal'):
            return

        scan_list = sorted(self.selected_scans)

        if event.key == 'right':
            self.save_undo(scan_list)
            for i in scan_list: self.px[i] += NUDGE_M
            self.project_scans(scan_list)
            self.draw()
        elif event.key == 'left':
            self.save_undo(scan_list)
            for i in scan_list: self.px[i] -= NUDGE_M
            self.project_scans(scan_list)
            self.draw()
        elif event.key == 'up':
            self.save_undo(scan_list)
            for i in scan_list: self.py[i] += NUDGE_M
            self.project_scans(scan_list)
            self.draw()
        elif event.key == 'down':
            self.save_undo(scan_list)
            for i in scan_list: self.py[i] -= NUDGE_M
            self.project_scans(scan_list)
            self.draw()
        elif event.key == 'r':  # rotate CCW
            self.save_undo(scan_list)
            for i in scan_list: self.yaw[i] += np.radians(NUDGE_DEG)
            self.project_scans(scan_list)
            self.draw()
        elif event.key == 't':  # rotate CW
            self.save_undo(scan_list)
            for i in scan_list: self.yaw[i] -= np.radians(NUDGE_DEG)
            self.project_scans(scan_list)
            self.draw()
        elif event.key == 'z':  # undo
            if self.history:
                scan_list, old_px, old_py, old_yaw = self.history.pop()
                for j, i in enumerate(scan_list):
                    self.px[i] = old_px[j]
                    self.py[i] = old_py[j]
                    self.yaw[i] = old_yaw[j]
                self.project_scans(scan_list)
                self.draw()
                print("Undo!")
        elif event.key == 's':  # save
            np.savetxt('manually_corrected_poses.csv',
                       np.column_stack([np.arange(self.N), self.px, self.py, np.degrees(self.yaw)]),
                       delimiter=',', header='scan,x,y,yaw_deg', comments='')
            print("Saved manually_corrected_poses.csv")
        elif event.key in ('+', '=', 'equal'):
            self.selection_radius *= 1.3
            if self.click_center: self.draw()
        elif event.key in ('-', 'minus'):
            self.selection_radius /= 1.3
            if self.click_center: self.draw()

    def save_undo(self, scan_list):
        self.history.append((
            scan_list[:],
            np.array([self.px[i] for i in scan_list]),
            np.array([self.py[i] for i in scan_list]),
            np.array([self.yaw[i] for i in scan_list]),
        ))

    def run(self):
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.draw()
        plt.show()

if __name__ == '__main__':
    print("Loading data...")
    editor = ScanEditor()
    print(f"Loaded {editor.N} poses, {len(editor.pts_x)} wall points")
    print("\nControls:")
    print("  Click          = select scans near cursor")
    print("  Arrow keys     = nudge selected scans ±5cm")
    print("  R / T          = rotate selected scans ±0.5°")
    print("  Z              = undo last edit")
    print("  S              = save corrected poses")
    print("  + / -          = change selection radius")
    editor.run()
