#!/usr/bin/env python3
"""Viewer for a rosbag recorded by qt_example_full (Rosbag or Folder + rosbag format).

Shows the camera images, the occupancy map (GridMap with the class layer) and the segmented lidar points from above,
with the robot state, and plays the bag back in real time.

Usage (ROS 2 sourced):
    source /opt/ros/jazzy/setup.bash
    scripts/bag_viewer.py <folder>/rosbag_YYYYMMDD_hhmmss [--range 0.5 1.5]
--range: height range above the ground drawn bright in the point view (use the occupancy_map range of sensors.json).
Keys: space = play/pause, left/right = step one scan; the slider jumps.
"""
import argparse
import bisect
import json
import math
import sys
import time

import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from PyQt5 import QtCore, QtGui, QtWidgets

CAMERAS = ["cam1", "cam2", "cam3", "cam4"]
MAX_CAMERA_OFFSET = 0.06    # s: camera frame belongs to a scan when it is this close


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    return reader


class Bag:
    """Index of the message times per topic; messages are read on demand (seek + filtered read)."""

    def __init__(self, uri):
        self.uri = uri
        reader = open_reader(uri)
        self.types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        self.times = {}         # topic -> sorted list of ns
        self.small = {}         # topic -> list of (ns, msg) for the light topics, kept in memory
        light = {"/robot/odom", "/robot/gnss", "/tf", "/tf_static", "/occupancy_map/class_names"}
        while reader.has_next():
            topic, data, t = reader.read_next()
            self.times.setdefault(topic, []).append(t)
            if topic in light:
                self.small.setdefault(topic, []).append((t, deserialize_message(data, self.types[topic])))
        for topic in self.times:
            self.times[topic].sort()
        self.scans = self.times.get("/lidar/points", [])
        names = self.small.get("/occupancy_map/class_names")
        self.class_names = {int(k): v for k, v in json.loads(names[0][1].data).items()} if names else {}
        self.reader = open_reader(uri)

    def nearest(self, topic, t, max_offset_ns=None):
        times = self.times.get(topic, [])
        if not times:
            return None
        i = bisect.bisect_left(times, t)
        best = min((times[j] for j in (i - 1, i) if 0 <= j < len(times)), key=lambda x: abs(x - t))
        if max_offset_ns is not None and abs(best - t) > max_offset_ns:
            return None
        return best

    def read(self, wanted):
        """wanted: {topic: time ns}. Returns {topic: message}."""
        wanted = {k: v for k, v in wanted.items() if v is not None}
        if not wanted:
            return {}
        self.reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
        self.reader.seek(min(wanted.values()))
        found = {}
        last = max(wanted.values())
        while self.reader.has_next() and len(found) < len(wanted):
            topic, data, t = self.reader.read_next()
            if t > last:
                break
            if wanted.get(topic) == t and topic not in found:
                found[topic] = deserialize_message(data, self.types[topic])
        return found

    def latest_small(self, topic, t):
        items = self.small.get(topic, [])
        i = bisect.bisect_right([x[0] for x in items], t) - 1
        return items[i][1] if i >= 0 else None


def to_qimage(rgb):
    rgb = np.ascontiguousarray(rgb)
    h, w = rgb.shape[:2]
    return QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()


def grid_layer(msg, name):
    """GridMap layer as [row, col] array: row 0 = largest x (forward, up), col 0 = largest y (left)."""
    if name not in msg.layers:
        return None
    arr = msg.data[list(msg.layers).index(name)]
    rows = arr.layout.dim[1].size
    cols = arr.layout.dim[0].size
    return np.asarray(arr.data, dtype=np.float32).reshape(cols, rows).T


def class_colours(cloud):
    """Class id -> RGB, from the segmented point cloud (rgb field = class colour)."""
    data = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
    labels = data[:, 16]
    rgb = data[:, 12:16].copy().view(np.uint32).ravel()
    colours = {}
    for label in np.unique(labels):
        if label == 255:
            continue
        v = int(rgb[np.argmax(labels == label)])
        colours[int(label)] = ((v >> 16) & 255, (v >> 8) & 255, v & 255)
    return colours


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Viewer(QtWidgets.QWidget):
    def __init__(self, bag, height_range=(0.5, 1.5)):
        super().__init__()
        self.bag = bag
        self.height_range = height_range
        self.index = 0
        self.playing = False
        self.colours = {}
        self.setWindowTitle(f"Treebot rosbag viewer – {bag.uri}")
        self.resize(1500, 950)
        self.setStyleSheet("background:#15181d; color:#d8dde3;")

        self.cam_labels = []
        cam_row = QtWidgets.QHBoxLayout()
        for name in CAMERAS:
            box = QtWidgets.QVBoxLayout()
            title = QtWidgets.QLabel(name)
            title.setStyleSheet("color:#8a929c; font-weight:600;")
            label = QtWidgets.QLabel()
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumSize(200, 280)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
            box.addWidget(title)
            box.addWidget(label, 1)
            cam_row.addLayout(box, 1)
            self.cam_labels.append(label)

        self.map_label = QtWidgets.QLabel()
        self.map_label.setAlignment(QtCore.Qt.AlignCenter)
        self.map_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.points_label = QtWidgets.QLabel()
        self.points_label.setAlignment(QtCore.Qt.AlignCenter)
        self.points_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.info = QtWidgets.QLabel()
        self.info.setTextFormat(QtCore.Qt.RichText)
        self.info.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.info.setMinimumWidth(300)
        self.info.setStyleSheet("font-family:monospace; font-size:10pt;")

        def titled(text, widget):
            box = QtWidgets.QVBoxLayout()
            title = QtWidgets.QLabel(text)
            title.setStyleSheet("color:#8a929c; font-weight:600;")
            box.addWidget(title)
            box.addWidget(widget, 1)
            return box

        bottom = QtWidgets.QHBoxLayout()
        bottom.addLayout(titled("Occupancy map (GridMap, class layer) – forward ↑", self.map_label), 2)
        bottom.addLayout(titled(f"Segmented points from above ({height_range[0]:g}–{height_range[1]:g} m: bright, "
                                f"other heights: dim)", self.points_label), 2)
        bottom.addLayout(titled("Info", self.info), 1)

        self.play_button = QtWidgets.QPushButton("▶ Play")
        self.play_button.clicked.connect(self.toggle_play)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, max(0, len(bag.scans) - 1))
        self.slider.valueChanged.connect(self.show_frame)
        self.speed = QtWidgets.QComboBox()
        for s in ("0.25×", "0.5×", "1×", "2×"):
            self.speed.addItem(s)
        self.speed.setCurrentIndex(2)
        self.time_label = QtWidgets.QLabel()
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(self.play_button)
        controls.addWidget(self.slider, 1)
        controls.addWidget(self.speed)
        controls.addWidget(self.time_label)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(cam_row, 4)
        layout.addLayout(bottom, 5)
        layout.addLayout(controls)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.play_start = None
        self.show_frame(0)

    # ---- playback
    def toggle_play(self):
        self.playing = not self.playing
        self.play_button.setText("⏸ Pause" if self.playing else "▶ Play")
        if self.playing:
            if self.index >= len(self.bag.scans) - 1:
                self.slider.setValue(0)
            self.play_start = (time.monotonic(), self.bag.scans[self.index])
            self.timer.start(15)
        else:
            self.timer.stop()

    def tick(self):
        factor = [0.25, 0.5, 1.0, 2.0][self.speed.currentIndex()]
        wall0, bag0 = self.play_start
        target = bag0 + (time.monotonic() - wall0) * factor * 1e9
        i = bisect.bisect_right(self.bag.scans, target) - 1
        if i >= len(self.bag.scans) - 1:
            self.slider.setValue(len(self.bag.scans) - 1)
            self.toggle_play()
        elif i != self.index:
            self.slider.setValue(i)     # Skips frames when drawing is slower than the recording

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space:
            self.toggle_play()
        elif event.key() == QtCore.Qt.Key_Right:
            self.slider.setValue(min(self.index + 1, len(self.bag.scans) - 1))
        elif event.key() == QtCore.Qt.Key_Left:
            self.slider.setValue(max(self.index - 1, 0))
        else:
            super().keyPressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        QtCore.QTimer.singleShot(0, lambda: self.show_frame(self.index))

    # ---- drawing
    def show_frame(self, index):
        if not self.bag.scans:
            return
        tic = time.monotonic()
        self.index = index
        t = self.bag.scans[index]
        offset = int(MAX_CAMERA_OFFSET * 1e9)
        wanted = {f"/{c}/image/compressed": self.bag.nearest(f"/{c}/image/compressed", t, offset) for c in CAMERAS}
        wanted["/occupancy_map/grid_map"] = self.bag.nearest("/occupancy_map/grid_map", t, offset)
        wanted["/lidar/points_segmented"] = self.bag.nearest("/lidar/points_segmented", t, offset)
        msgs = self.bag.read(wanted)

        for label, cam in zip(self.cam_labels, CAMERAS):
            msg = msgs.get(f"/{cam}/image/compressed")
            if msg is None:
                label.setText("no frame")
                continue
            bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            pix = QtGui.QPixmap.fromImage(to_qimage(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
            label.setPixmap(pix.scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

        cloud = msgs.get("/lidar/points_segmented")
        if cloud is not None:
            self.colours.update(class_colours(cloud))
        grid = msgs.get("/occupancy_map/grid_map")
        self.draw_map(grid)
        self.draw_points(cloud, t)
        self.draw_info(t, grid, cloud, (time.monotonic() - tic) * 1000)

        rel = (t - self.bag.scans[0]) / 1e9
        total = (self.bag.scans[-1] - self.bag.scans[0]) / 1e9
        self.time_label.setText(f"{rel:6.2f} / {total:.2f} s · scan {index + 1}/{len(self.bag.scans)}")

    def draw_map(self, grid):
        label = self.map_label
        if grid is None:
            label.setText("no GridMap for this scan")
            return
        occ = grid_layer(grid, "occupancy")
        cls = grid_layer(grid, "class")
        img = np.zeros(occ.shape + (3,), np.uint8)
        img[:] = (17, 20, 25)                      # unknown
        img[occ == 0] = (55, 60, 68)               # free
        img[occ == 100] = (235, 235, 235)          # occupied, no class
        if cls is not None:
            for k, colour in self.colours.items():
                img[cls == k] = colour
        res = grid.info.resolution
        scale = max(1, int(min(label.width(), label.height()) / occ.shape[0]))
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        # Lidar at the map centre; rings every 5 m
        c = (img.shape[1] // 2, img.shape[0] // 2)
        for r in range(5, int(grid.info.length_x / 2) + 1, 5):
            cv2.circle(img, c, int(r / res * scale), (80, 86, 95), 1, cv2.LINE_AA)
            cv2.putText(img, f"{r} m", (c[0] + 3, c[1] - int(r / res * scale) - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (120, 128, 138), 1, cv2.LINE_AA)
        cv2.drawMarker(img, c, (60, 200, 90), cv2.MARKER_TRIANGLE_UP, 12, 2)
        label.setPixmap(QtGui.QPixmap.fromImage(to_qimage(img)).scaled(label.size(), QtCore.Qt.KeepAspectRatio))

    def draw_points(self, cloud, t):
        label = self.points_label
        if cloud is None:
            label.setText("no segmented scan")
            return
        data = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
        xyz = data[:, 0:12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
        rgbu = data[:, 12:16].copy().view(np.uint32).ravel()
        rgb = np.stack([(rgbu >> 16) & 255, (rgbu >> 8) & 255, rgbu & 255], axis=1).astype(np.uint8)
        tf = self.bag.latest_small("/tf", t)
        if tf is not None:
            tr = tf.transforms[0].transform
            R = quat_to_matrix(tr.rotation)
            pts = xyz @ R.T + np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        else:
            pts = xyz
        size = max(10, min(label.width(), label.height()))
        half_m = 15.0
        px = (size / 2 - pts[:, 1] / half_m * size / 2).astype(int)     # left = left
        py = (size / 2 - pts[:, 0] / half_m * size / 2).astype(int)     # forward = up
        ok = (px >= 0) & (px < size) & (py >= 0) & (py < size)
        in_range = (pts[:, 2] >= self.height_range[0]) & (pts[:, 2] <= self.height_range[1])
        img = np.zeros((size, size, 3), np.uint8)
        img[:] = (17, 20, 25)
        dim = ok & ~in_range
        img[py[dim], px[dim]] = (rgb[dim] * 0.35).astype(np.uint8)
        bright = ok & in_range
        order = np.argsort(pts[bright, 2])          # Highest on top
        img[py[bright][order], px[bright][order]] = rgb[bright][order]
        c = (size // 2, size // 2)
        for r in range(5, int(half_m) + 1, 5):
            cv2.circle(img, c, int(r / half_m * size / 2), (80, 86, 95), 1, cv2.LINE_AA)
        cv2.drawMarker(img, c, (60, 200, 90), cv2.MARKER_TRIANGLE_UP, 12, 2)
        label.setPixmap(QtGui.QPixmap.fromImage(to_qimage(img)))

    def draw_info(self, t, grid, cloud, ms):
        lines = []
        odom = self.bag.latest_small("/robot/odom", t)
        gnss = self.bag.latest_small("/robot/gnss", t)
        tf = self.bag.latest_small("/tf", t)
        if odom is not None:
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            lines += ["<b>Robot</b>", f"x {p.x:9.3f}  y {p.y:9.3f}", f"yaw {yaw:7.1f}°",
                      f"speed {odom.twist.twist.linear.x:5.2f} m/s  ω {odom.twist.twist.angular.z:5.2f}"]
        if gnss is not None:
            fix = "fix" if gnss.status.status >= 0 else "no fix"
            lines += [f"lat {gnss.latitude:.7f}", f"lon {gnss.longitude:.7f}  ({fix})"]
        if tf is not None:
            lines += ["", "<b>Ground plane</b>", f"lidar {tf.transforms[0].transform.translation.z:.2f} m above ground"]
        if grid is not None:
            occ = grid_layer(grid, "occupancy")
            lines += ["", "<b>Occupancy</b>", f"occupied {int(np.sum(occ == 100))} · free {int(np.sum(occ == 0))} cells"]
            cls = grid_layer(grid, "class")
            if cls is not None:
                vals, counts = np.unique(cls[~np.isnan(cls)], return_counts=True)
                for v, n in sorted(zip(vals, counts), key=lambda x: -x[1]):
                    k = int(v)
                    colour = "#%02x%02x%02x" % self.colours.get(k, (200, 200, 200))
                    lines.append(f"<span style='color:{colour}'>■</span> {self.bag.class_names.get(k, k)}: {n}")
        if cloud is not None:
            lines += ["", f"{cloud.width} lidar points"]
        lines += ["", f"<span style='color:#8a929c'>drawn in {ms:.0f} ms</span>"]
        self.info.setText("<br>".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag folder (with metadata.yaml and the .mcap file)")
    parser.add_argument("--range", nargs=2, type=float, default=(0.5, 1.5), metavar=("MIN", "MAX"),
                        help="height range above the ground drawn bright in the point view [m] (default 0.5 1.5)")
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv[:1])
    print("Indexing", args.bag, "…", flush=True)
    tic = time.monotonic()
    bag = Bag(args.bag)
    if not bag.scans:
        print("No /lidar/points in this bag", file=sys.stderr)
        return 1
    print(f"{len(bag.scans)} scans, {sum(len(v) for v in bag.times.values())} messages, indexed in {time.monotonic() - tic:.1f} s", flush=True)
    viewer = Viewer(bag, tuple(args.range))
    viewer.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
