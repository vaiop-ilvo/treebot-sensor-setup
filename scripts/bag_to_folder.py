#!/usr/bin/env python3
"""Writes a rosbag recorded by qt_example_full in the original folder format:

    <out>/
    ├── sensor_log_YYYYMMDD_hhmmss.csv
    ├── cam1/000000.png, …   cam2/ … camN/
    └── lidar/000000.ply, …

The CSV has the same columns and values as the folder recording (the rows come from /record/row), the images are the
recorded PNGs (lossless, written as they are) and the point clouds binary PLY files with x, y, z. The application's
Replay reads the result.

Usage (ROS 2 sourced):
    source /opt/ros/jazzy/setup.bash
    scripts/bag_to_folder.py <folder>/rosbag_YYYYMMDD_hhmmss [--out <folder>]
Default output: <bag>_folder next to the bag. The output folder must be empty or not exist (--force overwrites).
"""
import argparse
import datetime
import json
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROBOT_FIELDS = ["x", "y", "z", "roll", "pitch", "yaw", "lat", "lon", "gps_fix", "gps_lng_speed", "odom_lng_speed",
                "odom_ang_speed"]


def open_reader(uri, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


# Formatting as RecordWidget::logData (Qt)
def number(value):
    """QString::number(value, 'g', 17)"""
    return "%.17g" % value


def short_number(value):
    """QString::number(double): 'g' with 6 significant digits"""
    return "%g" % value


def iso_time(ts_us):
    """QDateTime::fromMSecsSinceEpoch(ts / 1000) in local time, yyyy-MM-ddTHH:mm:ss.zzz"""
    ms = int(ts_us) // 1000
    t = datetime.datetime.fromtimestamp(ms // 1000) + datetime.timedelta(milliseconds=ms % 1000)
    return t.strftime("%Y-%m-%dT%H:%M:%S.") + "%03d" % (ms % 1000)


def csv_field(value):
    if "," in value or '"' in value or "\n" in value:
        return '"' + value.replace('"', '""') + '"'
    return value


def write_ply(path, cloud):
    """Binary PLY with x, y, z (float) from a PointCloud2 with x, y, z float fields"""
    offsets = {f.name: f.offset for f in cloud.fields}
    data = bytes(cloud.data)
    step = cloud.point_step
    n = cloud.width * cloud.height
    if (offsets.get("x"), offsets.get("y"), offsets.get("z")) == (0, 4, 8) and step == 12:
        body = data[: n * 12]
    else:
        body = b"".join(data[i * step + offsets["x"]: i * step + offsets["x"] + 4]
                        + data[i * step + offsets["y"]: i * step + offsets["y"] + 4]
                        + data[i * step + offsets["z"]: i * step + offsets["z"] + 4] for i in range(n))
    with open(path, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
                b"property float x\nproperty float y\nproperty float z\nend_header\n" % n)
        f.write(body)


def csv_row(row, header, written_cloud, written_images, extra_keys):
    """The CSV line of a /record/row document, as RecordWidget::logData writes it"""
    fields = [row["log_time"]]
    lidar = row["lidar"]
    if lidar["valid"]:
        path = "lidar/%06d.ply" % row["row"] if written_cloud else ""
        fields += [lidar["ip"], iso_time(lidar["ts"]), str(int(lidar["frame_id"])), short_number(lidar["fps"]), csv_field(path)]
    else:
        fields += [lidar["ip"], "", "", "", ""]
    n_cameras = sum(1 for c in header if c.startswith("cam") and c.endswith("_ip"))
    for i in range(n_cameras):
        cam = row["cameras"][i] if i < len(row["cameras"]) else None
        if cam is not None and cam["valid"]:
            path = "cam%d/%06d.png" % (i + 1, row["row"]) if written_images[i] else ""
            fields += [cam["ip"], iso_time(cam["ts"]), str(int(cam["frame_id"])), short_number(cam["fps"]), csv_field(path)]
        else:
            fields += [cam["ip"] if cam is not None else "", "", "", "", ""]
    robot = row["robot"]
    if robot["valid"]:
        for name in ROBOT_FIELDS:
            fields.append(str(int(robot[name])) if name == "gps_fix" else number(robot[name]))
    else:
        fields += [""] * len(ROBOT_FIELDS)
    for key in extra_keys:
        raw = robot["redis"].get(key) if robot["valid"] else None
        fields.append(csv_field(raw if raw is not None else ""))
    return ",".join(fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag folder (with metadata.yaml and the .mcap file)")
    parser.add_argument("--out", help="output folder (default: <bag>_folder)")
    parser.add_argument("--force", action="store_true", help="write into a folder that is not empty")
    args = parser.parse_args()
    bag = os.path.normpath(args.bag)
    out = args.out or bag + "_folder"
    if os.path.isdir(out) and os.listdir(out) and not args.force:
        sys.exit(f"{out} is not empty (use --force to write into it)")

    # 1. The rows and the recording info
    info, rows = None, []
    reader = open_reader(bag, ["/record/info", "/record/row"])
    string_type = get_message("std_msgs/msg/String")
    while reader.has_next():
        topic, data, _ = reader.read_next()
        doc = json.loads(deserialize_message(data, string_type).data)
        if topic == "/record/info":
            info = doc
        else:
            rows.append(doc)
    if info is None or not rows:
        sys.exit(f"{bag} has no /record/info or /record/row: recorded before the folder conversion was supported")
    rows.sort(key=lambda r: r["row"])
    cameras = info["cameras"]
    header = info["csv_header"]
    extra_keys = [c[len("redis:"):] for c in header if c.startswith("redis:")]

    # Where each recorded message goes: (topic, stamp in us) -> (kind, row, camera)
    wanted = {}
    for r in rows:
        if r["lidar"].get("cloud_stamp"):
            wanted[("/lidar/points", int(r["lidar"]["cloud_stamp"]))] = ("lidar", r["row"], None)
        for i, cam in enumerate(r["cameras"]):
            if cam.get("image_stamp") and i < len(cameras):
                wanted[(f"/{cameras[i]}/image/compressed", int(cam["image_stamp"]))] = ("image", r["row"], i)

    # 2. The images and point clouds
    os.makedirs(os.path.join(out, "lidar"), exist_ok=True)
    for i in range(len(cameras)):
        os.makedirs(os.path.join(out, "cam%d" % (i + 1)), exist_ok=True)   # As the folder recording: by index
    written = set()
    topics = sorted({t for t, _ in wanted})
    types = {"/lidar/points": get_message("sensor_msgs/msg/PointCloud2")}
    reader = open_reader(bag, topics)
    image_type = get_message("sensor_msgs/msg/CompressedImage")
    while reader.has_next():
        topic, data, t = reader.read_next()
        target = wanted.get((topic, t // 1000))
        if target is None:
            continue
        kind, row, camera = target
        if kind == "lidar":
            write_ply(os.path.join(out, "lidar", "%06d.ply" % row), deserialize_message(data, types[topic]))
        else:
            image = deserialize_message(data, image_type)
            if "png" not in image.format:
                print(f"{topic}: format {image.format} is not PNG, skipped", file=sys.stderr)
                continue
            folder = os.path.join(out, "cam%d" % (camera + 1))
            with open(os.path.join(folder, "%06d.png" % row), "wb") as f:
                f.write(bytes(image.data))
        written.add((kind, row, camera))

    # 3. The CSV log
    csv_path = os.path.join(out, info["csv_name"])
    with open(csv_path, "w", newline="") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            images = [("image", r["row"], i) in written for i in range(len(r["cameras"]))]
            f.write(csv_row(r, header, ("lidar", r["row"], None) in written, images, extra_keys) + "\n")

    missing = len(wanted) - len(written)
    print(f"{len(rows)} rows, {sum(1 for w in written if w[0] == 'image')} images, "
          f"{sum(1 for w in written if w[0] == 'lidar')} point clouds -> {out}"
          + (f" ({missing} messages missing in the bag)" if missing else ""))


if __name__ == "__main__":
    main()
