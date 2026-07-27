#!/usr/bin/env python3

"""
Convert compressed ROS 2 image topics into videos and depth images.

Example:
    python bag_to_video.py /path/to/bag

Output:
    output_videos/
    ├── color/
    │   └── camera_serial_color.mp4
    │
    ├── depth/
    │   └── camera_serial_depth/
    │       ├── 000000.png
    │       ├── 000001.png
    │       └── timestamps.csv
    │
    └── depth_video/
        └── camera_serial_depth.mp4
"""

from pathlib import Path
import csv

import cv2
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class VideoStream:
    """
    RGB compressed image -> MP4
    """

    def __init__(self, output_file: Path):
        self.output_file = output_file

        self.writer = None
        self.prev_stamp = None
        self.fps = 30.0
        self.frame_count = 0

    def write(self, frame, timestamp):
        if self.writer is None:
            if self.prev_stamp is None:
                self.prev_stamp = timestamp
                return

            dt = (timestamp - self.prev_stamp) / 1e9

            if dt > 0:
                self.fps = 1.0 / dt

            h, w = frame.shape[:2]

            self.writer = cv2.VideoWriter(
                str(self.output_file),
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps,
                (w, h),
            )

        self.prev_stamp = timestamp

        self.writer.write(frame)

        self.frame_count += 1

    def close(self):
        if self.writer:
            self.writer.release()


class DepthStream:
    """
    Depth compressed image -> uint16 PNGs + visualization MP4
    """

    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.video_file = output_dir.parent / f"{output_dir.name}.mp4"

        self.writer = None
        self.prev_stamp = None
        self.fps = 30.0

        self.frame_count = 0

    def write(self, depth, timestamp):
        # initialize visualization video
        if self.writer is None:
            if self.prev_stamp is None:
                self.prev_stamp = timestamp

                self.frame_count += 1
                return

            dt = (timestamp - self.prev_stamp) / 1e9

            if dt > 0:
                self.fps = 1.0 / dt

            h, w = depth.shape

            self.writer = cv2.VideoWriter(
                str(self.video_file),
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps,
                (w, h),
                False,
            )

        self.prev_stamp = timestamp

        # depth visualization
        depth_vis = cv2.normalize(
            depth,
            None,
            0,
            255,
            cv2.NORM_MINMAX,
        ).astype(np.uint8)

        self.writer.write(depth_vis)

        self.frame_count += 1

    def close(self):
        if self.writer:
            self.writer.release()


def decode_compressed_depth(msg):
    """
    Decode sensor_msgs/msg/CompressedImage with format:
    16UC1; compressedDepth
    """

    # The first 12 bytes contain depth quantization parameters
    # (float32 depth quantization values)
    raw = np.frombuffer(msg.data[12:], dtype=np.uint8)

    depth = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED)

    return depth


def main():
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("bag", help="Path to rosbag")

    parser.add_argument("--output", default="output_videos", help="Output directory")

    args = parser.parse_args()

    output_dir = Path(args.output)

    color_dir = output_dir / "color"
    depth_dir = output_dir / "depth"

    color_dir.mkdir(parents=True, exist_ok=True)

    depth_dir.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(
        uri=args.bag,
        storage_id="sqlite3",
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()

    reader.open(storage_options, converter_options)

    #
    # Find image topics
    #

    topics = reader.get_all_topics_and_types()

    topic_types = {t.name: t.type for t in topics}

    image_topics = []

    for topic, msg_type in topic_types.items():
        if msg_type == "sensor_msgs/msg/CompressedImage":
            image_topics.append(topic)

    print("Found topics:")

    for t in image_topics:
        print(" ", t)

    streams = {}

    #
    # Read messages
    #

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic not in image_topics:
            continue

        if topic not in streams:
            name = topic.strip("/").replace("/", "_")

            if "depth" in topic.lower():
                print("Depth:", topic)

                streams[topic] = DepthStream(depth_dir / name)

            else:
                print("Color:", topic)

                streams[topic] = VideoStream(color_dir / f"{name}.mp4")

        msg_type = get_message(topic_types[topic])

        msg = deserialize_message(data, msg_type)

        if "depth" in topic.lower():
            depth = decode_compressed_depth(msg)

            if depth is None:
                continue

            streams[topic].write(depth, timestamp)

        else:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            streams[topic].write(frame, timestamp)

    #
    # Cleanup
    #

    for stream in streams.values():
        stream.close()

    print("Done.")


if __name__ == "__main__":
    main()
