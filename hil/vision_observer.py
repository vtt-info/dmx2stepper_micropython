#!/usr/bin/env python3
"""Observe two stepper-linked pointers with the Pi camera and log coarse angles."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import statistics
import sys
import time
from collections import deque
from pathlib import Path

import vision_config as config


def normalize_angle(angle_deg: float) -> float:
    return angle_deg % 360.0


class AxisTracker:
    """Median + deadband filter for one circular angle stream."""

    def __init__(self, window: int, deadband_deg: float):
        self._history = deque(maxlen=window)
        self._deadband_deg = deadband_deg
        self._last_unwrapped = None
        self._last_filtered = None

    def update(self, raw_angle_deg):
        if raw_angle_deg is None:
            return None, False

        unwrapped = self._unwrap(raw_angle_deg)
        self._history.append(unwrapped)

        filtered = statistics.median(self._history)
        if self._last_filtered is not None and abs(filtered - self._last_filtered) < self._deadband_deg:
            filtered = self._last_filtered

        self._last_unwrapped = unwrapped
        self._last_filtered = filtered
        return round(normalize_angle(filtered), 3), True

    def _unwrap(self, angle_deg: float) -> float:
        if self._last_unwrapped is None:
            return angle_deg
        candidates = (angle_deg - 360.0, angle_deg, angle_deg + 360.0)
        return min(candidates, key=lambda item: abs(item - self._last_unwrapped))


def get_sharpest_edge_angle(np, approx):
    """Return the pointing angle of a detected triangle contour."""
    if len(approx) != 3:
        return None, None

    angles = []
    for index in range(3):
        p1 = approx[index][0]
        p2 = approx[(index + 1) % 3][0]
        p3 = approx[(index + 2) % 3][0]

        v1 = p1 - p2
        v2 = p3 - p2
        cosine = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        angle = np.arccos(np.clip(cosine, -1, 1)) * 180 / np.pi
        angles.append((angle, index))

    _, sharpest_index = min(angles, key=lambda item: item[0])
    corner = approx[sharpest_index][0].astype(float)
    other_corners = [
        approx[(sharpest_index + 1) % 3][0].astype(float),
        approx[(sharpest_index + 2) % 3][0].astype(float),
    ]
    centroid = (other_corners[0] + other_corners[1]) / 2
    direction = centroid - corner
    pointing_angle = np.arctan2(direction[1], direction[0]) * 180 / np.pi
    return (pointing_angle + 90) % 360, None


def process_frame(cv2, np, frame):
    """Detect both triangle targets and return coarse angles for T1 and T2."""
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if config.ROTATE_180:
        frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower, upper = config.get_orange_hsv_range()
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

    kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    triangles = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < config.MIN_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, config.APPROX_EPSILON * perimeter, True)
        if len(approx) == 3 and cv2.isContourConvex(approx):
            x_pos = float(np.mean(approx[:, 0, 0]))
            triangles.append((x_pos, approx))

    observations = {"T1": None, "T2": None}
    for x_pos, triangle in sorted(triangles, key=lambda item: item[0]):
        pointing_angle, _ = get_sharpest_edge_angle(np, triangle)
        if pointing_angle is None:
            continue

        if x_pos < config.LEFT_RIGHT_SPLIT_X:
            if observations["T1"] is None:
                observations["T1"] = round(normalize_angle(pointing_angle + config.LEFT_OFFSET_DEG), 1)
        else:
            if observations["T2"] is None:
                observations["T2"] = round(normalize_angle(pointing_angle + config.RIGHT_OFFSET_DEG), 1)

    return observations


def timestamped_path(output_dir: Path, prefix: str) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return output_dir / f"{prefix}_{stamp}.csv"


def build_argument_parser():
    parser = argparse.ArgumentParser(description="Camera-based coarse angle observer")
    parser.add_argument("--output", help="Explicit output CSV path")
    parser.add_argument("--output-dir", default=str(config.CAPTURE_DIR), help="Directory for timestamped CSV output")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Optional capture duration, 0 = run until interrupted")
    parser.add_argument("--status-interval", type=float, default=1.0, help="Console status interval in seconds")
    parser.add_argument("--prefix", default="vision", help="Filename prefix when --output is not set")
    return parser


def configure_camera(picamera2_class):
    picam2 = picamera2_class()
    preview = picam2.create_preview_configuration(main={"size": config.RESOLUTION})
    picam2.configure(preview)
    picam2.start()

    controls = {}
    if config.AWB_MODE is not None:
        controls["AwbMode"] = config.AWB_MODE
    if config.EXPOSURE_COMPENSATION is not None:
        controls["ExposureValue"] = config.EXPOSURE_COMPENSATION
    if controls:
        picam2.set_controls(controls)

    return picam2


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    try:
        import cv2
        import numpy as np
        from picamera2 import Picamera2
    except ImportError as exc:
        print(f"[ERROR] Missing dependency: {exc}", file=sys.stderr)
        print("Install with: pip install picamera2 numpy opencv-python", file=sys.stderr)
        return 2

    output_path = Path(args.output) if args.output else timestamped_path(Path(args.output_dir), args.prefix)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    trackers = {
        "T1": AxisTracker(config.FILTER_WINDOW, config.JITTER_DEADBAND_DEG),
        "T2": AxisTracker(config.FILTER_WINDOW, config.JITTER_DEADBAND_DEG),
    }

    csv_file = None
    picam2 = None
    camera_started = False
    frame_index = 0
    status_start = time.monotonic()
    status_frames = 0
    capture_start = status_start

    try:
        picam2 = configure_camera(Picamera2)
        camera_started = True
        csv_file = output_path.open("w", newline="")
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "t_monotonic",
                "t_wall",
                "frame_index",
                "axis",
                "raw_angle_deg",
                "filtered_angle_deg",
                "visible",
            ],
        )
        writer.writeheader()

        while True:
            now_monotonic = time.monotonic()
            if args.duration_s > 0 and (now_monotonic - capture_start) >= args.duration_s:
                break

            frame = picam2.capture_array()
            observations = process_frame(cv2, np, frame)
            now_wall = dt.datetime.now().isoformat(timespec="milliseconds")

            frame_parts = []
            for axis in ("T1", "T2"):
                raw_angle = observations[axis]
                filtered_angle, visible = trackers[axis].update(raw_angle)
                writer.writerow(
                    {
                        "t_monotonic": f"{now_monotonic:.6f}",
                        "t_wall": now_wall,
                        "frame_index": frame_index,
                        "axis": axis,
                        "raw_angle_deg": "" if raw_angle is None else f"{raw_angle:.3f}",
                        "filtered_angle_deg": "" if filtered_angle is None else f"{filtered_angle:.3f}",
                        "visible": int(visible),
                    }
                )

                if visible:
                    frame_parts.append(f"{axis} raw={raw_angle:05.1f} filt={filtered_angle:05.1f}")
                else:
                    frame_parts.append(f"{axis} raw=--- filt=---")

            frame_index += 1
            status_frames += 1
            if frame_index % 10 == 0:
                csv_file.flush()

            elapsed = now_monotonic - status_start
            if elapsed >= args.status_interval:
                fps = status_frames / elapsed if elapsed > 0 else 0.0
                print(f"{now_wall} | frame {frame_index} | fps {fps:04.1f} | {' | '.join(frame_parts)}")
                status_start = now_monotonic
                status_frames = 0

    except KeyboardInterrupt:
        print("[INFO] Vision capture interrupted")
        return_code = 130
    except Exception as exc:
        print(f"[ERROR] Vision capture failed: {exc}", file=sys.stderr)
        sys.stderr.flush()
        sys.stdout.flush()
        if not camera_started:
            os._exit(1)
        return_code = 1
    else:
        return_code = 0
    finally:
        if picam2 is not None and camera_started:
            picam2.stop()
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

    if return_code == 0:
        print(f"[INFO] Vision capture written to {output_path}")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
