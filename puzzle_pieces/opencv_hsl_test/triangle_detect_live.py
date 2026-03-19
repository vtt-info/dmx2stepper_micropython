#!/usr/bin/env python3
"""
Triangle Detector v1.0
Detects 2 orange triangles and outputs angle of sharpest edge
"""
import cv2
import numpy as np
import time
from picamera2 import Picamera2
import config


def get_sharpest_edge_angle(approx):
    if len(approx) != 3:
        return None, None
    
    angles = []
    for i in range(3):
        p1 = approx[i][0]
        p2 = approx[(i + 1) % 3][0]
        p3 = approx[(i + 2) % 3][0]
        
        v1 = p1 - p2
        v2 = p3 - p2
        
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        angle = np.arccos(np.clip(cos_angle, -1, 1)) * 180 / np.pi
        angles.append((angle, i))
    
    sharpest_angle, sharpest_idx = min(angles, key=lambda x: x[0])
    
    corner = approx[sharpest_idx][0].astype(float)
    other_corners = [approx[(sharpest_idx + 1) % 3][0].astype(float),
                     approx[(sharpest_idx + 2) % 3][0].astype(float)]
    centroid = (other_corners[0] + other_corners[1]) / 2
    
    direction = centroid - corner
    pointing_angle = np.arctan2(direction[1], direction[0]) * 180 / np.pi
    pointing_angle = (pointing_angle + 90) % 360
    
    return pointing_angle, sharpest_angle


def process_frame(frame):
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if config.ROTATE_180:
        frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    lower_orange = np.array(config.get_orange_hsv_range()[0])
    upper_orange = np.array(config.get_orange_hsv_range()[1])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    triangles = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.MIN_AREA:
            continue
        
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, config.APPROX_EPSILON * peri, True)
        
        if len(approx) == 3 and cv2.isContourConvex(approx):
            triangles.append(approx)
    
    triangles.sort(key=lambda t: np.mean(t[:, 0, 0]), reverse=True)
    
    t1_angle = None
    t2_angle = None
    
    for tri in triangles:
        pointing_angle, _ = get_sharpest_edge_angle(tri)
        if pointing_angle is None:
            continue
        
        x_pos = np.mean(tri[:, 0, 0])
        if x_pos < 320:
            offset = 39
        else:
            offset = 45
        calibrated = (pointing_angle + offset) % 360
        
        if x_pos < 648 and t1_angle is None:
            t1_angle = round(calibrated, 1)
        elif x_pos >= 648 and t2_angle is None:
            t2_angle = round(calibrated, 1)
    
    return t1_angle, t2_angle


def main():
    picam2 = Picamera2()
    config_raw = {"size": config.RESOLUTION}
    if config.AWB_MODE is not None:
        config_raw["awb_mode"] = config.AWB_MODE
    picam2.configure(picam2.create_preview_configuration(main=config_raw))
    picam2.start()
    
    fps_counter = 0
    fps_start = time.time()
    fps = 0
    
    try:
        while True:
            frame = picam2.capture_array()
            
            t1_angle, t2_angle = process_frame(frame)
            
            fps_counter += 1
            if time.time() - fps_start >= 1.0:
                fps = fps_counter
                fps_counter = 0
                fps_start = time.time()
            
            t1_str = f"{t1_angle:05.1f}" if t1_angle is not None else "---"
            t2_str = f"{t2_angle:05.1f}" if t2_angle is not None else "---"
            output = f"FPS {fps:02d} | T1 {t1_str} | T2 {t2_str}"
            print(output)
            
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()
