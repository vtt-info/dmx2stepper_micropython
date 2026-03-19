#!/usr/bin/env python3
import cv2
import numpy as np
from picamera2 import Picamera2
import config

picam2 = Picamera2()
config_raw = {"size": config.RESOLUTION}
if config.AWB_MODE is not None:
    config_raw["awb_mode"] = config.AWB_MODE
picam2.configure(picam2.create_preview_configuration(main=config_raw))
picam2.start()

frame = picam2.capture_array()
frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
if config.ROTATE_180:
    frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)
cv2.imwrite("1_original.jpg", frame_bgr)

hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
cv2.imwrite("2_hsv.jpg", hsv)

lower_orange = np.array(config.get_orange_hsv_range()[0])
upper_orange = np.array(config.get_orange_hsv_range()[1])
mask = cv2.inRange(hsv, lower_orange, upper_orange)
cv2.imwrite("3_mask_raw.jpg", mask)

kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), np.uint8)
mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
cv2.imwrite("4_mask_clean.jpg", mask_clean)

contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
result = frame_bgr.copy()
triangles = []

for cnt in contours:
    area = cv2.contourArea(cnt)
    if area < config.MIN_AREA:
        continue
    
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, config.APPROX_EPSILON * peri, True)
    
    if len(approx) == 3 and cv2.isContourConvex(approx):
        triangles.append(approx)
        cv2.drawContours(result, [approx], -1, (0, 255, 0), 3)

cv2.imwrite("5_result.jpg", result)

print(f"Found {len(triangles)} triangles")
for i, tri in enumerate(triangles):
    print(f"  Triangle {i+1}: {tri.tolist()}")

picam2.stop()
