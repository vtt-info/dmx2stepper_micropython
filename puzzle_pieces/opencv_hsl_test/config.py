"""
Triangle Detector v1.0

Hardware: Raspberry Pi with OV5647 camera
Purpose: Detect 2 orange triangles and output angle of sharpest edge

Output Format:
    FPS 15 | T1 025.0 | T2 007.3
    
    - FPS: frames per second
    - T1: left triangle angle (degrees)
    - T2: right triangle angle (degrees)

Calibration:
    - T1 uses offset 39
    - T2 uses offset 45
    - Adjust in triangle_detect_live.py if needed
    
    To recalibrate:
    1. Point triangles up (should read ~000.0)
    2. Note the reading (e.g., 355.0 = -5.0)
    3. Adjust offset: new_offset = old_offset + (0 - reading)
       Example: 355.0 -> 39 + 5 = 44

Resolution: 1296x972 (full sensor FOV)
Typical FPS: ~7-15 depending on lighting

Usage:
    python3 triangle_detect_live.py
    
    Press Ctrl+C to stop

Dependencies:
    - picamera2
    - numpy
    - opencv-python
"""

RESOLUTION = (1296, 972)

# Rotate image 180 degrees (for upside-down camera mounting)
ROTATE_180 = True

# Camera settings (may not be available on all Pi cameras)
# AWB modes: 0=Auto, 1=Tungsten, 2=Fluorescent, 3=Daylight, 4=Cloudy
AWB_MODE = None               # Set to int (0-4) to lock white balance, None for auto
EXPOSURE_COMPENSATION = None  # Set to int to adjust exposure, None for auto

# Orange color in HSL (H: 0-360, S: 0-100, L: 0-100)
HUE = 30           # 0-360 (color wheel)
SATURATION = 100  # 0-100 (intensity)
LIGHTNESS = 50    # 0-100 (brightness)
TOLERANCE = 10    # Hue tolerance range (+/-)

MIN_AREA = 500
APPROX_EPSILON = 0.02

MORPH_KERNEL_SIZE = 5

# Marker ID expectations for marker_detect_live.py
# Set to integer IDs (e.g., 17, 23) or None to disable side-ID filtering.
MARKER_LEFT_ID = 17
MARKER_RIGHT_ID = 23


def hsl_to_hsv(h, s, l):
    """Convert HSL (0-100 scale) to HSV (OpenCV scale)"""
    s /= 100.0
    l /= 100.0
    
    if s == 0:
        v = l
        s = 0
    else:
        if l <= 0.5:
            v = l * (1 + s)
        else:
            v = l + s - l * s
        s = (v - l) / v if v > 0 else 0
    
    hsv_h = int(h / 2)
    hsv_s = int(s * 255)
    hsv_v = int(v * 255)
    
    return (hsv_h, hsv_s, hsv_v)


def get_orange_hsv_range():
    """Returns HSV range for orange based on config"""
    h, s, l = HUE, SATURATION, LIGHTNESS
    
    h_center, s_val, v_val = hsl_to_hsv(h, s, l)
    h_low = max(0, h_center - TOLERANCE)
    h_high = min(179, h_center + TOLERANCE)
    
    lower = (h_low, int(s_val * 0.5), int(v_val * 0.5))
    upper = (h_high, 255, 255)
    
    return lower, upper
