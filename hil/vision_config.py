"""Configuration for the host-side vision observer."""

from pathlib import Path


CAPTURE_DIR = Path(__file__).resolve().parent / "captures"

RESOLUTION = (640, 480)
ROTATE_180 = True

# Camera controls are optional. Keep them as None unless tuned on the target Pi.
AWB_MODE = None
EXPOSURE_COMPENSATION = None

# Orange HSL target used by the original prototype.
HUE = 30
SATURATION = 100
LIGHTNESS = 50
TOLERANCE = 10

MIN_AREA = 150
APPROX_EPSILON = 0.02
MORPH_KERNEL_SIZE = 5

LEFT_RIGHT_SPLIT_X = RESOLUTION[0] // 2
LEFT_OFFSET_DEG = 39.0
RIGHT_OFFSET_DEG = 45.0

FILTER_WINDOW = 7
JITTER_DEADBAND_DEG = 3.0
SETTLE_WINDOW_S = 0.5


def hsl_to_hsv(hue, saturation, lightness):
    """Convert HSL values into OpenCV HSV values."""
    saturation /= 100.0
    lightness /= 100.0

    if saturation == 0:
        value = lightness
        saturation = 0
    else:
        if lightness <= 0.5:
            value = lightness * (1 + saturation)
        else:
            value = lightness + saturation - lightness * saturation
        saturation = (value - lightness) / value if value > 0 else 0

    hsv_h = int(hue / 2)
    hsv_s = int(saturation * 255)
    hsv_v = int(value * 255)
    return (hsv_h, hsv_s, hsv_v)


def get_orange_hsv_range():
    """Return lower and upper HSV bounds for the orange target."""
    h_center, s_value, v_value = hsl_to_hsv(HUE, SATURATION, LIGHTNESS)
    h_low = max(0, h_center - TOLERANCE)
    h_high = min(179, h_center + TOLERANCE)
    lower = (h_low, int(s_value * 0.5), int(v_value * 0.5))
    upper = (h_high, 255, 255)
    return lower, upper
