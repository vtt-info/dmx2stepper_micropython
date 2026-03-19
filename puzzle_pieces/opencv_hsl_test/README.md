# Triangle Detector v1.0

Detects 2 orange triangles using Raspberry Pi camera and outputs angle of sharpest edge.

## Output Format

```
FPS 15 | T1 025.0 | T2 007.3
```

- **FPS**: Frames per second (~7-15fps at 1296x972)
- **T1**: Left triangle angle (degrees)
- **T2**: Right triangle angle (degrees)

## Hardware

- Raspberry Pi with OV5647 camera module
- Resolution: 1296x972 (full sensor FOV)

## Quick Start

```bash
python3 triangle_detect_live.py
```

Press `Ctrl+C` to stop.

## Calibration

If angles don't read ~000.0 when triangles point up:

1. Edit `triangle_detect_live.py`
2. Find the offset lines:
   ```python
   if x_pos < 648:
       offset = 39  # T1 calibration
   else:
       offset = 45  # T2 calibration
   ```
3. Adjust until triangles pointing up read ~000.0

**Formula**: `new_offset = old_offset + (0 - current_reading)`

Example: Reading 355.0 (-5°) → `39 + 5 = 44`

## Files

| File | Description |
|------|-------------|
| `triangle_detect_live.py` | Main detector script |
| `config.py` | Configuration (HSL color, resolution) |
| `triangle_detect.py` | Single-shot version with debug images |

## Dependencies

```bash
pip install picamera2 numpy opencv-python
```

## Marker Version (v2)

Uses ArUco markers from the `markers/` folder instead of orange triangle color segmentation.

### Run live

```bash
python3 marker_detect_live.py
```

Output format is unchanged:

```text
FPS 15 | T1 025.0 | T2 007.3
```

- `T1`: marker on left half of frame
- `T2`: marker on right half of frame
- Marker IDs are auto-read from filenames in `markers/` (currently `17` and `23`)

### Single-shot debug

```bash
python3 marker_detect.py
```

Writes:
- `1_original.jpg`
- `2_hsv.jpg`
- `3_mask_raw.jpg`
- `4_mask_clean.jpg`
- `5_result.jpg`
