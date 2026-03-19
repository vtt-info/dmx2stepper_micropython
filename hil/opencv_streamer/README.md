# opencv_streamer

Captures a Raspberry Pi camera feed, detects the brightest shape in the scene,
overlays its outline and center X coordinate on the stream, serves it as MJPEG
over HTTP, and pushes the X coordinate to any number of connected Python scripts
over TCP.

---

## Requirements

- Raspberry Pi with a CSI camera (tested: imx477 HQ Camera)
- Python 3.10+
- `picamera2`, `opencv-python`, `flask`, `numpy`

```bash
pip install flask numpy opencv-python
# picamera2 is pre-installed on Raspberry Pi OS
```

---

## Running

```bash
python3 streamer.py
```

Then open a browser at `http://<pi-ip>:8080/` to see the annotated live feed.
Stop with `Ctrl+C`.

---

## How it works

### Camera capture

The camera is opened via **picamera2** (the native libcamera Python interface for
Pi CSI cameras). It captures at 640×480, 25 fps, using the 2028×1080 sensor mode
(full-width field of view, 16:9). Auto-exposure is disabled and exposure is set
manually via `EXPOSURE_TIME_US`.

Each frame is read as a numpy array, converted from RGB to BGR for OpenCV, and
passed to the detector.

### Bright blob detection

The detector does not try to match a specific shape. Instead it finds the
**largest bright region** in the frame:

1. Convert frame to grayscale
2. Apply a binary threshold: every pixel brighter than `BRIGHT_THRESHOLD`
   becomes white, everything else black
3. Find external contours of the white regions
4. Pick the contour with the largest area that is at least `MIN_AREA` pixels
5. Compute the **centroid** of that contour — this becomes the reported X

This approach is robust to blur, partial occlusion, and irregular shapes, as
long as the target is the brightest thing in the frame.

### Overlay

On a detection the frame is annotated with:
- Green outline of the detected blob
- Filled green dot at the centroid
- Vertical green line at the center X across the full frame height
- `X: <value>` text label

### MJPEG stream (port 8080)

Flask serves two routes:

| Route | Description |
|-------|-------------|
| `GET /` | HTML page with the stream embedded |
| `GET /stream` | Raw MJPEG multipart stream |

The stream can be embedded in any HTML page with:
```html
<img src="http://<pi-ip>:8080/stream">
```

### TCP coordinate server (port 9999)

Every time the detected X **changes**, all connected TCP clients receive a
newline-terminated message:

- `"348\n"` — detected center X in pixels (0 = left edge, 639 = right edge)
- `"null\n"` — no bright blob found in the current frame

Messages are only sent on change, not on every frame. Multiple clients can
connect simultaneously.

---

## Tunable constants

All tunables are at the top of `streamer.py`:

| Constant | Default | Description |
|----------|---------|-------------|
| `BRIGHT_THRESHOLD` | `100` | Grayscale cutoff (0–255). Pixels above this are considered part of the bright shape. Raise to ignore dim areas, lower if the target is not detected. |
| `MIN_AREA` | `50` | Minimum blob size in pixels. Filters out tiny noise specks. |
| `EXPOSURE_TIME_US` | `50000` | Manual shutter time in microseconds. Range: 1–40000 µs at 25 fps (hardware max 66666 µs, but values above 40000 µs may cause frame drops). 1000 = bright scene, 10000 = dim room, 40000 = low light. |
| `JPEG_QUALITY` | `80` | MJPEG stream quality (1–100). Lower = less bandwidth. |
| `CAMERA_INDEX` | `0` | Which CSI camera to use (0 or 1 if two cameras are connected). |
| `CAMERA_WIDTH/HEIGHT` | `640×480` | Output resolution. |
| `CAMERA_FPS` | `25` | Target frame rate. |
| `HTTP_PORT` | `8080` | Port for the MJPEG web stream. |
| `TCP_PORT` | `9999` | Port for the X coordinate server. |

---

## Reading the X coordinate from another Python script

Connect to TCP port 9999. The server pushes `"{x}\n"` or `"null\n"` whenever
the value changes. Buffer incoming bytes and split on newlines.

### Minimal example

```python
import socket

with socket.create_connection(("raspberrypi.local", 9999)) as sock:
    buf = ""
    while True:
        buf += sock.recv(64).decode()
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if line == "null":
                x = None
            else:
                x = int(line)
            print(f"X: {x}")
```

### With reconnection handling

```python
import socket
import time

HOST = "raspberrypi.local"
PORT = 9999

def stream_x():
    while True:
        try:
            with socket.create_connection((HOST, PORT), timeout=5) as sock:
                print("Connected")
                buf = ""
                while True:
                    chunk = sock.recv(64).decode()
                    if not chunk:
                        raise ConnectionResetError("server closed connection")
                    buf += chunk
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        x = None if line == "null" else int(line)
                        yield x
        except (OSError, ConnectionResetError) as e:
            print(f"Disconnected: {e}, retrying in 2s...")
            time.sleep(2)

for x in stream_x():
    print(f"X = {x}")
```

### Quick test from the terminal

```bash
nc raspberrypi.local 9999
```

You will see a stream of numbers (or `null`) printed whenever the detected
position changes.

---

## Architecture overview

```
Main thread
  ├── CaptureThread    — picamera2 read → threshold detect → JPEG encode → SharedState.update()
  ├── FlaskThread      — MJPEG HTTP server on :8080, waits on frame events
  └── TcpServerThread  — accepts connections on :9999
        └── TcpClientHandler (one per client) — blocks on X change, pushes line to socket
```

`SharedState` is the central thread-safe hub:
- Frame updates wake the Flask MJPEG generator via a `threading.Event`
- X updates wake all TCP client handlers via a `threading.Condition` with a
  monotonic version counter (so a handler that missed a change still wakes up)
