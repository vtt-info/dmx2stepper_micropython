"""
OpenCV Camera Streamer with Rectangle Detection

Captures camera feed at 25fps, detects a high-contrast slim rectangle,
overlays detection results, serves MJPEG over HTTP on :8080,
and exposes the center X coordinate via TCP on :9999.
"""

import threading
import time
import socket
import logging

import cv2
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
from libcamera import controls

# --- Tunable constants ---
# Brightness threshold: pixels above this value (0–255) are considered part of the bright shape
BRIGHT_THRESHOLD = 100   # range: 0–255
MIN_AREA = 50           # minimum blob area in pixels to be considered
JPEG_QUALITY = 80

# Manual exposure (auto exposure is disabled)
# Range: 1 – 40000 µs (hardware max is 66666 µs, but >40000 causes frame drops at 25 fps)
# 1000 µs = bright indoor/outdoor, 10000 µs = dim room, 40000 µs = very low light
EXPOSURE_TIME_US = 50000

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 25
FRAME_INTERVAL = 1.0 / CAMERA_FPS  # 0.04s

HTTP_PORT = 8080
TCP_PORT = 9999
MAX_RECONNECT_FAILURES = 5

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(threadName)s] %(levelname)s: %(message)s",
)
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Shared State
# ---------------------------------------------------------------------------

class SharedState:
    """Thread-safe container for the latest JPEG frame and detected X coord."""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame: bytes | None = None
        self._x: int | None = None
        self._x_version: int = 0
        self._frame_event = threading.Event()
        self._x_condition = threading.Condition(self._lock)

    def update(self, frame: bytes, x: int | None) -> None:
        with self._x_condition:
            self._frame = frame
            changed = x != self._x
            self._x = x
            if changed:
                self._x_version += 1
                self._x_condition.notify_all()
        self._frame_event.set()

    def get_frame(self) -> bytes | None:
        with self._lock:
            return self._frame

    def wait_for_frame(self, timeout: float = 1.0) -> bytes | None:
        self._frame_event.wait(timeout)
        self._frame_event.clear()
        return self.get_frame()

    def wait_for_x_update(
        self, last_version: int, timeout: float = 5.0
    ) -> tuple[int | None, int]:
        with self._x_condition:
            self._x_condition.wait_for(
                lambda: self._x_version != last_version, timeout=timeout
            )
            return self._x, self._x_version


# ---------------------------------------------------------------------------
# Bright Blob Detector
# ---------------------------------------------------------------------------

class EllipseDetector:
    """Finds the largest bright blob in the frame and reports its centroid X."""

    # Spinner segments drawn clockwise; each is an arc centre-angle offset
    _SPINNER_ANGLES = [0, 45, 90, 135, 180, 225, 270, 315]

    def detect(self, frame: np.ndarray) -> tuple[np.ndarray, int | None]:
        annotated = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, mask = cv2.threshold(gray, BRIGHT_THRESHOLD, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best_cnt = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area and area >= MIN_AREA:
                best_area = area
                best_cnt = cnt

        cx = None
        if best_cnt is not None:
            M = cv2.moments(best_cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                x, y, w, h = cv2.boundingRect(best_cnt)
                cx, cy = x + w // 2, y + h // 2

            cv2.drawContours(annotated, [best_cnt], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)
            cv2.line(annotated, (cx, 0), (cx, annotated.shape[0]), (0, 255, 0), 1)
            label = f"X: {cx}"
            label_w, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            if cx >= annotated.shape[1] * 2 // 3:
                label_x = cx - label_w - 5
            else:
                label_x = cx + 5
            cv2.putText(
                annotated,
                label,
                (label_x, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        # --- Activity spinner (bottom-right) ---
        h, w = annotated.shape[:2]
        cx_spin, cy_spin = w - 18, h - 18
        radius = 10
        t = time.monotonic()
        step = int(t * 8) % len(self._SPINNER_ANGLES)  # advances ~8 steps/sec
        for i, angle_deg in enumerate(self._SPINNER_ANGLES):
            # brightest segment leads; tail fades
            age = (step - i) % len(self._SPINNER_ANGLES)
            alpha = max(0, 255 - age * 32)
            color = (0, alpha, 0)
            rad = np.deg2rad(angle_deg)
            px = int(cx_spin + radius * np.cos(rad))
            py = int(cy_spin + radius * np.sin(rad))
            dot_r = 3 if i == step else 2
            cv2.circle(annotated, (px, py), dot_r, color, -1)

        return annotated, cx


# ---------------------------------------------------------------------------
# Capture Thread
# ---------------------------------------------------------------------------

class CaptureThread(threading.Thread):
    def __init__(self, state: SharedState, stop_event: threading.Event):
        super().__init__(name="CaptureThread", daemon=True)
        self._state = state
        self._stop = stop_event
        self._detector = EllipseDetector()

    def run(self):
        try:
            picam2 = Picamera2(CAMERA_INDEX)
            config = picam2.create_video_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
                sensor={"output_size": (2028, 1080)},
                controls={
                    "FrameRate": CAMERA_FPS,
                    "AeEnable": False,
                    "ExposureTime": EXPOSURE_TIME_US,
                },
            )
            picam2.configure(config)
            picam2.start()
            log.info(
                "Camera %d opened via picamera2: %dx%d @ %dfps",
                CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
            )
        except Exception as exc:
            log.error("Cannot open camera %d: %s", CAMERA_INDEX, exc)
            return

        try:
            while not self._stop.is_set():
                t0 = time.monotonic()

                frame = picam2.capture_array()
                # picamera2 RGB888 → OpenCV BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                annotated, cx = self._detector.detect(frame)

                ret, buf = cv2.imencode(
                    ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                )
                if ret:
                    self._state.update(buf.tobytes(), cx)

                elapsed = time.monotonic() - t0
                sleep_time = FRAME_INTERVAL - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            picam2.stop()
            picam2.close()
            log.info("CaptureThread stopped.")


# ---------------------------------------------------------------------------
# Flask MJPEG Server
# ---------------------------------------------------------------------------

app = Flask(__name__)
_shared_state: SharedState | None = None


def _mjpeg_generator():
    while True:
        frame = _shared_state.wait_for_frame(timeout=1.0)
        if frame is None:
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/stream")
def stream():
    return Response(
        _mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/")
def index():
    return (
        "<html><body style='margin:0;background:#000;'>"
        "<img src='/stream' style='max-width:100%;'/>"
        "</body></html>"
    )


def start_flask(state: SharedState):
    global _shared_state
    _shared_state = state
    t = threading.Thread(name="FlaskThread", daemon=True, target=lambda: app.run(
        host="0.0.0.0",
        port=HTTP_PORT,
        use_reloader=False,
        threaded=True,
    ))
    t.start()
    log.info("MJPEG server starting on http://0.0.0.0:%d/", HTTP_PORT)
    return t


# ---------------------------------------------------------------------------
# TCP Coordinate Server
# ---------------------------------------------------------------------------

class TcpClientHandler(threading.Thread):
    def __init__(self, conn: socket.socket, addr, state: SharedState):
        super().__init__(daemon=True, name=f"TcpClient-{addr[0]}:{addr[1]}")
        self._conn = conn
        self._addr = addr
        self._state = state

    def run(self):
        log.info("TCP client connected: %s:%d", *self._addr)
        last_version = -1
        try:
            while True:
                x, last_version = self._state.wait_for_x_update(last_version, timeout=5.0)
                msg = "null\n" if x is None else f"{x}\n"
                self._conn.sendall(msg.encode())
        except (BrokenPipeError, ConnectionResetError):
            log.info("TCP client disconnected: %s:%d", *self._addr)
        except Exception as exc:
            log.warning("TCP client error (%s:%d): %s", *self._addr, exc)
        finally:
            self._conn.close()


class TcpServerThread(threading.Thread):
    def __init__(self, state: SharedState, stop_event: threading.Event):
        super().__init__(name="TcpServerThread", daemon=True)
        self._state = state
        self._stop = stop_event

    def run(self):
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", TCP_PORT))
            srv.listen(8)
            srv.settimeout(1.0)
            log.info("TCP coordinate server listening on port %d", TCP_PORT)
        except OSError as exc:
            log.error("TCP server failed to bind on port %d: %s", TCP_PORT, exc)
            return

        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
                TcpClientHandler(conn, addr, self._state).start()
            except socket.timeout:
                continue
            except Exception as exc:
                log.error("TCP accept error: %s", exc)
                break

        srv.close()
        log.info("TcpServerThread stopped.")


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------

def main():
    state = SharedState()
    stop_event = threading.Event()

    capture_thread = CaptureThread(state, stop_event)
    tcp_thread = TcpServerThread(state, stop_event)

    capture_thread.start()
    tcp_thread.start()
    start_flask(state)

    log.info("Streamer running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down...")
        stop_event.set()
        capture_thread.join(timeout=3)
        tcp_thread.join(timeout=3)
        log.info("Done.")


if __name__ == "__main__":
    main()
