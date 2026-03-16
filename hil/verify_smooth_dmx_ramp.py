#!/usr/bin/env python3
"""Run a smooth DMX ramp and score the resulting OpenCV trace."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
HIL_DIR = ROOT / "hil"
FIRMWARE_DIR = ROOT / "firmware"
CAPTURE_DIR = HIL_DIR / "captures"
DEFAULT_SCENARIO = HIL_DIR / "scenarios" / "smooth_position_ramp.csv"
REMOTE_HOMING_FILE = "homing_result.json"
REMOTE_STATUS_FILE = "controller_status.json"


def timestamped_path(prefix: str, suffix: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return CAPTURE_DIR / f"{prefix}_{stamp}{suffix}"


def run_command(cmd, cwd=None, check=True, capture_output=False):
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=check,
        text=True,
        capture_output=capture_output,
    )


def deploy_firmware(device: str) -> None:
    py_files = sorted(str(path.name) for path in FIRMWARE_DIR.glob("*.py"))
    if not py_files:
        raise RuntimeError("No firmware files found to deploy")
    run_command(["mpremote", "connect", device, "fs", "cp", *py_files, ":"], cwd=str(FIRMWARE_DIR))


def clear_remote_file(device: str, remote_path: str) -> None:
    run_command(
        ["mpremote", "connect", device, "fs", "rm", remote_path],
        check=False,
        capture_output=True,
    )


def build_exec_code(runtime_exit_after_ms: int, debug_logging: bool) -> str:
    return "; ".join(
        [
            "import sys",
            "sys.modules.pop('main', None)",
            "sys.modules.pop('config', None)",
            "import config",
            "config.RUN_RUNTIME_AFTER_HOMING=True",
            f"config.RUNTIME_EXIT_AFTER_MS={int(runtime_exit_after_ms)}",
            f"config.DEBUG_LOGGING={bool(debug_logging)}",
            "import main",
            "main.main()",
        ]
    )


def launch_firmware(device: str, runtime_exit_after_ms: int, debug_logging: bool):
    return subprocess.Popen(
        ["mpremote", "connect", device, "exec", build_exec_code(runtime_exit_after_ms, debug_logging)],
        cwd=str(FIRMWARE_DIR),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def read_remote_json(device: str, remote_path: str, timeout_s: float, require_done: bool = False):
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        proc = run_command(
            ["mpremote", "connect", device, "fs", "cat", f":{remote_path}"],
            check=False,
            capture_output=True,
        )
        if proc.returncode == 0 and proc.stdout.strip():
            try:
                payload = json.loads(proc.stdout)
            except json.JSONDecodeError as exc:
                last_error = f"invalid JSON in {remote_path}: {exc}"
            else:
                if not require_done or payload.get("status") == "done":
                    return payload
                last_error = f"{remote_path} still marked running"
        else:
            last_error = proc.stderr.strip() or f"{remote_path} not available yet"
        time.sleep(0.5)
    raise RuntimeError(last_error or f"timed out waiting for {remote_path}")


def load_vision_rows(path: Path, axis: str):
    rows = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row["axis"] != axis or row["visible"] != "1":
                continue
            filtered = row["filtered_angle_deg"]
            if not filtered:
                continue
            rows.append((float(row["t_monotonic"]), float(filtered)))
    return rows


def unwrap_series(values):
    if not values:
        return []

    unwrapped = [values[0]]
    for value in values[1:]:
        candidates = [value - 360.0, value, value + 360.0]
        chosen = min(candidates, key=lambda item: abs(item - unwrapped[-1]))
        unwrapped.append(chosen)
    return unwrapped


def unwrap_vision_rows(rows):
    if not rows:
        return []
    values = unwrap_series([value for _, value in rows])
    return [(rows[index][0], values[index]) for index in range(len(rows))]


def load_target_series(path: Path, msb_channel: int = 1, lsb_channel: int = 2):
    state = {msb_channel: None, lsb_channel: None}
    series = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            channel = int(row["channel"])
            if channel not in state:
                continue
            state[channel] = int(row["value"])
            if state[msb_channel] is None or state[lsb_channel] is None:
                continue
            target_u16 = (int(state[msb_channel]) << 8) | int(state[lsb_channel])
            timestamp = float(row["t_monotonic"])
            if series and abs(series[-1][0] - timestamp) < 1e-6 and series[-1][1] == target_u16:
                continue
            if series and series[-1][1] == target_u16:
                continue
            series.append((timestamp, target_u16))
    return series


def delta_sign(delta: float, epsilon: float) -> int:
    if delta > epsilon:
        return 1
    if delta < -epsilon:
        return -1
    return 0


def extract_target_segments(target_series, min_delta_u16: int):
    segments = []
    if len(target_series) < 2:
        return segments

    start_index = None
    current_sign = 0
    last_index = 0

    for index in range(1, len(target_series)):
        delta = target_series[index][1] - target_series[index - 1][1]
        sign = delta_sign(delta, 16.0)
        if sign == 0:
            continue

        if current_sign == 0:
            current_sign = sign
            start_index = index - 1
            last_index = index
            continue

        if sign == current_sign:
            last_index = index
            continue

        start = target_series[start_index]
        end = target_series[last_index]
        if abs(end[1] - start[1]) >= int(min_delta_u16):
            segments.append(
                {
                    "start_t": start[0],
                    "end_t": end[0],
                    "start_u16": int(start[1]),
                    "end_u16": int(end[1]),
                    "direction": int(current_sign),
                }
            )

        current_sign = sign
        start_index = index - 1
        last_index = index

    if current_sign != 0 and start_index is not None:
        start = target_series[start_index]
        end = target_series[last_index]
        if abs(end[1] - start[1]) >= int(min_delta_u16):
            segments.append(
                {
                    "start_t": start[0],
                    "end_t": end[0],
                    "start_u16": int(start[1]),
                    "end_u16": int(end[1]),
                    "direction": int(current_sign),
                }
            )

    return segments


def evaluate_segment(segment, vision_series, lead_s: float, lag_s: float, noise_deg: float):
    window_start = float(segment["start_t"]) - float(lead_s)
    window_end = float(segment["end_t"]) + float(lag_s)
    window = [(timestamp, angle) for timestamp, angle in vision_series if window_start <= timestamp <= window_end]

    if len(window) < 4:
        return {
            "pass": False,
            "message": "not enough visible samples in segment window",
            "visible_samples": len(window),
            "net_move_deg": None,
            "monotonic_ratio": None,
            "max_step_deg": None,
            "backtrack_deg": None,
        }

    direction = int(segment["direction"])
    deltas = []
    for index in range(1, len(window)):
        delta = window[index][1] - window[index - 1][1]
        deltas.append(delta)

    significant = [delta for delta in deltas if abs(delta) >= float(noise_deg)]
    if not significant:
        return {
            "pass": False,
            "message": "segment contains no significant visible motion",
            "visible_samples": len(window),
            "net_move_deg": 0.0,
            "monotonic_ratio": 0.0,
            "max_step_deg": 0.0,
            "backtrack_deg": 0.0,
        }

    total_abs = sum(abs(delta) for delta in significant)
    backtrack = sum(abs(delta) for delta in significant if direction * delta < 0.0)
    monotonic_ratio = max(0.0, 1.0 - (backtrack / total_abs if total_abs > 0 else 1.0))
    max_step = max(abs(delta) for delta in significant)
    net_move = direction * (window[-1][1] - window[0][1])

    return {
        "pass": True,
        "message": "ok",
        "visible_samples": len(window),
        "net_move_deg": round(net_move, 3),
        "monotonic_ratio": round(monotonic_ratio, 4),
        "max_step_deg": round(max_step, 3),
        "backtrack_deg": round(backtrack, 3),
    }


def evaluate_smoothness(
    vision_rows,
    target_series,
    min_visible_samples: int,
    min_travel_deg: float,
    min_segment_move_deg: float,
    min_monotonic_ratio: float,
    max_step_deg: float,
    segment_lead_s: float,
    segment_lag_s: float,
    noise_deg: float,
):
    if len(vision_rows) < int(min_visible_samples):
        return False, "not enough visible vision samples", {
            "visible_samples": len(vision_rows),
            "travel_span_deg": None,
            "segment_count": 0,
            "segments": [],
        }

    vision_series = unwrap_vision_rows(vision_rows)
    travel_span = max(angle for _, angle in vision_series) - min(angle for _, angle in vision_series)
    if travel_span < float(min_travel_deg):
        return False, "observed travel span too small", {
            "visible_samples": len(vision_rows),
            "travel_span_deg": round(travel_span, 3),
            "segment_count": 0,
            "segments": [],
        }

    segments = extract_target_segments(target_series, min_delta_u16=1024)
    if len(segments) < 3:
        return False, "stimulus did not produce enough target ramp segments", {
            "visible_samples": len(vision_rows),
            "travel_span_deg": round(travel_span, 3),
            "segment_count": len(segments),
            "segments": segments,
        }

    segment_results = []
    failures = []
    for index, segment in enumerate(segments, start=1):
        segment_result = evaluate_segment(
            segment,
            vision_series,
            lead_s=segment_lead_s,
            lag_s=segment_lag_s,
            noise_deg=noise_deg,
        )
        segment_result["index"] = index
        segment_result["direction"] = int(segment["direction"])
        segment_result["start_u16"] = int(segment["start_u16"])
        segment_result["end_u16"] = int(segment["end_u16"])
        segment_results.append(segment_result)

        if not segment_result["pass"]:
            failures.append(f"segment {index}: {segment_result['message']}")
            continue
        if float(segment_result["net_move_deg"]) < float(min_segment_move_deg):
            failures.append(f"segment {index}: net visible move too small")
        if float(segment_result["monotonic_ratio"]) < float(min_monotonic_ratio):
            failures.append(f"segment {index}: excessive backtracking in vision trace")
        if float(segment_result["max_step_deg"]) > float(max_step_deg):
            failures.append(f"segment {index}: frame-to-frame vision jump too large")

    details = {
        "visible_samples": len(vision_rows),
        "travel_span_deg": round(travel_span, 3),
        "segment_count": len(segments),
        "segments": segment_results,
    }
    if failures:
        return False, "; ".join(failures), details
    return True, "vision trace stayed smooth across the commanded ramps", details


def build_argument_parser():
    parser = argparse.ArgumentParser(description="Verify smooth DMX ramps against OpenCV data")
    parser.add_argument("--device", default="/dev/ttyACM0", help="MicroPython device path for mpremote")
    parser.add_argument("--upload", action="store_true", help="Upload firmware before running verification")
    parser.add_argument("--axis", default="T1", choices=("T1", "T2"), help="Observed axis to evaluate")
    parser.add_argument("--scenario", default=str(DEFAULT_SCENARIO), help="Scenario CSV for dmx_stimulus.py")
    parser.add_argument("--dmx-fps", type=float, default=44.0, help="DMX send rate during the scenario")
    parser.add_argument("--vision-duration-s", type=float, default=18.0, help="Vision capture duration for the ramp run")
    parser.add_argument("--vision-warmup-s", type=float, default=0.5, help="Camera warmup before stimulus starts")
    parser.add_argument("--startup-wait-s", type=float, default=13.0, help="Time reserved for startup homing before stimulus starts")
    parser.add_argument("--segment-lead-s", type=float, default=0.2, help="Time to include before each ramp segment")
    parser.add_argument("--segment-lag-s", type=float, default=0.75, help="Time to include after each ramp segment")
    parser.add_argument("--min-frame-count", type=int, default=200, help="Minimum DMX frames the Pico must report")
    parser.add_argument("--min-visible-samples", type=int, default=80, help="Minimum visible OpenCV samples required")
    parser.add_argument("--min-travel-deg", type=float, default=80.0, help="Minimum total visible travel span required")
    parser.add_argument("--min-segment-move-deg", type=float, default=20.0, help="Minimum net move per ramp segment")
    parser.add_argument("--min-monotonic-ratio", type=float, default=0.75, help="Minimum monotonic ratio for each ramp segment")
    parser.add_argument("--max-step-deg", type=float, default=35.0, help="Maximum allowed frame-to-frame jump in the filtered vision trace")
    parser.add_argument("--noise-deg", type=float, default=0.75, help="Ignore smaller vision deltas when scoring smoothness")
    parser.add_argument("--debug-firmware", action="store_true", help="Enable RP2040 debug logging during the run")
    return parser


def terminate_process(proc):
    if proc is None:
        return ""
    try:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=2.0)
        else:
            proc.wait(timeout=0.1)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            return ""

    if proc.stdout is None:
        return ""
    try:
        return proc.stdout.read() or ""
    except Exception:
        return ""


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    stimulus_output = timestamped_path("dmx_smooth_ramp", ".csv")
    vision_output = timestamped_path("vision_smooth_ramp", ".csv")
    homing_output = timestamped_path("smooth_ramp_homing_result", ".json")
    status_output = timestamped_path("smooth_ramp_runtime_status", ".json")
    summary_output = timestamped_path("smooth_ramp_summary", ".json")

    if args.upload:
        deploy_firmware(args.device)

    clear_remote_file(args.device, REMOTE_HOMING_FILE)
    clear_remote_file(args.device, REMOTE_STATUS_FILE)

    runtime_window_ms = int((args.startup_wait_s + args.vision_duration_s + 3.0) * 1000)
    firmware_wait_s = max(args.startup_wait_s + args.vision_duration_s + 6.0, (runtime_window_ms / 1000.0) + 4.0)
    firmware_proc = launch_firmware(
        args.device,
        runtime_exit_after_ms=runtime_window_ms,
        debug_logging=bool(args.debug_firmware),
    )
    firmware_output = ""

    time.sleep(args.startup_wait_s)

    vision_cmd = [
        sys.executable,
        str(HIL_DIR / "vision_observer.py"),
        "--output",
        str(vision_output),
        "--duration-s",
        str(args.vision_duration_s),
        "--prefix",
        "vision_smooth_ramp",
    ]
    stimulus_cmd = [
        sys.executable,
        str(HIL_DIR / "dmx_stimulus.py"),
        "--output",
        str(stimulus_output),
        "--fps",
        str(args.dmx_fps),
        "scenario",
        "--path",
        str(Path(args.scenario)),
    ]

    vision_output_text = ""
    stimulus_output_text = ""
    vision_proc = subprocess.Popen(
        vision_cmd,
        cwd=str(HIL_DIR),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    try:
        time.sleep(args.vision_warmup_s)
        if vision_proc.poll() is not None:
            vision_output_text = terminate_process(vision_proc)
            firmware_output = terminate_process(firmware_proc)
            if vision_output_text.strip():
                print(vision_output_text.strip())
            if firmware_output.strip():
                print(firmware_output.strip())
            print("[FAIL] Vision observer exited before stimulus started")
            return 1

        stimulus_proc = subprocess.run(stimulus_cmd, cwd=str(HIL_DIR), check=False, text=True, capture_output=True)
        stimulus_output_text = (stimulus_proc.stdout or "") + (stimulus_proc.stderr or "")
        if stimulus_proc.returncode != 0:
            vision_output_text = terminate_process(vision_proc)
            firmware_output = terminate_process(firmware_proc)
            if stimulus_output_text.strip():
                print(stimulus_output_text.strip())
            if vision_output_text.strip():
                print(vision_output_text.strip())
            if firmware_output.strip():
                print(firmware_output.strip())
            return stimulus_proc.returncode

        if vision_proc.poll() is not None:
            vision_output_text = terminate_process(vision_proc)
            firmware_output = terminate_process(firmware_proc)
            if stimulus_output_text.strip():
                print(stimulus_output_text.strip())
            if vision_output_text.strip():
                print(vision_output_text.strip())
            if firmware_output.strip():
                print(firmware_output.strip())
            print("[FAIL] Vision observer exited during the DMX ramp run")
            return 1

        try:
            vision_output_text, _ = vision_proc.communicate(timeout=max(1.0, args.vision_duration_s + 3.0))
        except subprocess.TimeoutExpired:
            vision_proc.kill()
            vision_output_text, _ = vision_proc.communicate()

        if vision_proc.returncode not in (0, None):
            firmware_output = terminate_process(firmware_proc)
            if stimulus_output_text.strip():
                print(stimulus_output_text.strip())
            if vision_output_text.strip():
                print(vision_output_text.strip())
            if firmware_output.strip():
                print(firmware_output.strip())
            print("[FAIL] Vision observer did not complete successfully")
            return 1

        firmware_output, _ = firmware_proc.communicate(timeout=max(1.0, firmware_wait_s))
    finally:
        if vision_proc.poll() is None:
            vision_output_text = terminate_process(vision_proc)
        if firmware_proc.poll() is None:
            firmware_output = terminate_process(firmware_proc)

    if vision_proc.returncode not in (0, None):
        print(vision_output_text.strip())
        print("[FAIL] Vision observer did not complete successfully")
        return 1

    homing_result = read_remote_json(args.device, REMOTE_HOMING_FILE, 8.0, require_done=True)
    homing_output.write_text(json.dumps(homing_result, indent=2))
    runtime_status = read_remote_json(args.device, REMOTE_STATUS_FILE, 8.0, require_done=False)
    status_output.write_text(json.dumps(runtime_status, indent=2))

    vision_rows = load_vision_rows(vision_output, args.axis)
    target_series = load_target_series(stimulus_output)
    vision_ok, vision_message, vision_details = evaluate_smoothness(
        vision_rows,
        target_series,
        min_visible_samples=args.min_visible_samples,
        min_travel_deg=args.min_travel_deg,
        min_segment_move_deg=args.min_segment_move_deg,
        min_monotonic_ratio=args.min_monotonic_ratio,
        max_step_deg=args.max_step_deg,
        segment_lead_s=args.segment_lead_s,
        segment_lag_s=args.segment_lag_s,
        noise_deg=args.noise_deg,
    )

    firmware_ok = (
        bool(homing_result.get("success"))
        and bool(runtime_status.get("runtime_active"))
        and int(runtime_status.get("dmx_frame_count", 0)) >= int(args.min_frame_count)
    )

    summary = {
        "firmware_ok": bool(firmware_ok),
        "vision_ok": bool(vision_ok),
        "vision_message": vision_message,
        "homing_result_path": str(homing_output),
        "runtime_status_path": str(status_output),
        "vision_log_path": str(vision_output),
        "stimulus_log_path": str(stimulus_output),
        "firmware_console_nonempty": bool(firmware_output.strip()),
        "vision_details": vision_details,
        "runtime_status": {
            "runtime_active": runtime_status.get("runtime_active"),
            "dmx_frame_count": runtime_status.get("dmx_frame_count"),
            "current_position_steps": runtime_status.get("current_position_steps"),
            "target_position_steps": runtime_status.get("target_position_steps"),
            "selected_trial": runtime_status.get("selected_trial"),
        },
        "homing_summary": {
            "success": homing_result.get("success"),
            "centered": homing_result.get("centered"),
            "selected_trial": homing_result.get("selected_trial"),
        },
    }
    summary_output.write_text(json.dumps(summary, indent=2))

    print(f"[INFO] Homing result: {homing_output}")
    print(f"[INFO] Runtime status: {status_output}")
    print(f"[INFO] Vision log: {vision_output}")
    print(f"[INFO] Stimulus log: {stimulus_output}")
    print(f"[INFO] Summary: {summary_output}")
    if firmware_output.strip():
        print("[INFO] Firmware console captured")
    print(
        "[INFO] Firmware summary: runtime_active={} dmx_frames={} selected_trial={} pos={}/{}".format(
            runtime_status.get("runtime_active"),
            runtime_status.get("dmx_frame_count"),
            runtime_status.get("selected_trial"),
            runtime_status.get("current_position_steps"),
            runtime_status.get("target_position_steps"),
        )
    )
    print(
        "[INFO] Vision summary: travel_span={} deg visible_samples={} segment_count={}".format(
            vision_details.get("travel_span_deg"),
            vision_details.get("visible_samples"),
            vision_details.get("segment_count"),
        )
    )
    for segment in vision_details.get("segments", []):
        print(
            "[INFO] Segment {index}: dir={direction} move={net_move_deg} deg monotonic={monotonic_ratio} max_step={max_step_deg} backtrack={backtrack_deg}".format(
                **segment
            )
        )

    if firmware_output.strip():
        print(firmware_output.strip())

    if firmware_ok and vision_ok:
        print(f"[PASS] {vision_message}")
        return 0

    if not firmware_ok:
        print("[FAIL] Firmware did not report stable DMX runtime activity")
        return 1

    print(f"[FAIL] {vision_message}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
