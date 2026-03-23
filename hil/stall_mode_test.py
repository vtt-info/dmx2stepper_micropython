#!/usr/bin/env python3
"""Test different stall detection modes and compare position accuracy."""

import atexit
import os
import re
import subprocess
import sys
import time
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), 'firmware')
CONFIG_PATH = os.path.join(FIRMWARE_DIR, 'config.py')
CAPTURES_DIR = os.path.join(SCRIPT_DIR, 'captures')

DEFAULT_DEVICE = '/dev/ttyACM0'
TCP_PORT = 9999
CAPTURE_DURATION_S = 180
PROBE_S = 15

_original_config = None


# Mode configurations to test
STALL_MODES = [
    # (mode_name, config_patches)
    ('uart_baseline', {
        'HOME_STALL_MODE': '"uart"',
        'HOME_SGTHRS': '0',
        'HOME_POLL_MS': '5',
    }),
    ('diag_confirm_3', {
        'HOME_STALL_MODE': '"diag_confirm"',
        'HOME_SGTHRS': '1',
        'HOME_DIAG_CONFIRM_POLLS': '3',
        'HOME_DIAG_POLL_MS': '1',
    }),
    ('diag_confirm_5', {
        'HOME_STALL_MODE': '"diag_confirm"',
        'HOME_SGTHRS': '1',
        'HOME_DIAG_CONFIRM_POLLS': '5',
        'HOME_DIAG_POLL_MS': '1',
    }),
    ('diag_confirm_8', {
        'HOME_STALL_MODE': '"diag_confirm"',
        'HOME_SGTHRS': '1',
        'HOME_DIAG_CONFIRM_POLLS': '8',
        'HOME_DIAG_POLL_MS': '1',
    }),
    ('hybrid', {
        'HOME_STALL_MODE': '"hybrid"',
        'HOME_SGTHRS': '1',
        'HOME_DIAG_POLL_MS': '1',
    }),
    ('uart_fast_poll', {
        'HOME_STALL_MODE': '"uart"',
        'HOME_SGTHRS': '0',
        'HOME_POLL_MS': '1',
    }),
    ('uart_poll_2ms', {
        'HOME_STALL_MODE': '"uart"',
        'HOME_SGTHRS': '0',
        'HOME_POLL_MS': '2',
    }),
]


def read_config():
    with open(CONFIG_PATH, 'r') as f:
        return f.read()


def write_config(content):
    with open(CONFIG_PATH, 'w') as f:
        f.write(content)


def restore_config():
    global _original_config
    if _original_config is not None:
        write_config(_original_config)
        print('\n[restore] config.py restored to original state')
        _original_config = None


def patch_config(original, patches):
    """Apply key=value patches to config content using regex."""
    content = original
    for key, value in patches.items():
        pattern = rf'{key}\s*=\s*[^\n]+'
        replacement = f'{key} = {value}'
        if re.search(pattern, content):
            content = re.sub(pattern, replacement, content)
        else:
            # Key doesn't exist yet, add before "# Runtime motion limits"
            content = content.replace(
                '# Runtime motion limits',
                f'{key} = {value}\n\n# Runtime motion limits',
            )
    return content


def deploy(device):
    cmds = [
        ['mpremote', 'connect', device, 'exec',
         "import os; [os.remove(f) for f in os.listdir() if f not in ('boot.py',)]"],
        ['mpremote', 'connect', device, 'fs', 'cp',
         'config.py', 'dmx_receiver.py', 'pio_stepper.py',
         'tmc2209_uart.py', 'tmc2209.py', 'main.py', ':'],
        ['mpremote', 'connect', device, 'reset'],
    ]
    for cmd in cmds:
        r = subprocess.run(cmd, cwd=FIRMWARE_DIR, capture_output=True, text=True)
        if r.returncode != 0:
            print(f'  [warn] mpremote failed: {r.stderr.strip()}')
            return False
    return True


def estimate_settle_time(speed_base=500):
    """Estimate homing time in seconds."""
    actual_hz = speed_base * 16  # stallguard_adjustment(128) = 16
    max_steps = 4800 * 8  # scaled by microstep_distance_adjustment(128) = 8
    return int(2 * max_steps / actual_hz) + 10


def probe_valid_points():
    """Quick capture to verify homing succeeded and beam is visible."""
    try:
        r = subprocess.run(
            ['timeout', str(PROBE_S), 'nc', 'localhost', str(TCP_PORT)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            timeout=PROBE_S + 5,
        )
        count = 0
        for line in r.stdout.decode('utf-8', errors='replace').split('\n'):
            line = line.strip()
            if line and line != 'null' and ',' in line:
                try:
                    parts = line.split(',')
                    int(parts[0])
                    float(parts[1])
                    count += 1
                except (ValueError, IndexError):
                    pass
        return count
    except Exception:
        return 0


def capture_tcp(duration_s, output_path):
    """Capture TCP data for analysis."""
    try:
        r = subprocess.run(
            ['timeout', str(duration_s), 'nc', 'localhost', str(TCP_PORT)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            timeout=duration_s + 10,
        )
        with open(output_path, 'wb') as f:
            f.write(r.stdout)
        lines = r.stdout.decode('utf-8', errors='replace').strip().split('\n')
        valid = sum(1 for l in lines if l.strip() and l.strip() != 'null' and ',' in l)
        return valid
    except Exception as e:
        print(f'  [error] capture failed: {e}')
        return 0


def run_analysis(capture_path, output_dir):
    """Run analyze_x_data.py on a capture file."""
    analyze_script = os.path.join(SCRIPT_DIR, 'analyze_x_data.py')
    try:
        r = subprocess.run(
            ['python3', analyze_script, capture_path],
            capture_output=True, text=True, timeout=60,
            cwd=output_dir,
        )
        if r.returncode != 0:
            print(f'  [warn] analysis failed: {r.stderr.strip()[:200]}')
        return r.stdout
    except Exception as e:
        print(f'  [warn] analysis error: {e}')
        return ''


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DEVICE
    duration = CAPTURE_DURATION_S

    # Parse --modes flag
    selected_modes = None
    for i, arg in enumerate(sys.argv):
        if arg == '--modes' and i + 1 < len(sys.argv):
            selected_modes = sys.argv[i + 1].split(',')
        elif arg == '--duration' and i + 1 < len(sys.argv):
            duration = int(sys.argv[i + 1])

    modes = STALL_MODES
    if selected_modes:
        modes = [m for m in STALL_MODES if m[0] in selected_modes]
        if not modes:
            print(f'No matching modes. Available: {[m[0] for m in STALL_MODES]}')
            return

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(CAPTURES_DIR, f'stall_mode_test_{timestamp}')
    os.makedirs(run_dir, exist_ok=True)

    settle_time = estimate_settle_time()

    print(f'=== Stall Mode Comparison Test ===')
    print(f'Modes:    {[m[0] for m in modes]}')
    print(f'Duration: {duration}s per mode')
    print(f'Settle:   {settle_time}s')
    print(f'Output:   {run_dir}')
    est_min = len(modes) * (settle_time + PROBE_S + duration + 10) // 60
    print(f'Est time: ~{est_min} min')
    print()

    global _original_config
    _original_config = read_config()
    atexit.register(restore_config)

    results_summary = []

    try:
        for i, (mode_name, patches) in enumerate(modes):
            print(f'[{i+1}/{len(modes)}] Mode: {mode_name}')
            print(f'  Config: {patches}')

            mode_dir = os.path.join(run_dir, mode_name)
            os.makedirs(mode_dir, exist_ok=True)

            # Patch and deploy
            patched = patch_config(_original_config, patches)
            write_config(patched)

            print(f'  Deploying...')
            if not deploy(device):
                print(f'  [SKIP] Deploy failed\n')
                results_summary.append({'mode': mode_name, 'status': 'deploy_failed'})
                continue

            print(f'  Waiting {settle_time}s for homing...')
            time.sleep(settle_time)

            # Probe to verify homing worked
            print(f'  Probing for beam...')
            valid = probe_valid_points()
            print(f'  Probe: {valid} valid points')
            if valid < 5:
                print(f'  [SKIP] Homing failed or beam not visible\n')
                results_summary.append({'mode': mode_name, 'status': 'homing_failed', 'probe_points': valid})
                continue

            # Full capture
            capture_path = os.path.join(mode_dir, 'capture.txt')
            print(f'  Capturing {duration}s...')
            cap_valid = capture_tcp(duration, capture_path)
            print(f'  Captured {cap_valid} valid points')

            # Analysis
            print(f'  Analyzing...')
            analysis_output = run_analysis(capture_path, mode_dir)

            results_summary.append({
                'mode': mode_name,
                'status': 'ok',
                'probe_points': valid,
                'capture_points': cap_valid,
                'analysis': analysis_output[:500] if analysis_output else '',
            })
            print()

    finally:
        restore_config()

    # Summary
    print('\n=== Results Summary ===')
    for r in results_summary:
        status = r.get('status', '?')
        mode = r.get('mode', '?')
        if status == 'ok':
            print(f'  {mode}: OK ({r.get("capture_points", 0)} points captured)')
        else:
            print(f'  {mode}: {status}')

    print(f'\nCaptures saved to: {run_dir}')
    print('Run compare_homing_speeds.py on each mode dir for detailed analysis.')


if __name__ == '__main__':
    main()
