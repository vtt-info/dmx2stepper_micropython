#!/bin/bash
# Deploy and run PIO-based DMX receiver

set -e

echo "=== PIO DMX Receiver Deployment ==="
echo ""

# Clean the Pico
echo "[1/3] Cleaning Pico..."
mpremote connect /dev/ttyACM0 exec 'import os; [os.remove(f) for f in os.listdir() if f.endswith(".py")]' 2>/dev/null || true
echo "      Done"
echo ""

# Copy files
echo "[2/3] Copying files to Pico..."
cd /home/pi/Documents/DMX_input_PIO
mpremote connect /dev/ttyACM0 fs cp config.py dmx_receiver.py main.py :
echo "      Done"
echo ""

# Run
echo "[3/3] Running main.py on Pico..."
echo ""
mpremote connect /dev/ttyACM0 run main.py
