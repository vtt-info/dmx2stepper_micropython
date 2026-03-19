# DMX512 Receiver - PIO Version

A high-performance DMX512 receiver for RP2040 Pico using PIO (Programmable I/O) instead of hardware UART. This frees up both UART0 and UART1 for other project uses.

## Quick Start

```bash
./deploy.sh
```

This will:
1. Clean the Pico (remove all `.py` files)
2. Copy `config.py`, `dmx_receiver.py`, and `main.py` to the device
3. Run `main.py` and start receiving DMX frames

## Performance

- **Frame rate:** ~30 fps
- **Channel values:** Accurate and stable
- **Frame format:** 513 bytes (start code + 512 channels)
- **Resources used:** 1 PIO StateMachine, GPIO1
- **UARTs freed:** UART0 and UART1 available for other use

## Configuration

Edit `config.py` to adjust:
- `DMX_PIN`: GPIO pin for DMX input (default: 1)
- `START_CHANNEL`: First DMX channel to monitor (default: 1)
- `NUM_CHANNELS`: Number of channels to display (default: 8)
- `PRINT_INTERVAL`: Status print interval in seconds (default: 1.0)

## Files

- `config.py` - Configuration settings
- `dmx_receiver.py` - PIO-based DMX receiver class
- `main.py` - Application loop and status display
- `deploy.sh` - Deployment script

## How It Works

The PIO program samples bits at 8 cycles per bit (2MHz frequency) to match 250kbaud DMX:
1. Waits for start bit (line LOW)
2. Samples 8 data bits
3. Pushes byte to FIFO

Break detection uses direct GPIO register polling (`mem32[GPIO_IN]`) to detect the >44µs low pulse that marks frame boundaries. This approach works alongside any pin function configuration.

## Signal Loss Detection

If no frames are received for >1 second, the receiver prints "NO SIGNAL" but continues trying to receive.

## Example Output

```
[13:28:11] PIO DMX receiver initialized on GPIO1
[13:28:11] Waiting for DMX signal...

[13:28:12] Frame:     30 (29.8 fps) | CH1:255 | CH2:183 | CH3: 92 | CH4: 32 | CH5:  0 | CH6:  0 | CH7:210 | CH8:  0
[13:28:13] Frame:     60 (29.9 fps) | CH1:255 | CH2:183 | CH3: 92 | CH4: 32 | CH5:  0 | CH6:  0 | CH7:210 | CH8:  0
```

## Troubleshooting

- **No signal detected?** Check GPIO1 connection and verify DMX source is active
- **Start code errors?** Normal on first frame after startup; indicates proper frame synchronization
- **Low frame rate?** Check signal integrity and cable connections

## Deployment Alternatives

Deploy and run in one command:
```bash
cd /home/pi/Documents/DMX_input_PIO && \
mpremote connect /dev/ttyACM0 fs cp config.py dmx_receiver.py main.py : && \
mpremote connect /dev/ttyACM0 run main.py
```

Or use the UART version instead (preserves both PIO blocks):
```bash
./deploy.sh
```
