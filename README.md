# dmx2stepper

RP2040-based DMX-to-stepper controller firmware with PIO DMX input, PIO step generation, TMC2209 UART control, UART StallGuard homing, and OpenCV-based hardware-in-the-loop verification.

## Current Status

- DMX reception on the RP2040 is implemented with PIO.
- Step generation is implemented with PIO.
- TMC2209 configuration is handled over UART for microsteps, current, and driver control.
- Single-axis startup homing is working with UART StallGuard and has been optically verified.
- After homing, the active firmware moves to center and enters one-axis DMX runtime.
- One-axis runtime at 44 FPS DMX input has been validated functionally.
- Runtime optical verification is currently blocked by an unstable host camera stack.
- External `DIAG` is not part of the current MVP path.

## MVP Direction

The current MVP path is:

1. Reliable startup homing with PIO steps and UART StallGuard.
2. Stable one-axis DMX runtime.
3. Second-axis bring-up with the same architecture.
4. Dual-axis validation under sustained 44 FPS DMX input.

## Current DMX Runtime Behavior

The current runtime build is configured for simple manual testing:

- Channel 1: position MSB
- Channel 2: position LSB
- Channels 3-7: ignored in the current `position-only` runtime mode

The axis homes on startup, moves to center, and then responds to channels 1 and 2.

## Repository Layout

- [firmware/](firmware): active RP2040 firmware
- [hil/](hil): host-side DMX and vision verification tools
- [puzzle_pieces/](puzzle_pieces): historical experiments and reference implementations
- [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md): higher-level project plan
- [next_steps.md](next_steps.md): short-term milestones
- [diary.md](diary.md): recent implementation notes and findings

## Running The Firmware

Upload and run the current firmware:

```bash
./run_firmware.sh --upload
```

Run without re-uploading:

```bash
./run_firmware.sh
```

Direct deploy only:

```bash
bash firmware/deploy.sh /dev/ttyACM0
```

## Next Steps

- Restore reliable optical verification for runtime motion.
- Bring up the second axis with the same PIO + UART architecture.
- Validate dual-axis runtime under sustained DMX load.
- Add soak and fault-handling validation for MVP.
