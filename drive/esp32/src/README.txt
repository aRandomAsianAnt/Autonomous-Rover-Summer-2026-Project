# ESP32 Drive Controller — README

Firmware for the ESP32 acting as the rover's real-time motor control
co-processor. Receives commands from the Raspberry Pi 4B over UART2 and
drives two Cytron MDD10A motor controllers (4x TT gearbox motors,
left/right grouped differential drive).

## Serial Command Protocol

All commands are single ASCII lines, newline-terminated (`\n`), sent over
either UART2 (from the Pi, primary) or USB/UART0 (for bench testing directly
in a Serial Monitor). Every command returns an `ACK,<cmd>` or `ERR,...`
response on the same line it was sent on.

| Command | Format | Description |
|---|---|---|
| **M** | `M,<left>,<right>` | Direct differential speed control. Each value -255 (full reverse) to 255 (full forward). |
| **D** | `D,<speed>,<turn>` | Speed + turn mixing. `speed` = forward/back magnitude, `turn` = steering bias (negative = left, positive = right). |
| **S** | `S` | Soft stop — ramps both sides down to 0 smoothly instead of cutting instantly. |
| **B** | `B` | Hard brake — immediate coast-to-zero. Also cancels any running speed test or path. |
| **T** | `T` | **Manual speed test** — non-blocking sweep of PWM duty 0→255→0 in steps, logging `LOG,SPEED_TEST,t=<ms>,duty=<value>` at each step. Used for PWM-to-real-world-speed calibration (no encoders yet, so pair this log with external timing). |
| **P** | `P` | Runs the programmed path defined in `pathSteps[]` (see `src/main.ino`) — a timed sequence of `{left, right, duration}` steps. Used for autonomous-path and ADAS response testing. |
| **X** | `X` | Aborts any running speed test or path immediately and stops both motors. |

**Examples:**
```
M,150,150      -> straight forward at ~59% speed
M,-100,-100    -> straight reverse
D,100,50       -> forward while turning right
B              -> emergency stop
T              -> run the calibration sweep
```

## Pinout

| Signal | ESP32 GPIO | MDD10A | Notes |
|---|---|---|---|
| Left-Front DIR | 32 | #1 (Left), DIR1 | |
| Left-Front PWM | 33 | #1 (Left), PWM1 | |
| Left-Rear DIR | 25 | #1 (Left), DIR2 | |
| Left-Rear PWM | 26 | #1 (Left), PWM2 | |
| Right-Front DIR | 27 | #2 (Right), DIR1 | |
| Right-Front PWM | 14 | #2 (Right), PWM1 | |
| Right-Rear DIR | 18 | #2 (Right), DIR2 | |
| Right-Rear PWM | 19 | #2 (Right), PWM2 | |
| UART2 RX2 | 16 | — | Pi GPIO14 (TX) -> here |
| UART2 TX2 | 17 | — | Pi GPIO15 (RX) <- here |

Full wiring verification/photos: see `Wiring & Pinouts` doc.

## Build / Flash (PlatformIO)

From inside `esp32/`:

```
pio run                # compile only, no hardware needed
pio device list         # confirm the ESP32 is detected before uploading
pio run -t upload       # compile + flash over USB-C
pio device monitor       # watch live serial output (Ctrl+C to exit)
```

On successful boot, expect to see:
```
LOG,ESP32_DRIVE_READY
```

If upload hangs on `Connecting....`, hold the **BOOT** button on the ESP32
and release once you see `Writing at 0x...` start.

## Firmware Structure (`src/main.ino`)

| Section | Purpose |
|---|---|
| `MotorChannel` | Single motor: one DIR pin + one PWM pin, `setSpeed(-255..255)` |
| `DriveSystem` | Left/right grouped control — `setDifferential()`, `driveMix()`, `hardBrake()`, `softStop()` |
| `SpeedTest` | Handles the `T` command — non-blocking PWM sweep + logging |
| `PathRunner` | Handles the `P` command — non-blocking timed step sequence from `pathSteps[]` |
| `handleCommand()` | Parses incoming serial lines and routes to the right system |
| `setup()` / `loop()` | Boot init + main loop (polls both serial ports, updates all three systems) |

## Safety Notes

- `hardBrake()` = coast-to-zero, not active electronic braking. The MDD10A in
  PWM+DIR mode doesn't support regen braking without reversing direction
  briefly ("plugging"), which risks current spikes on these gear motors —
  intentionally not implemented.
- Firmware forces a hard brake on boot (`setup()`) so motors don't move
  unexpectedly on power-up.
- Disconnect motor battery power (not signal wires) before flashing or
  bench-testing new firmware, until behavior is confirmed as expected.