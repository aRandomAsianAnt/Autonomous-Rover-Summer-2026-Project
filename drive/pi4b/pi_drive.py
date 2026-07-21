#!/usr/bin/env python3
"""
Summer 2026 Rover Project — Pi-side manual teleop (WASD + N/M/B)
----------------------------------------------------------------
Run this directly on the Raspberry Pi (e.g. over SSH from your laptop).
Your SSH session carries the keystrokes live to this script; this script
relays drive commands out the Pi's hardware UART to the ESP32 (UART2),
using the exact "D,<speed>,<turn>" protocol built into the ESP32 drive
firmware. No firmware changes needed.

Controls:
  w        forward mode        (direction = +1)
  s        reverse mode        (direction = -1)
  a        steer input left    (turn_input -= TURN_STEP, persists)
  d        steer input right   (turn_input += TURN_STEP, persists)
  n        accelerate          (throttle += THROTTLE_STEP, up to 255)
  m        decelerate          (throttle -= THROTTLE_STEP, floor 0)
  space    recenter steering to 0
  b        HARD BRAKE — stop now, zeroes direction/turn/throttle
  q / ESC  hard brake and quit

Steering convention (matches real-car steering):
  'd' always means "steer input right" and 'a' always means "steer input
  left" — but the actual motor effect flips when reversing, the same way
  a car's rear swings the opposite direction from the front when backing
  up. So:
    w + d -> forward, turning right
    w + a -> forward, turning left
    s + d -> reversing, curving LEFT
    s + a -> reversing, curving RIGHT
  This is intentional, not a bug — it's what "s+d = reversing to the left"
  means in practice.

Safety: if no key is pressed for SAFETY_TIMEOUT_S seconds (e.g. the SSH
link drops), this script sends a hard brake automatically.

Setup notes for the Pi's hardware UART (GPIO14/15, /dev/serial0):
  - On Raspberry Pi 4, the primary UART can be claimed by the Bluetooth
    module and/or the serial console by default. To free it up:
      sudo raspi-config -> Interface Options -> Serial Port
        "login shell over serial?" -> No
        "serial port hardware enabled?" -> Yes
    Also ensure /boot/firmware/config.txt has both:
      dtoverlay=disable-bt
      enable_uart=1
    then reboot, and confirm: ls -l /dev/serial0  -> should point to ttyAMA0
  - Install pyserial if needed: pip install pyserial
"""

import sys
import termios
import tty
import select
import time
import serial

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/serial0"   # Pi hardware UART (GPIO14 TX / GPIO15 RX)
BAUD_RATE = 115200             # must match ESP32 PiSerial.begin() baud

THROTTLE_STEP = 15
TURN_STEP = 20
MAX_MAGNITUDE = 255

SAFETY_TIMEOUT_S = 2.0         # auto-brake if no keypress within this window
POLL_INTERVAL_S = 0.1          # how often to check for input / timeout

QUIT_KEYS = {"q", "\x1b"}      # 'q' or ESC


def read_key_nonblocking(timeout_s):
    """Return a single character if available within timeout_s, else None."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if ready:
        return sys.stdin.read(1)
    return None


class TeleopState:
    def __init__(self):
        self.direction = 0     # -1, 0, +1
        self.throttle = 0      # 0..255
        self.turn_input = 0    # -255..255, steer input as the driver would give it

    def speed(self):
        return self.direction * self.throttle

    def effective_turn(self):
        # Real-car convention: same steer input flips its ground effect in reverse.
        if self.direction >= 0:
            return self.turn_input
        return -self.turn_input

    def describe(self):
        mode = {1: "FWD", -1: "REV", 0: "NEUTRAL"}[self.direction]
        return (f"[{mode}] throttle={self.throttle:3d}  "
                f"steer_input={self.turn_input:4d}  "
                f"effective_turn={self.effective_turn():4d}")


def send_command(ser, line):
    ser.write((line + "\n").encode("ascii"))


def drain_esp32_output(ser):
    """Print any ACK/LOG lines the ESP32 sent back, without blocking."""
    while ser.in_waiting:
        try:
            resp = ser.readline().decode("ascii", errors="replace").strip()
        except Exception:
            break
        if resp:
            print(f"  ESP32> {resp}")


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    except serial.SerialException as e:
        print(f"Failed to open {SERIAL_PORT}: {e}")
        print("Check wiring, /dev/serial0 setup, and that nothing else has the port open.")
        sys.exit(1)

    state = TeleopState()

    print("Rover teleop ready.")
    print("Controls: w/s=direction  a/d=steer  n=accelerate  m=decelerate  space=center  b=BRAKE  q/ESC=quit")
    print(state.describe())

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_key_time = time.time()

    try:
        tty.setcbreak(fd)  # read keys one at a time, no Enter required

        while True:
            key = read_key_nonblocking(POLL_INTERVAL_S)
            now = time.time()

            if key is not None:
                last_key_time = now

                if key in QUIT_KEYS:
                    send_command(ser, "B")
                    print("\nQuit requested — hard brake sent.")
                    break

                elif key == "b":
                    send_command(ser, "B")
                    state.direction = 0
                    state.throttle = 0
                    state.turn_input = 0
                    print("\nBRAKE — stopped.")
                    print(state.describe())
                    drain_esp32_output(ser)
                    continue  # already sent B, skip the D command below

                elif key == "w":
                    state.direction = 1
                elif key == "s":
                    state.direction = -1
                elif key == "a":
                    state.turn_input = max(-MAX_MAGNITUDE, state.turn_input - TURN_STEP)
                elif key == "d":
                    state.turn_input = min(MAX_MAGNITUDE, state.turn_input + TURN_STEP)
                elif key == " ":
                    state.turn_input = 0
                elif key == "n":
                    state.throttle = min(MAX_MAGNITUDE, state.throttle + THROTTLE_STEP)
                elif key == "m":
                    state.throttle = max(0, state.throttle - THROTTLE_STEP)
                else:
                    # ignore unrecognized keys
                    drain_esp32_output(ser)
                    continue

                send_command(ser, f"D,{state.speed()},{state.effective_turn()}")
                print(state.describe())

            # Safety watchdog: no input for too long -> brake
            if now - last_key_time > SAFETY_TIMEOUT_S:
                if state.throttle != 0 or state.direction != 0 or state.turn_input != 0:
                    send_command(ser, "B")
                    state.direction = 0
                    state.throttle = 0
                    state.turn_input = 0
                    print("\n[safety] No input received — auto hard-brake sent.")
                last_key_time = now  # avoid resending brake every loop while idle

            drain_esp32_output(ser)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        try:
            send_command(ser, "B")  # always try to leave the rover stopped
        except Exception:
            pass
        ser.close()
        print("Serial closed, terminal restored.")


if __name__ == "__main__":
    main()