#!/usr/bin/env python3
import sys
import termios
import tty
import time

MAX_SPEED = 30.0
MAX_STEER = 1.0

def get_key():
    """
    Read a single keypress from stdin without requiring Enter.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def clip_vel(val):
    return max(min(val, MAX_SPEED), -MAX_SPEED)


def clip_steer(val):
    return max(min(val, MAX_STEER), -MAX_STEER)


def teleop_loop(callback=None, rate_hz=10):
    """
    Reads keyboard input (WASD/X/Q) to adjust speed and steering locally.
    Optionally calls `callback(speed, steer)` every iteration.

    Keys:
        w/s: increase/decrease speed
        a/d: steer left/right
        x:   reset to 0
        q:   quit
    """
    curr_speed = 0.0
    curr_steer = 0.0

    print("Press WASD to adjust, 'x' to reset, 'q' to quit.")

    period = 1.0 / rate_hz
    running = True

    while running:
        if sys.stdin.isatty():
            key = get_key()

            if key == 'w':
                curr_speed = clip_vel(curr_speed + 5.0)
            elif key == 's':
                curr_speed = clip_vel(curr_speed - 5.0)
            elif key == 'a':
                curr_steer = clip_steer(curr_steer + 0.1)
            elif key == 'd':
                curr_steer = clip_steer(curr_steer - 0.1)
            elif key == 'x':
                curr_speed = 0.0
                curr_steer = 0.0
            elif key == 'q':
                print("Exiting.")
                curr_speed = 0.0
                curr_steer = 0.0
                running = False
            else:
                pass  # ignore other keys

            print(f"\rSpeed: {curr_speed:>6.2f} | Steer: {curr_steer:>5.2f}", end="", flush=True)

        # call external function with latest values, if given
        if callback:
            callback(curr_speed, curr_steer)

        time.sleep(period)


if __name__ == "__main__":
    # Example usage: print values continuously
    def debug_callback(speed, steer):
        pass  # or use the values elsewhere

    teleop_loop(debug_callback)
