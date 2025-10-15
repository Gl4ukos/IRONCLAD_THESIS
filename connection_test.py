import socket
import struct
import time
import math
import sys, select, termios, tty

RPI_IP   = "139.91.61.87"
RPI_PORT = 5005

# Initial commands
velocity = 0.0    # m/s
steering = 0.0    # radians
vel_cap = 30.0
steering_cap = 0.7

# step sizes
VEL_STEP = 1
STEER_STEP = math.radians(5)   # 5° per keypress

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ----- Terminal helpers -----
old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())   # raw, non-blocking

print("Controls:\n"
      "  w/s : increase/decrease velocity\n"
      "  a/d : left/right steering\n"
      "  space : zero velocity & steering\n"
      "  q : quit\n")

try:
    while True:
        # Check if a key is waiting (non-blocking)
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'w':
                velocity += VEL_STEP
            elif key == 's':
                velocity -= VEL_STEP
            elif key == 'a':
                steering += STEER_STEP
            elif key == 'd':
                steering -= STEER_STEP
            elif key == ' ':
                velocity, steering = 0.0, 0.0
            elif key == 'q':
                break

        # Pack and send
        msg = struct.pack('ff', velocity, steering)
        sock.sendto(msg, (RPI_IP, RPI_PORT))
        print(f"\rVelocity={velocity:.2f} m/s | Steering={math.degrees(steering):.1f}°   ",
              end='', flush=True)

        time.sleep(0.1)  # ~10 Hz

except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    sock.close()
    print("\nStopped sending commands.")
