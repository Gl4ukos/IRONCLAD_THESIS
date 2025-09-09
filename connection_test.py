import socket
import struct
import time

# Replace with your RPi's IP address
RPI_IP = "10.42.0.171"  
RPI_PORT = 5005

# UDP socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        # Example dummy commands
        velocity = 1.5   # m/s
        steering = 0.3   # radians

        # Pack the floats into bytes
        msg = struct.pack('ff', velocity, steering)
        sock.sendto(msg, (RPI_IP, RPI_PORT))
        print(f"Sent velocity={velocity}, steering={steering}")

        time.sleep(0.1)  # 10 Hz
except KeyboardInterrupt:
    print("Stopped sending commands.")
finally:
    sock.close()
