import csv
from tf.transformations import quaternion_from_euler
import math
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import numpy as np

INTERPOLATING_RESOLUTION = 10
trajectory = []
interpolated_trajectory = []

print("Enter trajectory name: ")
name = "src/informatics/pose_sequences/PLAN" + input() + ".csv"

# Request waypoints
while True:
    print("give x,y (or q to quit):")
    x = input("x: ")
    if x.lower() == 'q':
        break
    y = input("y: ")

    try:
        x = float(x)
        y = float(y)
    except ValueError:
        print("Invalid number, try again.")
        continue

    trajectory.append((x, y, 0.0))  # store as tuple instead of dict

# Convert to numpy for spline
trajectory = np.array(trajectory)  # shape (N,3)

# Spline parameter
t = np.arange(len(trajectory))

cs_x = CubicSpline(t, trajectory[:, 0], bc_type='natural')
cs_y = CubicSpline(t, trajectory[:, 1], bc_type='natural')

# Sample the spline
tt = np.linspace(0, len(trajectory) - 1, INTERPOLATING_RESOLUTION*len(trajectory))
x = cs_x(tt)
y = cs_y(tt)

# Derivatives for heading
dx = cs_x(tt, 1)
dy = cs_y(tt, 1)
headings = np.arctan2(dy, dx)

# Build interpolated trajectory with quaternions
for xi, yi, hi in zip(x, y, headings):
    qx, qy, qz, qw = quaternion_from_euler(0, 0, hi)
    interpolated_trajectory.append({
        "x": xi,
        "y": yi,
        "z": 0.0,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw
    })

print(len(x), len(headings))

# Plot
plt.plot(trajectory[:, 1], trajectory[:, 0], 'ro--', label='waypoints')
plt.plot(y, x, 'b-', label='spline path')
plt.axis('equal')
plt.legend()
plt.show()

# Write CSV
with open(name, mode='w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=["x", "y", "z", "qx", "qy", "qz", "qw"])
    writer.writeheader()
    for pose in interpolated_trajectory:
        writer.writerow(pose)

print(f"Trajectory saved to {name}")
