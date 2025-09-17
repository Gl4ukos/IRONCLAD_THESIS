import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# 1. Your waypoints
points = np.array([
    [0, 0],
    [0, 1],
    [1, 2],
    [2, 2],
    [3, 0],
    [2, -1],
    [0, -1]
])

# 2. Make "time" t for each point
t = np.arange(len(points))  # [0,1,2,3,...]

# 3. Fit splines for x(t), y(t)
cs_x = CubicSpline(t, points[:,0], bc_type='natural')
cs_y = CubicSpline(t, points[:,1], bc_type='natural')

# 4. Sample the spline
tt = np.linspace(0, len(points), 300)  # lots of samples
x = cs_x(tt)
y = cs_y(tt)

# 5. Get orientations
dx = cs_x(tt, 1)
dy = cs_y(tt, 1)
headings = np.arctan2(dy, dx)

# --- plot ---
plt.plot(points[:,0], points[:,1], 'ro--', label='waypoints')
plt.plot(x, y, 'b-', label='spline path')
plt.quiver(x[::30], y[::30], np.cos(headings[::30]), np.sin(headings[::30]),
           color='g', scale=20, width=0.005, label='heading')
plt.axis('equal')
plt.legend()
plt.show()
