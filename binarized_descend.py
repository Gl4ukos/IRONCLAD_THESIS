import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D

# -------------------------
# Vehicle / Ackermann Cost
# -------------------------
L = 2.5
v_ref = 6.0
delta_ref = 0.1

# Cost weights
w_v = 1.0
w_d2 = 8.0
w_d4 = 20.0
w_stability = 2.0

def ackermann_cost(v, delta):
    steer_err = delta - delta_ref
    steer_term = w_d2 * steer_err**2 + w_d4 * steer_err**4
    stability_term = w_stability * (v**2 * np.tan(delta)**2) / (L**2)
    vel_term = w_v * (v - v_ref)**2
    return vel_term + steer_term + stability_term

# -------------------------
# Higher-resolution grid
# -------------------------
v_choices = np.linspace(0, 12, 25)       # 25 velocity steps
delta_choices = np.linspace(-0.7, 0.7, 25)  # 25 steering steps
V, D = np.meshgrid(v_choices, delta_choices)
J = ackermann_cost(V, D)

# Global minimum
min_idx = np.argmin(J)
v_min = V.flatten()[min_idx]
d_min = D.flatten()[min_idx]

# -------------------------
# Discrete per-square descent
# -------------------------
start = (v_choices[0], delta_choices[0])
path_v = [start[0]]
path_d = [start[1]]
current = start

def get_neighbors(v, d):
    """Return neighbors in grid (up/down/left/right) within bounds."""
    neighbors = []
    i_v = np.argmin(np.abs(v_choices - v))
    i_d = np.argmin(np.abs(delta_choices - d))
    if i_v > 0:
        neighbors.append((v_choices[i_v-1], delta_choices[i_d]))
    if i_v < len(v_choices)-1:
        neighbors.append((v_choices[i_v+1], delta_choices[i_d]))
    if i_d > 0:
        neighbors.append((v_choices[i_v], delta_choices[i_d-1]))
    if i_d < len(delta_choices)-1:
        neighbors.append((v_choices[i_v], delta_choices[i_d+1]))
    return neighbors

max_steps = 200
for _ in range(max_steps):
    neighbors = get_neighbors(*current)
    costs = [ackermann_cost(v,d) for v,d in neighbors]
    min_idx = np.argmin(costs)
    next_v, next_d = neighbors[min_idx]
    if ackermann_cost(*current) <= costs[min_idx]:
        break
    path_v.append(next_v)
    path_d.append(next_d)
    current = (next_v, next_d)

path_J = [ackermann_cost(v,d) for v,d in zip(path_v, path_d)]

# -------------------------
# Plot & animate
# -------------------------
fig = plt.figure(figsize=(10,7))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(V.flatten(), D.flatten(), J.flatten(), c=J.flatten(), cmap='viridis', s=50)
ax.scatter(v_min, d_min, ackermann_cost(v_min, d_min), color='red', s=150, label='Global Minimum')

line, = ax.plot([], [], [], 'r-', lw=2, label='Descent Path')
point, = ax.plot([], [], [], 'ro', markersize=6)
ax.set_xlabel("Velocity (v)")
ax.set_ylabel("Steering (δ)")
ax.set_zlabel("Cost J(v, δ)")
ax.set_title("Discrete Gradient Descent - One Square per Step")
ax.legend()
ax.view_init(elev=35, azim=-60)

def update(frame):
    line.set_data(path_v[:frame+1], path_d[:frame+1])
    line.set_3d_properties(path_J[:frame+1])
    point.set_data([path_v[frame]], [path_d[frame]])
    point.set_3d_properties([path_J[frame]])
    return line, point

anim = FuncAnimation(fig, update, frames=len(path_v), interval=200, blit=True)

# -------------------------
# Save as GIF
# -------------------------
anim.save("discrete_descent.gif", writer=PillowWriter(fps=3))
plt.show()
