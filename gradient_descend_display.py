import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# --- Vehicle / cost parameters ---
L = 2.5             # wheelbase (m)
v_ref = 6.0         # desired velocity
delta_ref = 0.1     # desired steering (rad)

# Weights (tune these to make steering cost harsher)
w_v = 1.0           # velocity tracking weight
w_d2 = 8.0          # quadratic steering weight (increased)
w_d4 = 20.0         # quartic steering weight (heavily punishes large delta errors)
w_stability = 2.0   # lateral-acceleration weight (increased)

# Control space for plotting
v = np.linspace(0, 12, 200)
delta = np.linspace(-0.7, 0.7, 200)
V, D = np.meshgrid(v, delta)

def ackermann_cost(v_, delta_):
    steer_err = delta_ - delta_ref
    steer_term = w_d2 * steer_err**2 + w_d4 * steer_err**4
    stability_term = w_stability * (v_**2 * np.tan(delta_)**2) / (L**2)
    vel_term = w_v * (v_ - v_ref)**2
    return vel_term + steer_term + stability_term





J = ackermann_cost(V, D)

def grad_cost(v_, delta_):
    dJdv = 2 * w_v * (v_ - v_ref) + (2 * w_stability * v_ * np.tan(delta_)**2) / (L**2)

    steer_err = (delta_ - delta_ref)
    dJdd = 2 * w_d2 * steer_err + 4 * w_d4 * steer_err**3
    dJdd += (2 * w_stability * v_**2 * np.tan(delta_) * (1.0 / np.cos(delta_)**2)) / (L**2)

    return dJdv, dJdd

v0, d0 = 0.0, 0.0   # start
alpha = 0.03         # learning rate (may need tuning)
steps = 60

path_v, path_d = [v0], [d0]
max_grad_norm = 5.0  # clip gradient magnitude to avoid huge jumps
for _ in range(steps):
    dv, dd = grad_cost(path_v[-1], path_d[-1])
    grad_norm = np.hypot(dv, dd)
    if grad_norm > max_grad_norm:
        dv *= max_grad_norm / grad_norm
        dd *= max_grad_norm / grad_norm
    v_new = path_v[-1] - alpha * dv
    d_new = path_d[-1] - alpha * dd
    v_new = np.clip(v_new, v.min(), v.max())
    d_new = np.clip(d_new, delta.min(), delta.max())
    path_v.append(v_new)
    path_d.append(d_new)

path_J = [ackermann_cost(vv, dd) for vv, dd in zip(path_v, path_d)]

# --- Plot & animate ---
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
surface = ax.plot_surface(V, D, J, cmap='viridis', alpha=0.75, linewidth=0, antialiased=False)

# Mark target and start
ax.scatter(v_ref, delta_ref, ackermann_cost(v_ref, delta_ref), color='black', s=60, label='Target')
ax.scatter(v0, d0, ackermann_cost(v0, d0), color='blue', s=40, label='Start')

# Animated path and point
line, = ax.plot([], [], [], 'r-', lw=2, label='GD Path')
point, = ax.plot([], [], [], 'ro', markersize=6)

ax.set_xlabel("Velocity (v) [m/s]")
ax.set_ylabel("Steering (δ) [rad]")
ax.set_zlabel("Cost J(v, δ)")
ax.set_title("Gradient Descent with Strong Steering Penalty (Ackermann)")
ax.legend()
ax.view_init(elev=35, azim=-60)

def update(frame):
    # update GD path
    line.set_data(path_v[:frame], path_d[:frame])
    line.set_3d_properties(path_J[:frame])
    point.set_data([path_v[frame]], [path_d[frame]])
    point.set_3d_properties([path_J[frame]])
    
    # rotate camera
    azim = -60 + frame * 2  # change 2 to adjust rotation speed
    ax.view_init(elev=35, azim=azim)
    
    return line, point


anim = FuncAnimation(fig, update, frames=len(path_v), interval=200, blit=True)


plt.tight_layout()

plt.show()
