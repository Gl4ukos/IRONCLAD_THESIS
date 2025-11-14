import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ----- Parameters -----
W_d = 7.0
W_h = 1.0
W_e = 2.0
l = 1.0
x_target = -2.0
y_target =3.1
theta_target = 0.0

# ----- Cost function -----
def cost(v, s):
    theta = (v / l) * np.tan(s)
    term1 = W_d * ((v * np.cos(theta) - x_target)**2 + (v * np.sin(theta) - y_target)**2)
    term2 = W_h * (theta - theta_target)**2
    term3 = W_e * s**2
    return term1 + term2 + term3

# ----- Numerical gradient -----
def gradient(v, s, h=1e-4):
    dC_dv = (cost(v + h, s) - cost(v - h, s)) / (2 * h)
    dC_ds = (cost(v, s + h) - cost(v, s - h)) / (2 * h)
    return np.array([dC_dv, dC_ds])

# ----- Create meshgrid for surface -----
v = np.linspace(0, 10, 200)
s = np.linspace(-0.75, 0.75, 200)
V, S = np.meshgrid(v, s)
C = cost(V, S)

# ----- Gradient descent path -----
# ----- Parameters -----
alpha_v = 1.0     # step size in velocity units (discrete)
alpha_s = 0.05    # step size in steering units (discrete)
steps = 200

v_path = [0.0]
s_path = [0.0]

for k in range(steps - 1):
    g = gradient(v_path[-1], s_path[-1], h=1e-3)

    # Determine direction only (sign of gradient)
    dir_v = -np.sign(g[0])   # negative gradient direction
    dir_s = -np.sign(g[1])

    if abs(g[0]) < 1e-3: dir_v = 0
    if abs(g[1]) < 1e-3: dir_s = 0


    # Discrete step updates
    v_new = v_path[-1] + dir_v * alpha_v
    s_new = s_path[-1] + dir_s * alpha_s

    # Bound values
    v_new = np.clip(v_new, 0, 10)
    s_new = np.clip(s_new, -0.75, 0.75)

    v_path.append(v_new)
    s_path.append(s_new)


C_path = [cost(v_path[i], s_path[i]) for i in range(len(v_path))]

# ----- 3D Plot -----
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(V, S, C, cmap='viridis', alpha=0.8, rstride=4, cstride=4)
ax.set_xlabel('v')
ax.set_ylabel('s')
ax.set_zlabel('Cost C(v, s)')
ax.set_title('The Cost Function (target: )'+str(x_target)+ ','+str(y_target))


# ----- Find global minimum -----
min_idx = np.unravel_index(np.argmin(C, axis=None), C.shape)
v_min = V[min_idx]
s_min = S[min_idx]
C_min = C[min_idx]

# ----- Plot the global minimum -----
ax.scatter(v_min, s_min, C_min, color='black', s=80, marker='*', label='Global Minimum (dt:1sec, Optimal Cmd:' + str(np.round(v_min, 2))+ ',' + str(np.round(s_min,2)) + '.')
ax.legend(loc='upper center', bbox_to_anchor=(0.2, -0.01), ncol=1)



# ----- Plot moving point -----
point, = ax.plot([], [], [], 'ro', markersize=8)
path_line, = ax.plot([], [], [], 'r-', lw=2)



# ----- Animation update -----
def update(frame):
    path_line.set_data(v_path[:frame+1], s_path[:frame+1])
    path_line.set_3d_properties(C_path[:frame+1])
    point.set_data([v_path[frame]], [s_path[frame]])
    point.set_3d_properties([C_path[frame]])
    return point, path_line


ani = FuncAnimation(fig, update, frames=360, interval=50, blit=False)

ani.event_source.stop()
plt.show()