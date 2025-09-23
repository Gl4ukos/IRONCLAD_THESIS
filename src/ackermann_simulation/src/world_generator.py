import random

# Parameters
num_blocks = 50
min_size = 0.5
max_size = 3.0
min_spacing = 0.3
world_file = "src/ackermann_simulation/sdf/trajectory_world.sdf"

# Trajectory corridor (empty space) - list of (x,y) points
trajectory_corridor = [(0,0), (5,0), (10,5), (15,10)]

def collides_with_corridor(x, y, size):
    for tx, ty in trajectory_corridor:
        if abs(tx - x) < size + min_spacing and abs(ty - y) < size + min_spacing:
            return True
    return False

# Start SDF
sdf_str = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="mirror_edge_world">
"""

# Add directional light (sun)
sdf_str += """
  <light name="sun" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>1 1 1 1</diffuse>
    <specular>0.9 0.9 0.9 1</specular>
    <direction>-0.5 0.5 -1</direction>
    <cast_shadows>true</cast_shadows>
  </light>
"""

# Ground Plane
sdf_str += """
  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
"""

# Generate blocks (grid-based for tight packing)
grid_step = max_size + min_spacing
x_positions = [i * grid_step for i in range(int(-20/grid_step), int(21/grid_step)+1)]
y_positions = [i * grid_step for i in range(int(-20/grid_step), int(21/grid_step)+1)]

for i in range(num_blocks):
    w = random.uniform(min_size, max_size)
    d = random.uniform(min_size, max_size)
    h = random.uniform(min_size*2, max_size*2)

    x = random.choice(x_positions)
    y = random.choice(y_positions)

    if collides_with_corridor(x, y, max(w,d)):
        continue

    block_sdf = f"""
    <model name="block_{i}">
      <static>true</static>
      <link name="link">
        <pose>{x} {y} {h/2} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{w} {d} {h}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{w} {d} {h}</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.9 0.9 0.9 1</specular>
            <emissive>0.05 0.05 0.05 1</emissive>
          </material>
        </visual>
      </link>
    </model>
    """
    sdf_str += block_sdf

sdf_str += "</world>\n</sdf>"

# Save world file
with open(world_file, "w") as f:
    f.write(sdf_str)

print(f"Mirror’s Edge world saved to {world_file}")
