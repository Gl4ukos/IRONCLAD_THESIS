import random
import csv
import math

world_file = "src/ackermann_simulation/sdf/trajectory_world.sdf"
trajectory_file = "src/informatics/pose_sequences/PLAN.csv"

# Trajectory corridor (empty space) - list of (x,y) points (loading from csv)
trajectory_corridor = []
file = open(trajectory_file,mode='r')
data = csv.reader(file)
world_x_bounds= [0,0]
world_y_bounds= [0,0]
for lines in data:
    if len(lines) != 7:
            continue
    x = round(float(lines[0]),3)
    y = round(float(lines[1]),3)
    if(x>world_x_bounds[1]):
        world_x_bounds[1] = x
    elif(x<world_x_bounds[0]):
        world_x_bounds[0] = x
    if(y>world_y_bounds[1]):
        world_y_bounds[1]=y
    elif(y<world_y_bounds[0]):
        world_y_bounds[0]=y
    trajectory_corridor.append((x, y))
world_x_bounds[1]+=5
world_x_bounds[0]-=5
world_y_bounds[1]+=5
world_y_bounds[0]-=5


x_span = abs(world_x_bounds[0]) + world_x_bounds[1]
y_span = abs(world_y_bounds[0]) + world_y_bounds[1]


#generating other trajectory corridors to give the impression of other roads

def generate_spline(start, end, num_points= int(2*(max(world_x_bounds[1] + abs(world_x_bounds[0]), world_y_bounds[1] + abs(world_y_bounds[0])))), max_deviation=0.0):
    """Generates a wavy spline from start to end with some random deviation."""
    x1 = start[0]
    y1 = start[1]
    x2 = end[0]
    y2 = end[1]
    spline = []
    for i in range(num_points):
        t = i / (num_points - 1)
        # linear interpolation
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        # add random deviation perpendicular to the line
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            nx = -dy / length  # normal vector x
            ny = dx / length   # normal vector y
            deviation = random.uniform(-max_deviation, max_deviation)
            x += nx * deviation
            y += ny * deviation
        spline.append((x, y))
    return spline

num_extra_roads = int(min(x_span, y_span) / 10)
num_extra_roads =0
print("roads: ", num_extra_roads)
extra_trajectory_corridors = []
for r in range(num_extra_roads):

    start = [0,0]
    end = [0,0]
    if(random.getrandbits(1) == 1):
        start[0] = world_x_bounds[0]
        start[1] = random.uniform(world_y_bounds[0], world_y_bounds[1])
    else:
        start[0] = random.uniform(world_x_bounds[0], world_x_bounds[1])
        start[1] = world_y_bounds[0]

    if(random.getrandbits(1) == 1):
        end[0] = world_x_bounds[1]
        end[1] = random.uniform(world_y_bounds[0], world_y_bounds[1])
    else:
        end[0] = random.uniform(world_x_bounds[0], world_x_bounds[1])
        end[1] = world_y_bounds[1]

    spline = generate_spline(start, end, max_deviation=0.2)
    extra_trajectory_corridors.append(spline)

alleys = int(min(x_span, y_span)/2)
print("alleys: ",alleys)
alley_corridors = []
alley_corridors_radius = 0.2
for r in range(alleys):
    start = [0,0]
    end = [0,0]

    if(random.getrandbits(1) ==1 ): #vertical
        if(random.getrandbits(1) ==1): #bottom
            start[1] = world_y_bounds[0]
        else: #top
            start[1] = world_y_bounds[1]
        end[1] = random.uniform(world_y_bounds[0], world_y_bounds[1])
        start[0] = random.uniform(world_x_bounds[0], world_y_bounds[1])
        end[0] = start[0]
    else: #horizontal
        if(random.getrandbits(1) ==1): #left
            start[0] = world_x_bounds[0]
        else: #right
            start[0] = world_x_bounds[1]
        end[0] = random.uniform(world_x_bounds[0], world_x_bounds[1])
        start[1] = random.uniform(world_y_bounds[0], world_y_bounds[1])
        end[1] = start[1]

    spline = generate_spline(start, end, max_deviation=0.0)
    alley_corridors.append(spline)


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

# Add pavement to main trajectory

corridor_radius = 1.5 # half of corridor width
corridor_height = 0.01
for i in range(len(trajectory_corridor)-1):
    x1, y1 = trajectory_corridor[i]
    x2, y2 = trajectory_corridor[i+1]

    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    num_segments = max(int((dist / corridor_radius)**2), 1)

    for j in range(num_segments):
        t = j / num_segments
        cx = x1 + t * (x2 - x1)
        cy = y1 + t * (y2 - y1)
        sdf_str += f"""
        <model name="corridor_hex_{i}_{j}">
          <static>true</static>
          <link name="link">
            <pose>{cx} {cy} {corridor_height/2} 0 0 0</pose>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>{corridor_radius}</radius>
                  <length>{corridor_height}</length>
                  <segments>6</segments>
                </cylinder>
              </geometry>
              <material>
                <ambient>0.5 0.5 0.5 1</ambient>
                <diffuse>0.5 0.5 0.5 1</diffuse>
                <specular>0.3 0.3 0.3 1</specular>
                <emissive>0 0 0 1</emissive>
              </material>
            </visual>
          </link>
        </model>
        """

# adding a highlighted trajectory path 
highlighted_corridor_radius = 0.02 # half of corridor width
highlighted_corridor_height = 0.03
for i in range(len(trajectory_corridor)):
    x1, y1 = trajectory_corridor[i]

    sdf_str += f"""
    <model name="highlighted_corridor_hex_{i}">
      <static>true</static>
      <link name="link">
        <pose>{x1} {y1} {highlighted_corridor_height/2} 0 0 0</pose>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{highlighted_corridor_radius}</radius>
              <length>{highlighted_corridor_height}</length>
              <segments>6</segments>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
    """


extra_corridor_radius = 0.3

# # #Adding pavement to extra trajectories
# for s_idx, spline in enumerate(alley_corridors):
#     for i in range(len(spline)-1):
#         x1, y1 = spline[i]
#         x2, y2 = spline[i+1]

#         # distance between points
#         dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)

#         # number of segments to fill the gap
#         num_segments = max(int((dist / alley_corridors_radius)**2), 1)

#         for j in range(num_segments):
#             t = j / num_segments
#             cx = x1 + t * (x2 - x1)
#             cy = y1 + t * (y2 - y1)

#             sdf_str += f"""
#             <model name="road_hex_{s_idx}_{i}_{j}">
#               <static>true</static>
#               <link name="link">
#                 <pose>{cx} {cy} {corridor_height/2} 0 0 0</pose>
#                 <visual name="visual">
#                   <geometry>
#                     <cylinder>
#                       <radius>{alley_corridors_radius}</radius>
#                       <length>{corridor_height}</length>
#                       <segments>5</segments>
#                     </cylinder>
#                   </geometry>
#                   <material>
#                     <ambient>0.2 0.2 0.2 1</ambient>
#                     <diffuse>0.2 0.2 0.2 1</diffuse>
#                     <specular>0.3 0.3 0.3 1</specular>
#                     <emissive>0 0 0 1</emissive>
#                   </material>
#                 </visual>
#               </link>
#             </model>
#             """


min_spacing = 0.1
max_spacing = 3.5

def collides_with_corridors(x, y, building_w, building_d):
    half_w = building_w / 2
    half_d = building_d / 2

    for tx, ty in trajectory_corridor:
        dx = abs(tx-x)
        dy = abs(ty-y)
        if dx<half_w + corridor_radius + min_spacing and dy<half_d+corridor_radius+min_spacing:
            return True

    for spline in extra_trajectory_corridors:
        for tx, ty in spline:
            dx = abs(tx - x)
            dy = abs(ty - y)

            if dx < half_w + extra_corridor_radius+min_spacing and dy < half_d + extra_corridor_radius + min_spacing:
                return True
    
    for spline in alley_corridors:
        for tx, ty in spline:
            dx = abs(tx - x)
            dy = abs(ty - y)

            if dx < half_w + alley_corridors_radius +min_spacing and dy < half_d + alley_corridors_radius + min_spacing:
                return True
    
    return False


def is_close_to_corridors(x, y, building_w, building_d):
    half_w = building_w / 2
    half_d = building_d / 2

    for tx, ty in trajectory_corridor:
        dx = abs(tx-x)
        dy = abs(ty-y)
        if dx<half_w + corridor_radius + max_spacing and dy<half_d+corridor_radius+max_spacing:
            return True

    for spline in extra_trajectory_corridors:
        for tx, ty in spline:
            dx = abs(tx - x)
            dy = abs(ty - y)

            if dx < half_w + extra_corridor_radius+max_spacing and dy < half_d + extra_corridor_radius + max_spacing:
                return True
    return False

min_size = 0.5
max_size = 2.5
num_passes = int(min(x_span, y_span))
total_blocks_per_pass = int((x_span + y_span))
print("Buildings (almost): ", total_blocks_per_pass*num_passes)
for pass_num in range(num_passes):
    for i in range(total_blocks_per_pass):
        w = random.uniform(min_size, max_size)
        d = random.uniform(min_size, max_size)
        
        x = random.uniform(world_x_bounds[0], world_x_bounds[1])
        y = random.uniform(world_y_bounds[0], world_y_bounds[1])
        
        # Skip collision check if you don't care about overlaps
        if collides_with_corridors(x, y, w,d):
            continue
        
        tries = 0
        while(collides_with_corridors(x,y,w,d)):
            w = random.gauss(mu=1.0, sigma=0.5)
            d = random.gauss(mu=1.0, sigma=0.5)
            tries +=1
            if(tries>50):
                break
            elif(tries>10):
              x = random.uniform(world_x_bounds[0], world_x_bounds[1])
              y = random.uniform(world_y_bounds[0], world_y_bounds[1])
            

        h = random.uniform(min_size*2, max_size*2)

        collision_check=0
        if (is_close_to_corridors(x,y,w,d) and (random.random() < 0.8)): # if its close to corridor, then high chance to shorten building so the corridor is more visible
            collision_check = 1
            h = h/4

        if(collision_check==1):
          block_sdf = f"""
          <model name="block_{pass_num}_{i}">
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
        else:
          block_sdf = f"""
          <model name="block_{pass_num}_{i}">
            <static>true</static>
            <link name="link">
              <pose>{x} {y} {h/2} 0 0 0</pose>
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
          collision_check=0





def gauss_clamped(mu, sigma, min_val, max_val):
    """Generate a Gaussian random number clamped into a range."""
    val = random.gauss(mu, sigma)
    return max(min_val, min(val, max_val))

num_of_small_buildings =0
# === Roadside buildings along extra splines ===
for s_idx, spline in enumerate(alley_corridors):
    for i in range(len(spline) - 1):
        x1, y1 = spline[i]
        x2, y2 = spline[i + 1]

        # distance between spline points
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # one building spot every ~2m
        num_spots = max(int(dist / 2.0), 1)

        for j in range(num_spots):
            t = j / num_spots
            cx = x1 + t * (x2 - x1)
            cy = y1 + t * (y2 - y1)

            # road direction vector
            dx = x2 - x1
            dy = y2 - y1
            length = math.sqrt(dx ** 2 + dy ** 2)
            if length == 0:
                continue
            dx /= length
            dy /= length

            # perpendicular vector
            px, py = -dy, dx

            # offset to left/right side (± corridor radius + gap)
            offset = extra_corridor_radius + 2.0
            side = random.choice([-1, 1])
            bx = cx + px * offset * side
            by = cy + py * offset * side

            # building dimensions (Gaussian)
            w = gauss_clamped(1.0, 0.5, 0.3, 2.0)
            d = gauss_clamped(1.0, 0.5, 0.3, 2.0)
            h = gauss_clamped(2.0, 0.7, 1.0, 4.0)

            if collides_with_corridors(bx, by, w, d):
                continue

            num_of_small_buildings+=1
            block_sdf = f"""
            <model name="roadside_block_{s_idx}_{i}_{j}">
              <static>true</static>
              <link name="link">
                <pose>{bx} {by} {h/2} 0 0 0</pose>
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
                    <ambient> 1 1 1 1</ambient>
                    <diffuse> 1 1 1 1 </diffuse>
                    <specular>0.9 0.9 0.9 1</specular>
                    <emissive>0.05 0.05 0.05 1</emissive>
                  </material>
                </visual>
              </link>
            </model>
            """
            sdf_str += block_sdf

sdf_str += "</world>\n</sdf>"
print("Complementary small buildings: ", num_of_small_buildings)

# Save world file
with open(world_file, "w") as f:
    f.write(sdf_str)

print(f"Mirror’s Edge world saved to {world_file}")
