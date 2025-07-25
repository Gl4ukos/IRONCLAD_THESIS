
import yaml
from tf.transformations import quaternion_from_euler
import math
trajectory = []

print("Enter trajectory name: ")
name = "src/brain/utils/trajectory_"+ input() +".yaml"

pose_num = 0



#requesting and adding coordinates
while(True):

    print("give x,y (or q to quit):\n")
    x = input()
    if(x=='q'):
        break

    y = input()
    z = 0.0

    try:
        x = float(x)
        y = float(y)
    except ValueError:
        print("Invalid number, try again.")
        continue

    trajectory.append({'x': x, 'y': y, 'z': 0.0}) 


#adding orientations
for i in range(len(trajectory)):
    if i<len(trajectory) -1 and i>0:
        dx = trajectory[i + 1]['x'] - trajectory[i - 1]['x']
        dy = trajectory[i + 1]['y'] - trajectory[i - 1]['y']
    elif i<len(trajectory)-1:
        dx = trajectory[i + 1]['x'] - trajectory[i]['x']
        dy = trajectory[i + 1]['y'] - trajectory[i]['y']
    elif i > 0:
        dx = trajectory[i]['x'] - trajectory[i - 1]['x']
        dy = trajectory[i]['y'] - trajectory[i - 1]['y']
    else:
        dx = 1.0
        dy = 0.0

    yaw = math.atan2(dy,dx)
    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
    trajectory[i]['qx'] = qx
    trajectory[i]['qy'] = qy
    trajectory[i]['qz'] = qz
    trajectory[i]['qw'] = qw

# Write to YAML file
with open(name, 'w') as f:
    yaml.dump({'trajectory': trajectory}, f)
