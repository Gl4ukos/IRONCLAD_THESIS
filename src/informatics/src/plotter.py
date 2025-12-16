import statistics
import csv
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
from time import sleep


dimension_modifier = 1/2 # to convert to real size


class POSE_SEQUENCE:
    def __init__(self):
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.ox_list = []
        self.oy_list = []
        self.oz_list = []
        self.ow_list = []
        self.the_numbers = np.zeros((1,1))

PLAN = POSE_SEQUENCE()
PURE_PURSUIT_TRAJECTORY = POSE_SEQUENCE()
LATERAL_TRAJECTORY = POSE_SEQUENCE()
MPC_TRAJECTORY = POSE_SEQUENCE()




def load_pose_sequence_from_csv(filename):
    NEW_POSE_SEQUENCE = POSE_SEQUENCE()
    file = open(filename,mode='r')
    data = csv.reader(file)
    sequence_len = 0
    for lines in data:
        if len(lines) != 7:
            print(lines,"Bad input, skip")
            continue
        
        NEW_POSE_SEQUENCE.x_list.append(round(float(lines[0]),3))
        NEW_POSE_SEQUENCE.y_list.append(round(float(lines[1]),3))
        NEW_POSE_SEQUENCE.z_list.append(round(float(lines[2]),3))
        NEW_POSE_SEQUENCE.ox_list.append(round(float(lines[3]),3))
        NEW_POSE_SEQUENCE.oy_list.append(round(float(lines[4]),3))
        NEW_POSE_SEQUENCE.oz_list.append(round(float(lines[5]),3))
        NEW_POSE_SEQUENCE.ow_list.append(round(float(lines[6]),3))
        sequence_len +=1

    NEW_POSE_SEQUENCE.the_numbers = np.zeros((sequence_len, 7))
    for i in range (sequence_len):
        NEW_POSE_SEQUENCE.the_numbers[i][0] = NEW_POSE_SEQUENCE.x_list[i]
        NEW_POSE_SEQUENCE.the_numbers[i][1] = NEW_POSE_SEQUENCE.y_list[i]
        NEW_POSE_SEQUENCE.the_numbers[i][2] = NEW_POSE_SEQUENCE.z_list[i]
        NEW_POSE_SEQUENCE.the_numbers[i][3] = NEW_POSE_SEQUENCE.ox_list[i]
        NEW_POSE_SEQUENCE.the_numbers[i][4] = NEW_POSE_SEQUENCE.oy_list[i]
        NEW_POSE_SEQUENCE.the_numbers[i][5] = NEW_POSE_SEQUENCE.oz_list[i]
        NEW_POSE_SEQUENCE.the_numbers[i][6] = NEW_POSE_SEQUENCE.ow_list[i]
    return NEW_POSE_SEQUENCE



def analytical_deviation_calculator(plan: POSE_SEQUENCE, trajectory: POSE_SEQUENCE):
    last_index = 0
    deviation = []
    sampled_trajectory_points = []
    
    for i in range(len(plan.x_list)):  # for each point in the given plan
        min_diff = math.inf
        best_index = last_index  # temporary best index
        
        for j in range(last_index, len(trajectory.x_list)):
            x_diff = plan.x_list[i] - trajectory.x_list[j]
            y_diff = plan.y_list[i] - trajectory.y_list[j]
            diff = x_diff**2 + y_diff**2  # squared distance
            
            if diff <= min_diff:
                min_diff = diff
                best_index = j
        
        # update last_index to avoid backtracking
        last_index = best_index
        
        # store deviation (sqrt if you want actual distance)
        deviation.append(math.sqrt(min_diff))  
        sampled_trajectory_points.append(
            (trajectory.x_list[best_index], trajectory.y_list[best_index])
        )

    return deviation, sampled_trajectory_points

    

controller_type = sys.argv[1]

    
PLAN = load_pose_sequence_from_csv("src/informatics/pose_sequences/PLAN.csv")
title = ""
anal_trajectory_filename = ""
means_filename = ""
controller_name = ""
if controller_type == "1":
    anal_trajectory_filename += "src/informatics/pose_sequences/PP_TRAJ_ANAL.csv"
    title = "Pure Pursuit trajectory deviation"
    means_filename = "src/informatics/data/PP_means.txt"
    controller_name = "Pure Pursuit"
elif controller_type == "2": 
    title = "Lateral trajectory deviation"
    anal_trajectory_filename += "src/informatics/pose_sequences/LAT_TRAJ_ANAL.csv"
    means_filename = "src/informatics/data/LAT_means.txt"
    controller_name = "Stanley"
elif controller_type == "3":
    title = "MPC trajectory deviation"
    anal_trajectory_filename += "src/informatics/pose_sequences/MPC_TRAJ_ANAL.csv"
    means_filename = "src/informatics/data/MPC_means.txt"
    controller_name = "MPC"
else:
    print("Cant load trajectory")
    print("controller type: ", controller_type)
    exit()

ANALYTICAL_TRAJECTORY = load_pose_sequence_from_csv(anal_trajectory_filename)
anal_traj_dev, sampled_traj = analytical_deviation_calculator(PLAN, ANALYTICAL_TRAJECTORY)




plt.figure()
scaled_anal_traj_dev = [x * (0.8) for x in anal_traj_dev]
plt.bar([i for i in range(len(anal_traj_dev))], scaled_anal_traj_dev, color = 'red')
plt.title(controller_name +" controller. Mean deviation " + str(str(round(statistics.mean(scaled_anal_traj_dev),3))) + "m")
plt.xlabel("Deviation (m)")
plt.ylabel("Target point indices")

#storing the mean of deviation for later
with open(means_filename, "a") as f:
    f.write(f"{statistics.mean(scaled_anal_traj_dev)}\n")


plt.figure()
scaled_plan_x_list = [x * (1) for x in PLAN.x_list]
flipped_plan_y_list = [x * (-1) for x in PLAN.y_list]
plt.plot(flipped_plan_y_list, scaled_plan_x_list, marker='o', linestyle='-', color='k', markersize = 0.1)


scaled_traj_x_list = [x * (1) for x in ANALYTICAL_TRAJECTORY.x_list]
flipped_traj_y_list = [x *(-1) for x in ANALYTICAL_TRAJECTORY.y_list]
plt.plot(flipped_traj_y_list, scaled_traj_x_list, marker='x', linestyle = '-', color = 'r', markersize=0)
plt.title("plan & trajectory")
plt.axis('equal') 
plt.xlabel("Y coordinates")
plt.ylabel("X coordinates")


plt.figure()
plt.plot(PLAN.y_list, PLAN.x_list, marker='o', linestyle='-', color="gray", markersize = 0.1)
x_samples, y_samples = zip(*sampled_traj)
plt.scatter(y_samples, x_samples, marker="x", color="r", s=15, label="Approximations")
plt.scatter(PLAN.y_list, PLAN.x_list, marker="x", color="black", s=15, label="Targets")
for k, (xs, ys, xp, yp) in enumerate(zip(x_samples, y_samples, PLAN.x_list, PLAN.y_list)):
    if k == 0:
        plt.plot([ys, yp], [xs, xp], color="gray", linestyle="--", linewidth=1.2, label="Deviation")
    else:
        plt.plot([ys, yp], [xs, xp], color="gray", linestyle="--", linewidth=1.2, label="_nolegend_")

plt.title("Targets & Approximations & Deviations")
plt.axis('equal')

plt.legend()
plt.show()


