import csv
import matplotlib.pyplot as plt
import numpy as np
import math


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


def deviation_calculator(plan:POSE_SEQUENCE, trajectory:POSE_SEQUENCE):
    deviation = np.array(abs(plan.the_numbers - trajectory.the_numbers))
    
    position_deviation = []
    for i in range(len(plan.x_list)):
        position_deviation.append(round(math.sqrt(deviation[i][0]**2 + deviation[i][1]**2),3))
    
    return position_deviation
    


PLAN = load_pose_sequence_from_csv("src/informatics/pose_sequences/PLAN.csv")
PURE_PURSUIT_TRAJECTORY = load_pose_sequence_from_csv("src/informatics/pose_sequences/PP_TRAJ.csv")
LATERAL_TRAJECTORY = load_pose_sequence_from_csv("src/informatics/pose_sequences/LAT_TRAJ.csv")
MPC_TRAJECTORY = load_pose_sequence_from_csv("src/informatics/pose_sequences/MPC_TRAJ.csv")


pp_traj_dev = deviation_calculator(PLAN, PURE_PURSUIT_TRAJECTORY)

plt.figure()
plt.bar([i for i in range(len(pp_traj_dev))], pp_traj_dev)
plt.title("pp deviation")

plt.figure()
plt.plot(PLAN.y_list, PLAN.x_list, marker='o', linestyle='-', color='g')
plt.plot(PURE_PURSUIT_TRAJECTORY.y_list, PURE_PURSUIT_TRAJECTORY.x_list, marker="x", linestyle = '-', color = 'c' )
plt.title("plan and trajectory")

plt.show()
