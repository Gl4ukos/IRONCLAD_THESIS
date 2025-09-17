import csv
import matplotlib.pyplot as plt
import numpy as np
import random



file_name = "src/informatics/pose_sequences/" + str(input("Give filename\n"))
noise = input("Give noise\n")

path_file = open(file_name, mode = 'w')
noisy_row = []





with open('src/informatics/pose_sequences/PLAN.csv', mode='r') as plan_file:
    plan_file = csv.reader(plan_file)
    for lines in plan_file:
        if len(lines) != 7:
            print("Bad input, skip\n")
            continue
        noisy_row = []
        for i in range(7):
            noisy_row.append(float(lines[i]) + random.uniform(-float(noise) , float(noise)))
        path_file.write(','.join(str(x) for x in noisy_row))
        path_file.write("\n")
