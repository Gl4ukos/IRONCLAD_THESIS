# Read numbers from file
filename = "MPC_means.txt"
with open(filename, "r") as f:
    numbers = [float(line.strip()) for line in f if line.strip()]

# Multiply each number by 2
numbers = [x * 2 for x in numbers]

# Write back to the file
with open(filename, "w") as f:
    for x in numbers:
        f.write(f"{x}\n")
