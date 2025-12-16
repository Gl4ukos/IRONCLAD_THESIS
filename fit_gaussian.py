import numpy as np
from scipy.stats import norm

# Path to your txt file
file_path = "metrics.txt"

# Load data (one value per line)
data = np.loadtxt(file_path)

# Fit Gaussian distribution
mu, sigma = norm.fit(data)

# Print results
print(f"Mean (μ): {mu}")
print(f"Standard deviation (σ): {sigma}")
