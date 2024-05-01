import numpy as np

# Step 1: Load data from a CSV file
data = np.loadtxt('data.csv', delimiter=',')

# Step 2: Remove duplicates
unique_data = np.unique(data, axis=0)

# Step 3: Save the unique data back to a CSV file
np.savetxt('cleaned_data.csv', unique_data, delimiter=',')

# Optionally, print the unique data to verify
print("Unique data:")
print(unique_data)

