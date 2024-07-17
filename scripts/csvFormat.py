import numpy as np
import pandas as pd

# Path to the input CSV file
input_file_path = '/home/nicolas/dev/research/KGMT/include/config/obstacles/trees/obstacles.csv'
output_file_path = '/home/nicolas/dev/research/KGMT/include/config/obstacles/trees/test.csv'

# Read the CSV file into a pandas DataFrame
df = pd.read_csv(input_file_path, header=None)

# Split the DataFrame into x, y, z coordinate parts
x_coords = df.iloc[:, [0, 3]]
y_coords = df.iloc[:, [1, 4]]
z_coords = df.iloc[:, [2, 5]]

# Find the minimum values for x, y, z coordinates
min_x = x_coords.min().min()
min_y = y_coords.min().min()
min_z = z_coords.min().min()

# Adjust the coordinates to ensure the minimum values are at 0
df.iloc[:, [0, 3]] -= min_x
df.iloc[:, [1, 4]] -= min_y
df.iloc[:, [2, 5]] -= min_z

# Save the updated DataFrame to a new CSV file
df.to_csv(output_file_path, header=False, index=False)

print(f"Updated CSV file saved to {output_file_path}")
