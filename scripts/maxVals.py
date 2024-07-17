import pandas as pd

# Path to the input CSV file
input_file_path = '/home/nicolas/dev/research/KGMT/include/config/obstacles/trees/obstacles.csv'  # Update with your actual file path

# Read the CSV file into a pandas DataFrame
df = pd.read_csv(input_file_path, header=None)

# Split the DataFrame into x, y, z coordinate parts
x_coords = df.iloc[:, [0, 3]]
y_coords = df.iloc[:, [1, 4]]
z_coords = df.iloc[:, [2, 5]]

# Find the maximum values for x, y, z coordinates
max_x = x_coords.max().max()
max_y = y_coords.max().max()
max_z = z_coords.max().max()

# Print the maximum values
print(f"Maximum x value: {max_x}")
print(f"Maximum y value: {max_y}")
print(f"Maximum z value: {max_z}")
