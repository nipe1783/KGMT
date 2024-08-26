import csv

def multiply_csv_values(input_csv, output_csv):
    # Read the input CSV file
    with open(input_csv, 'r') as infile:
        reader = csv.reader(infile)
        data = []
        for row in reader:
            # Multiply each value by 100 and convert to float
            new_row = [float(value) * 100 for value in row]
            data.append(new_row)
    
    # Write the modified data to the output CSV file
    with open(output_csv, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerows(data)

# Example usage
input_csv = '/home/nicolas/dev/research/KGMT/include/config/obstacles/trees/obstacles.csv'  # Replace with your input CSV file path
output_csv = 'obstacles.csv'  # Replace with your desired output CSV file path
multiply_csv_values(input_csv, output_csv)
