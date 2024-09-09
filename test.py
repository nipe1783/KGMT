import pandas as pd

def calculate_mean_from_csv(file_path):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)
    
    # Select only numerical columns
    numerical_df = df.select_dtypes(include='number')
    
    # Calculate the mean of all numerical values
    overall_mean = numerical_df.mean().mean()
    
    return overall_mean

# Example usage
file_path = '/home/nicolas/dev/research/KGMT/treeSize.csv'  # Replace with your CSV file path
mean_value = calculate_mean_from_csv(file_path)
print(f'The mean value of all numbers in the CSV file is: {mean_value}')
