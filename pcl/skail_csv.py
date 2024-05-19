import numpy as np
import argparse
import os

def process_csv(input_file, output_file, scale_factor):
    # Check if the input file exists
    if not os.path.isfile(input_file):
        print(f"Error: The file {input_file} does not exist.")
        return
    
    # Load data from CSV file
    data = np.loadtxt(input_file, delimiter=',')
    
    # Multiply all data by the scale factor
    modified_data = data * scale_factor
    
    # Save modified data to a new CSV file
    np.savetxt(output_file, modified_data, delimiter=',')
    
    print(f"Data from {input_file} has been multiplied by {scale_factor} and saved to {output_file}.")

def main():
    parser = argparse.ArgumentParser(description="Process a CSV file by scaling its data.")
    parser.add_argument('input_file', type=str, help='Input CSV file path')
    parser.add_argument('output_file', type=str, help='Output CSV file path')
    parser.add_argument('scale_factor', type=float, help='Scale factor to multiply the data by')

    args = parser.parse_args()
    
    process_csv(args.input_file, args.output_file, args.scale_factor)

if __name__ == "__main__":
    main()

