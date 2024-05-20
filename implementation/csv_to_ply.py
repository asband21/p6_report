import pandas as pd
from plyfile import PlyData, PlyElement
import numpy as np
import argparse

def csv_to_ply(input_csv, output_ply):
    # Read the CSV file without headers
    points = np.genfromtxt(input_csv, delimiter=',')
    # Create a PlyElement
    vertices = np.array([tuple(point) for point in points],
                        dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    ply_element = PlyElement.describe(vertices, 'vertex')
    
    # Write to a PLY file
    PlyData([ply_element]).write(output_ply)
    print(f"PLY file saved as {output_ply}")

def main():
    parser = argparse.ArgumentParser(description='Convert CSV file to PLY format.')
    parser.add_argument('input_csv', type=str, help='Path to the input CSV file')
    parser.add_argument('output_ply', type=str, help='Path to the output PLY file')
    
    args = parser.parse_args()
    csv_to_ply(args.input_csv, args.output_ply)

if __name__ == "__main__":
    main()
