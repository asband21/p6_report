import re
import numpy as np

# Step 1 - Centering
def extract_coordinates(line):
    # Regular expression pattern to match the coordinates inside [ and ]
    pattern = r'\[([-0-9.,\s]+)\]'
    match = re.search(pattern, line)
    if match:
        coordinates = match.group(1).split(',')
        # Convert coordinates from strings to floats
        coordinates = [float(coord) for coord in coordinates]
        return coordinates
    else:
        return None
def calculate_median_center(coordinates):
    # Extract XYZ coordinates from the array
    xyz_coordinates = np.array(coordinates)[:, :3]
    # Calculate median of each coordinate axis
    median_center = np.median(xyz_coordinates, axis=0)
    return median_center
def construct_point_below_center(median_center, distance_below=0.2):
    # Create a point 0.2 units below the median center
    point_below_center = median_center.copy()
    point_below_center[0] -= distance_below
    return point_below_center
# Step 2 - Angle vectors
def calculate_unit_vector(point1, point2):
    # Calculate vector from point1 to point2
    vector = np.array(point2) - np.array(point1)
    # Calculate length of the vector
    length = np.linalg.norm(vector)
    # Normalize the vector to length 1
    unit_vector = vector / length
    return unit_vector
def calculate_angle_between_vectors(vector1, vector2):
    # Calculate dot product of the two vectors
    dot_product = np.dot(vector1, vector2)
    # Calculate angle between vectors in radians
    angle_radians = np.arccos(dot_product)
    return angle_radians
def adjust_vector_direction(vector, center_point):
    # Calculate vector from center_point to vector
    center_to_vector = np.array(vector) - np.array(center_point)
    # Normalize the vector to length 1
    unit_center_to_vector = center_to_vector / np.linalg.norm(center_to_vector)
    return unit_center_to_vector
# Step 3 - Output data file
def replace_coordinates_with_angles(original_file_path, angled_coordinates_array):
    # Read the original file
    with open(original_file_path, 'r') as file:
        original_content = file.readlines()

    # Regular expression pattern to match the coordinates inside [ and ]
    pattern = r'\[([-0-9.,\s]+)\]'

    # Count the number of occurrences of coordinates in the file
    num_coordinates = sum(1 for line in original_content if re.search(pattern, line))

    # Ensure that there are enough elements in the angled_coordinates_array
    if num_coordinates > len(angled_coordinates_array):
        raise ValueError("Not enough angled coordinates provided to replace all occurrences in the file.")

    # Iterate through each line in the original content
    for i, line in enumerate(original_content):
        match = re.search(pattern, line)
        if match:
            # Extract coordinates from the line
            coordinates = match.group(1)
            # Replace coordinates with angled coordinates
            angled_coordinates = angled_coordinates_array.pop(0)
            # Convert angled_coordinates to a string
            angled_coordinates_str = ', '.join(map(str, angled_coordinates))
            # Replace the coordinates in the line with angled coordinates
            original_content[i] = line.replace(coordinates, angled_coordinates_str) + '\n'

    # Append "(angled)" to the filename
    angled_file_path = original_file_path.replace('.txt', '(angled).txt')

    # Write the updated content to the new file
    with open(angled_file_path, 'w') as file:
        file.writelines(original_content)

    return angled_file_path



def main():
    original_file_path = "01 path txt"

    # Read data from file
    with open(original_file_path, 'r') as file:
        data = file.readlines()

    # Initialize array to store coordinates
    coordinates_array = []

    # Loop through each line in the data
    for line in data:
        # Check if line contains movel() command
        if 'movel(pose_trans' in line:
            # Extract coordinates from the line
            coordinates = extract_coordinates(line)
            if coordinates:
                # Append coordinates to array
                coordinates_array.append(coordinates)

    # Calculate median center of XYZ coordinates
    median_center = calculate_median_center(coordinates_array)

    # Construct a point below the median center
    point_below_center = construct_point_below_center(median_center)

    # Calculate unit vectors and angles
    for i, coordinates in enumerate(coordinates_array):
        # Extract XYZ coordinates from the array
        xyz_coordinates = coordinates[:3]

        # Calculate unit vector from point below center to XYZ coordinates
        unit_vector = calculate_unit_vector(point_below_center, xyz_coordinates)

        # Adjust vector direction to point away from center point
        unit_vector_adjusted = adjust_vector_direction(unit_vector, point_below_center)

        # Calculate angle between vectors in radians
        angle_radians = calculate_angle_between_vectors(unit_vector, unit_vector_adjusted)

        # Round angle to 6 decimal places
        angle_rounded = round(angle_radians, 6)

        # Update last three indices of the coordinates array with adjusted vector and rounded angle
        coordinates_array[i][-3] = angle_rounded

    # Print the adjusted coordinates array
    print("Angled coordinates array:")
    for coordinates in coordinates_array:
        print(coordinates)

    # Replace coordinates with angles and save to new file
    angled_file_path = replace_coordinates_with_angles(original_file_path, coordinates_array)

    print(f"New file with angled coordinates saved as: {angled_file_path}")


if __name__ == "__main__":
    main()







