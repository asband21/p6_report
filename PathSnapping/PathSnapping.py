import re


with open(r'toolpath_Ideal.txt', 'r') as file:
    lines = file.readlines()

#for line in lines:
 #   print(line.strip())


for index, line in enumerate(lines):
    # Check if the line contains the word "movel"
    if "movel" in line:
        print("true")
    else:
        lines[index]="false"

for line in lines:
    print(line.strip())

for line in lines:
    print(line.strip())

# Define the string containing the coordinates
string = "movel(pose_trans(ref_frame,p[0.010000, 0.182525, 0.021866, 1.688324, -0.808544, -1.688324]),accel_mss,speed_ms,0,0.000)"
for index, line in enumerate(lines):
# Define a regular expression pattern to match the coordinates within square brackets
    pattern = r"\[(.*?), (.*?), (.*?), (.*?), (.*?), (.*?)\]"

# Use the findall method of the re module to extract the coordinates
    matches = re.findall(pattern, lines[index])
    print(matches)
# Extract the first three coordinates

    if matches:
        x_coord = float(matches[0][0])
        y_coord = float(matches[0][1])
        z_coord = float(matches[0][2])
        roll_coord = float(matches[0][3])
        pitch_coord = float(matches[0][4])
        yaw_coord = float(matches[0][5])
        print("Coordinates:", x_coord, y_coord, z_coord, roll_coord, pitch_coord, yaw_coord)
        print(string)
    else:
        print("No coordinates found.")