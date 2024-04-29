import re


with open(r'toolpath_Ideal.txt', 'r') as file:
    lines = file.readlines()

#for line in lines:
 #   print(line.strip())


for index, line in enumerate(lines):
    # Check if the line contains the word "movel"
    if "movel" not in line:
        lines[index] = "false"
        #print("false")
    #else:
        #print("true")


for index, line in enumerate(lines):
# Define a regular expression pattern to match the coordinates within square brackets
    pattern = r"\[(.*?), (.*?), (.*?), (.*?), (.*?), (.*?)\]"

# Use the findall method of the re module to extract the coordinates
    matches = re.findall(pattern, lines[index])

    if matches:
        x_coord = float(matches[0][0])
        y_coord = float(matches[0][1])
        z_coord = float(matches[0][2])
        roll_coord = float(matches[0][3])
        pitch_coord = float(matches[0][4])
        yaw_coord = float(matches[0][5])
        #print("Coordinates:", x_coord, y_coord, z_coord)



array = [[],[],[],[],[],[]]
print(array)
#print(array[1][0])
array[0].append(1)
print(array)
print(array[0][0])
