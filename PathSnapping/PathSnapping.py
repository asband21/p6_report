import re

with open(r'toolpath_Ideal.txt', 'r') as file:
    lines = file.readlines()
array = [[], [], [], [], [], []]

for index, line in enumerate(lines):
    # Check if the line contains the word "movel"
    if "movel" not in line:
        lines[index] = "false"
for index, line in enumerate(lines):
# Define a regular expression pattern to match the coordinates within square brackets
    pattern = r"\[(.*?), (.*?), (.*?), (.*?), (.*?), (.*?)\]"
# Use the findall method of the re module to extract the coordinates
    matches = re.findall(pattern, lines[index])
    if matches:
        array[0].append(index)
        for i in range(5):
            array[i+1].append(float(matches[0][i]))
for j in range(len(array[0])):
    print("Coordinates:", array[1][j], array[2][j], array[3][j])

