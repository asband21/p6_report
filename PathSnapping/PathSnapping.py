import math
import re
from os.path import isfile

CoordIdeal = [[], [], [], [], [], []]
CoordOperator = [[], [], []]
minIndex = []
NewIndices = []
pattern = r"\[(.*?), (.*?), (.*?), (.*?), (.*?), (.*?)\]"

with open(r'toolpath_Ideal.txt', 'r') as file:
    ScriptIdeal = file.readlines()
with open(r'toolpath_Operator.txt', 'r') as file:
    ScriptOperator = file.readlines()

NewFile = "NewPath.txt"
k = 1
while isfile(NewFile):
    NewFile = "NewPath({}).txt".format(k)
    k += 1

for index, line in enumerate(ScriptIdeal):
    # Check if the line contains the word "movel"
    if "movel" in line:
        matches = re.findall(pattern, ScriptIdeal[index])
        if matches:
            CoordIdeal[0].append(index)
            for i in range(5):
                CoordIdeal[i + 1].append(float(matches[0][i]))

for index, line in enumerate(ScriptOperator):
    if "movel" in line:
        matches = re.findall(pattern, ScriptOperator[index])
        if matches:
            for i in range(3):
                CoordOperator[i].append(float(matches[0][i]))

for j in range(len(CoordOperator[0])):
    Distance = [[], []]
    for i in range(len(CoordIdeal[0])):
        dist = math.sqrt(math.pow(CoordOperator[0][j]-CoordIdeal[1][i], 2) +
                         math.pow(CoordOperator[1][j]-CoordIdeal[2][i], 2) +
                         math.pow(CoordOperator[2][j]-CoordIdeal[3][i], 2))

        Distance[1].append(dist)
    minIndex.append(Distance[1].index(min(Distance[1])))
    NewIndices.append(CoordIdeal[0][minIndex[j]])

f = open(NewFile, "w")

for i in range(len(ScriptIdeal)):
    if "movel" not in ScriptIdeal[i] and "movej" not in ScriptIdeal[i]:
        f.write(ScriptIdeal[i])
    elif "movel" in ScriptIdeal[i] and i in range(min(NewIndices), (max(NewIndices)+1)):
        #if i == NewIndices[0]: For adding a translation to first point if necessary
            #f.write(ScriptIdeal[i])
        f.write(ScriptIdeal[i])

