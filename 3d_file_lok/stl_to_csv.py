import csv
import numpy
from stl import mesh

ref_3d = mesh.Mesh.from_file('./gig_AAUTest97.STL')

with open('./gig_AAUTest97.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    #writer = csv.writer(file, fieldnames=["x", "y", "z"])
    for pol in ref_3d:
        #print(pol)
        writer.writerow([pol[0], pol[1], pol[2]])
        writer.writerow([pol[3], pol[4], pol[5]])
        writer.writerow([pol[6], pol[7], pol[8]])

