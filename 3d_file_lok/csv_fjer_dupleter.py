import numpy as np

data = np.loadtxt('./gig_AAUTest97.csv', delimiter=',')
unique_data = np.unique(data, axis=0)
np.savetxt('./gig_AAUTest97_ransed.csv', unique_data, delimiter=',')
