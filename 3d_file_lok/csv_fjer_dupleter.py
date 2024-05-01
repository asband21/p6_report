import numpy as np

data = np.loadtxt('pungsky_med_dobbler.csv', delimiter=',')
unique_data = np.unique(data, axis=0)
np.savetxt('pungsky.csv', unique_data, delimiter=',')
