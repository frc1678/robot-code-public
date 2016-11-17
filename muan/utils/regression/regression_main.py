import regression
import sys
import numpy as np

assert len(sys.argv) == 3, "Arguments must be a csv file and the number of states."
data = np.loadtxt(open(sys.argv[1],"rb"),delimiter=",",skiprows=1)
numstates = int(sys.argv[2])
A, B, Q_noise = regression.state_space_regression(data, numstates)
np.savez("system_matrices", A=A, B=B, Q_noise=Q_noise)
