
from scipy.spatial import geometric_slerp
import matplotlib.pyplot as plt
import numpy as np
fig = plt.figure()
ax = fig.add_subplot(111)
start = np.array([0, 0])
end = np.array([10, 10])
t_vals = np.linspace(0, 1, 20)
result = geometric_slerp(start,end,t_vals)
ax.scatter(result[...,0], result[...,1], c='k')
ax.set_aspect('equal')

plt.show()
