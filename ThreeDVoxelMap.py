import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from voxmap import create_voxmap
import time

plt.rcParams['figure.figsize'] = 12,12

print("Time is: ", time.clock())

# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=3)
print(data)

voxmap = create_voxmap(data, 20)
print(voxmap.shape)
print("Time is after create_voxmap: ", time.clock())

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxmap, edgecolor='k')
ax.set_xlim(voxmap.shape[0], 0)
ax.set_ylim(0, voxmap.shape[1])
# add a bit to z-axis height for visualization
ax.set_zlim(0, voxmap.shape[2]+20)

plt.xlabel('North')
plt.ylabel('East')

plt.show()



