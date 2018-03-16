import numpy as np
import matplotlib.pyplot as plt

# Grid creation routine
from grid import create_grid
# Voxel map creation routine
from voxmap import create_voxmap
# 2D A* planning routine (can you convert to 3D??)
from planning import a_star
# Random sampling routine
from sampling import Sampler

#1. Load the colliders data
#Discretize your search space into a grid or graph
#Define a start and goal location
#Find a coarse 2D plan from start to goal
#Choose a location along that plan and discretize a local volume around that location (for example,
# you might try a 40x40 m area that is 10 m high discretized into 1m^3 voxels)
#Define your goal in the local volume to a a node or voxel at the edge of the volume in the direction of the
# next waypoint in your coarse global plan.
#Plan a path through your 3D grid or graph to that node or voxel at the edge of the local volume.


plt.rcParams['figure.figsize'] = 6,6

# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=3)
print(data)


flight_altitude = 3
safety_distance = 3
grid = create_grid(data, flight_altitude, safety_distance)

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()

