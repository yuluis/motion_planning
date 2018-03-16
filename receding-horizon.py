import numpy as np
import matplotlib.pyplot as plt
import time
# Grid creation routine
from grid import create_grid
# Voxel map creation routine
from voxmap import create_voxmap
# 2D A* planning routine (can you convert to 3D??)
from planning import a_star
# Random sampling routine
from sampling import Sampler
from planning_utils import a_star, heuristic, prune_path


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

north_offset = int(np.abs(np.min(data[:, 0])))
east_offset = int(np.abs(np.min(data[:, 1])))
print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
print("Time is: ", time.clock())
grid_start = (north_offset, east_offset)  # center of map? 305,345
grid_goal = (461, 510)
print('Local Start and Goal: ', grid_start, grid_goal)
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
print("Time is after astar: ", time.clock())
pruned_path = prune_path(path)
print(len(pruned_path))
print("Time is after pruning: ", time.clock())

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

for p in pruned_path :
    plt.plot(p[1], p[0], 'yo')

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()




