import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
# Grid creation routine
from grid import create_grid
# Voxel map creation routine
from voxmap import create_voxmap
# 2D A* planning routine (can you convert to 3D??)
from planning3D import a_star
# Random sampling routine
from sampling import Sampler
from planning_utils import heuristic, prune_path


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
myVoxelSize = 5
voxmap = create_voxmap(data, voxel_size=myVoxelSize )

north_offset = int(np.abs(np.min(data[:, 0])))
east_offset = int(np.abs(np.min(data[:, 1])))
# maximum altitude
alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))

print("North offset = {0}, east offset = {1}, altitude {2}".format(north_offset, east_offset, alt_max))
print("Time is: ", time.clock())
vox_start = (north_offset, east_offset, 10)  # center of map? 305,345
vox_start = list((np.array(vox_start)//myVoxelSize).astype(int))
vox_goal = (461, 510, 10)
vox_goal = list((np.array(vox_goal)//myVoxelSize).astype(int))

print('Local Start and Goal: ', vox_start, vox_goal)
path, _ = a_star(voxmap, heuristic, vox_start, vox_goal)
print("Time is after astar: ", time.clock())
pruned_path = prune_path(path)
print(len(pruned_path))
print("Time is after pruning: ", time.clock())




print(voxmap.shape)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxmap, edgecolor='k')
ax.set_xlim(voxmap.shape[0], 0)
ax.set_ylim(0, voxmap.shape[1])
# add 100 to the height so the buildings aren't so tall
ax.set_zlim(0, voxmap.shape[2]+70)

for p in pruned_path :
    plt.plot(p[1], p[0], P[2], 'yo')


plt.xlabel('North')
plt.ylabel('East')

plt.show()



