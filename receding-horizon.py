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

import pkg_resources
import networkx as nx
from planning_utils import a_star_graph, distance
from grid import create_grid_and_edges


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

close_start_dist= 9999
close_goal_dist = 9999
start_ne = (25,  100)
#goal_ne = (750., 370.)
goal_ne = (750, 370)

close_start_pt = (9999,9999)
close_goal_pt = (9999,9999)

dist_s = 9999
dist_g = 9999

# Static drone altitude (metres)
drone_altitude = 5

# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude)
print(len(edges))

# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower', cmap='Greys')
G = nx.Graph()

def heuristic(n1, n2):
    # TODO: define a heuristic
    return distance(n1,n2)

for e in edges:
    p1 = (int(e[0][0]), int(e[0][1]))
    p2 = (int(e[1][0]), int(e[1][1]))
    dist = distance(p2, p1)
    G.add_edge(p1,p2,weight=dist)
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    dist_s  = distance(start_ne, p1)
    if dist_s < close_start_dist :
        close_start_dist = dist_s
        close_start_pt = p1

    dist_g  = distance(goal_ne, p1)
    if dist_g < close_goal_dist :
        close_goal_dist = dist_g
        close_goal_pt = p1

print("graph digested", time.clock())
path = a_star_graph(G, heuristic, close_start_pt, close_goal_pt)
print("A* graph found", time.clock())
#    shortestpath = nx.dijkstra_path(G, close_start_pt, close_goal_pt, 'weight')
#    print("dijkstra graph found", time.clock())
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.plot(close_start_pt[1], close_start_pt[0], 'yo')
plt.plot(close_goal_pt[1], close_goal_pt[0], 'yo')

# plot dijkstra path in cyan
#    for pt in shortestpath:
#        plt.plot(pt[1], pt[0], 'g^')

# plot path in yellow
for p in range(0, len(path[0]) - 1):
    pt = path[0][p].next_node()
    plt.plot(pt[1], pt[0], 'yo')

plt.xlabel('EAST')
plt.ylabel('NORTH')







#--voxel code below

flight_altitude = 3
safety_distance = 3
myVoxelSize = 1



voxmap = create_voxmap(data, voxel_size=myVoxelSize )
center = (355,425,10) #NEU framing, center of voxel, start (305,435,10)
vox_start = (355,425,10)
vox_start_offset = (vox_start[0]-center[0], vox_start[1]-center[1], vox_start[2]-center[2])
vox_goal = (395,465,30)
vox_goal_offset = (vox_goal[0]-center[0], vox_goal[1]-center[1], vox_goal[2]-center[2])


def create_voxsubmap(voxmap, center, voxsubdim) :
    northrange = (0, voxmap.shape[0])
    eastrange = (0, voxmap.shape[1])
    altituderange = (0, voxmap.shape[2])

    submap = [0,0,0,0,0,0]
    submap[0] = max (northrange[0], center[0] - voxsubdim[0])
    submap[1] = min (northrange[1], center[0] + voxsubdim[0])
    submap[2] = max (eastrange[0], center[1]- voxsubdim[1])
    submap[3] = min (eastrange[1], center[1] + voxsubdim[1])
    submap[4] = max (altituderange[0], center[2]- voxsubdim[2])
    submap[5] = min (altituderange[1], center[2] + voxsubdim[2])

    voxsubmap = voxmap[submap[0]:submap[1], submap[2]:submap[3],submap[4]:submap[5]]


    return voxsubmap

voxsubdim = (20, 20, 10) #half dimension north, east, up
voxsubmap = create_voxsubmap(voxmap, center, voxsubdim)

print("Time after voxsubmap is: ", time.clock())


print('Local Start and Goal: ', vox_start_offset, vox_goal_offset)
path, _ = a_star(voxsubmap, heuristic, vox_start_offset, vox_goal_offset) # intermediate goal within submap
print("Time is after astar: ", time.clock())
pruned_path = prune_path(path)
print(len(pruned_path))
print("Time is after pruning: ", time.clock())

print("voxsubmap shape", voxsubmap.shape)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxsubmap, edgecolor='k')
ax.set_xlim(voxsubmap.shape[0], 0)
ax.set_ylim(0, voxsubmap.shape[1])
# add 100 to the height so the buildings aren't so tall
ax.set_zlim(0, voxsubmap.shape[2])

for p in pruned_path :
    ax.scatter(p[1], p[0], p[2], 'yo')


plt.xlabel('North')
plt.ylabel('East')

plt.show()



