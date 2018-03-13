# OK this might look a little ugly but...
# Just waiting on eng to install these in the backend!

import pkg_resources
#pkg_resources.require("networkx==2.0")
import networkx as nx


import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA

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

def distance(x,y):
    return LA.norm(np.array(x)-np.array(y))
    #return np.sqrt((x[0]- y[0])^2 + (x[1] - y[1])^2)

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

G.neighbors()

plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.plot(close_start_pt[1], close_start_pt[0], 'yo')
plt.plot(close_goal_pt[1], close_goal_pt[0], 'yo')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

from queue import PriorityQueue


def heuristic(n1, n2):
    # TODO: define a heuristic
    return distance(n1,n2)

def valid_action(graph, current_node):
    # show what edges of current_node
    print("valid_action")
    return 0

###### THIS IS YOUR OLD GRID-BASED A* IMPLEMENTATION #######
###### With a few minor modifications it can work with graphs! ####
# TODO: modify A* to work with a graph
def a_star(graph, heuristic, start, goal):
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break

        else:
            for action in valid_actions(graph, current_node):
                # get the tuple representation
                da = action.delta
                cost = action.cost
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                new_cost = current_cost + cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, action)

    path = []
    path_cost = 0
    if found:

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])

    return path[::-1], path_cost


path = a_star(G, heuristic, close_start_pt, close_goal_pt)

print("done")