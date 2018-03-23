import numpy as np

def create_grid_and_edges(data, drone_altitude):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Create an empty grid and points list
    grid = np.zeros((north_size, east_size))
    points = []

    # Populate obstacle cells in the grid with value of 1
    for i in range(data.shape[0]):
        # Extract data one line at a time
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # Only consider obstacle if it extends above flight altitude
        if alt + d_alt > drone_altitude:
            # Define obstacle extent based on data
            obstacle = [
                int(north - d_north - north_min),
                int(north + d_north - north_min),
                int(east - d_east - east_min),
                int(east + d_east - east_min),
            ]
            # Set obstacle cells to 1
            grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges



def create_voxsubmap(data, G, path, curr_position, voxsubdim, voxel_size=1):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.

    The `voxel_size` argument sets the resolution of the voxel map.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    # TODO: assume we are inside the world boundary
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((2*voxsubdim[0], 2*voxsubdim[1], 2*voxsubdim[2]), dtype=np.bool)

    # populate the voxmap centered on the curr_position and the input data.

    for i in range(data.shape[0]):
        # TODO: fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
        ]
        height = int(alt + d_alt) // voxel_size
        voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], 0:height] = True

    return voxmap, curr_position

#def create_voxsubmap(voxmap, center, voxsubdim) :
#    northrange = (0, voxmap.shape[0])
#    eastrange = (0, voxmap.shape[1])
#    altituderange = (0, voxmap.shape[2])

#    submap = [0,0,0,0,0,0]
#    submap[0] = max (northrange[0], center[0] - voxsubdim[0])
#    submap[1] = min (northrange[1], center[0] + voxsubdim[0])
#    submap[2] = max (eastrange[0], center[1]- voxsubdim[1])
#    submap[3] = min (eastrange[1], center[1] + voxsubdim[1])
#    submap[4] = max (altituderange[0], center[2]- voxsubdim[2])
#    submap[5] = min (altituderange[1], center[2] + voxsubdim[2])

#    voxsubmap = voxmap[submap[0]:submap[1], submap[2]:submap[3],submap[4]:submap[5]]


#    return voxsubmap


from enum import Enum
from queue import PriorityQueue
import numpy as np
from queue import PriorityQueue
import networkx as nx

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    print(north_min, north_max)

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    print(east_min, east_max)
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))
    print(north_size, east_size)
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min),
                int(north + d_north + safety_distance - north_min),
                int(east - d_east - safety_distance - east_min),
                int(east + d_east + safety_distance - east_min),
            ]
            grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1

    return grid


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTHEAST = (-1,1,np.sqrt(2))
    NORTHWEST = (-1,-1,np.sqrt(2))
    SOUTHEAST = (1,1,np.sqrt(2))
    SOUTHWEST = (1,-1,np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if x - 1 < 0 or y + 1 > m or grid[x - 1, y+1] == 1 :
        valid_actions.remove(Action.NORTHEAST)

    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)

    if x + 1 > n or y + 1 > m or grid[x + 1, y+1] == 1 :
        valid_actions.remove(Action.SOUTHEAST)

    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)

    return valid_actions


def a_star(grid, h, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0
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
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                new_cost = current_cost + a.cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def point(pt):
    return np.array([pt.action[0], pt.action[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=100):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path):

    i = 0
    while i < len(path[0]) - 2:
        p1 = point(path[0][i]) # point is np array
        p2 = point(path[0][i + 1])
        p3 = point(path[0][i + 2])

        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            path.remove(path[i + 1])
        else:
            i += 1
    return path

import numpy.linalg as LA

def distance(x,y):
    return LA.norm(np.array(x)-np.array(y))
    #return np.sqrt((x[0]- y[0])^2 + (x[1] - y[1])^2)

class Action_graph:
    """
    An action is represented by a 3 element tuple.

    The first 2 values is next_node. The third and final value
    is the cost of performing the action.
    """
    action = [0, 0,0]

    def __init__(self, action):

        self.action = action

    def cost(self):
        return self.action[0]


    def next_node(self):
        return (self.action[1], self.action[2])

def valid_actions_graph(graph, current_node):
    # show what edges of current_node
    action_list = []
    all_neighbors = list(nx.all_neighbors(graph, current_node))
    for neighbor in all_neighbors:
        # create list of Action_graphs
        cost = distance(current_node, neighbor)
        ag = Action_graph([cost, neighbor[0], neighbor[1]])
        action_list.append(ag)

    return(action_list)

###### THIS IS YOUR OLD GRID-BASED A* IMPLEMENTATION #######
###### With a few minor modifications it can work with graphs! ####
# TODO: modify A* to work with a graph
def a_star_graph(graph, heuristic, start, goal):
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    action_list2 = []

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
            action_list2 = valid_actions_graph(graph, current_node)
            for action in action_list2:
                cost = action.cost()
                next_node = action.next_node()
                new_cost = current_cost + cost + heuristic(next_node, goal)

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

