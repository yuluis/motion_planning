from enum import Enum
from queue import PriorityQueue
import numpy as np

# Quadroter assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (1, 0, 0, 1)
    SOUTH = (-1, 0, 0,  1)
    UP = (0, 0, 1, 1)
    DOWN = (0, 0, -1, 1)

    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])


def valid_actions(voxel, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m, p = voxel.shape[0] - 1, voxel.shape[1] - 1, voxel.shape[2] - 1
    x, y, z = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x + 1 > n or voxel[x + 1, y, z] == 1:
        valid_actions.remove(Action.NORTH)
    if x - 1 < 0 or voxel[x - 1, y, z] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or voxel[x, y - 1, z] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or voxel[x, y + 1, z] == 1:
        valid_actions.remove(Action.EAST)

    if (z - 1 < 0  or voxel[x,y,z-1] == 1) :  #NEU framing
        valid_actions.remove(Action.DOWN)
    if (z + 1 > p  or voxel[x,y,z+1] == 1) :  #NEU framing
        valid_actions.remove(Action.UP)

    return valid_actions


def a_star(voxel, heuristic_func, start, goal):
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
            for a in valid_actions(voxel, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1], current_node[2] + a.delta[2])
                new_cost = current_cost + a.cost + heuristic_func(next_node, goal)

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

    return path[::-1], path_cost
