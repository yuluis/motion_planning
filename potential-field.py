import numpy as np
import matplotlib.pyplot as plt

def attraction(position, goal, alpha):
    # TODO: implement attraction force

    x = alpha * (position[0] - goal[0])
    y = alpha * (position[1] - goal[1])
    return [x,y]

def distance (x,y):
    return np.sqrt((x[0] - y[0])^2 + (x[1]-y[1])^2)

def repulsion(position, obstacle, beta, q_max):
    # TODO: implement replusion force
    dist = distance (position, obstacle)

    if (dist < q_max):
        repel = beta * (1/q_max - 1/dist) * 1/ dist^2
    return [0,0]


def potential_field(grid, goal, alpha, beta, q_max):
    x = []
    y = []
    fx = []
    fy = []

    obs_i, obs_j = np.where(grid == 1)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 0:

                # add attraction force
                force = attraction([i, j], goal, alpha)

                for (oi, oj) in zip(obs_i, obs_j):
                    if np.linalg.norm(np.array([i, j]) - np.array([oi, oj])) < q_max:
                        # add replusion force
                        force += repulsion([i, j], [oi, oj], beta, q_max)

                x.append(i)
                y.append(j)
                fx.append(force[0])
                fy.append(force[1])

    return x, y, fx, fy

# generate environment
grid = np.zeros((30, 30))
grid[10:15,10:15] = 1.0
grid[17:25,10:17] = 1.0

goal  = [5, 5]

# constants
alpha = 1.0
beta = 2.0
q_max = 10

x, y, fx, fy = potential_field(grid, goal, alpha, beta, q_max)

plt.imshow(grid, cmap = 'Greys', origin='lower')
plt.plot(goal[1], goal[0], 'ro')
plt.quiver(y, x, fy, fx)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

