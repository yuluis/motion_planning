import numpy as np
import matplotlib.pyplot as plt


def distance (x,y):
    return np.sqrt((x[0] - y[0])^2 + (x[1]-y[1])^2)

def attraction(position, goal, alpha):
    # TODO: implement attraction force
    if goal[0] != position[0] :
        angle = np.arctan((goal[1]-position[1])/(goal[0]-position[0]))
        x = alpha * distance(position, goal) * np.sin(angle)
        y = alpha * distance(position, goal) * np.cos(angle)
        return (x,y)
    else:
        return(0,0)


def repulsion(position, obstacle, beta, q_max):
    # TODO: implement replusion force
    dist = distance(position, obstacle)
    if goal[0] != obstacle[0] :
        angle = np.arctan((obstacle[1]-position[1])/(obstacle[0]-position[0]))
        x = 0
        y = 0
        if dist < q_max :
            x = beta * (1/q_max - 1/dist) * 1/ dist/dist * np.sin(angle)
            y = beta * (1/q_max - 1/dist) * 1 / dist/dist * np.cos(angle)

        return (x, y)
    else:
        return (0,0)

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
                forcex, forcey = attraction([i, j], goal, alpha)

                for (oi, oj) in zip(obs_i, obs_j):
                    if np.linalg.norm(np.array([i, j]) - np.array([oi, oj])) < q_max:
                        # add replusion force
                        repulsex,repulsey = repulsion([i, j], [oi, oj], beta, q_max)
                        forcex = forcex +repulsex
                        forcey = forcey +repulsey


                x.append(i)
                y.append(j)
                fx.append(forcex)
                fy.append(forcey)

    return x, y, fx, fy

# generate environment
grid = np.zeros((30, 30))
grid[10:15,10:15] = 1.0
grid[17:25,10:17] = 1.0

goal  = [2, 25]

# constants
alpha = 1.0
beta = 2.0
q_max = 5.0

x, y, fx, fy = potential_field(grid, goal, alpha, beta, q_max)

plt.imshow(grid, cmap = 'Greys', origin='lower')
plt.plot(goal[1], goal[0], 'ro')
plt.quiver(y, x, fy, fx)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

