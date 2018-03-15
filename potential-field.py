import numpy as np
import matplotlib.pyplot as plt


def distance (x,y):
    #if (x[0] - y[0]) ==0 or (x[1]-y[1]) ==0 :
    #    return 0
    return np.linalg.norm(np.array(x) - np.array(y))
    #np.sqrt(int((x[0] - y[0])^2 + (x[1]-y[1])^2))

def attraction(position, goal, alpha):
    # TODO: implement attraction force

    x = alpha * (goal[0]-position[0])
    y = alpha * (goal[1]-position[1])
    return (x,y)



def repulsion(position, obstacle, beta, q_max):
    # TODO: implement replusion force
    dist = distance(position, obstacle)
    distx = obstacle[0]-position[0]
    disty = obstacle[1]-position[1]

    x = 0
    y = 0
    if distx == 0 or disty == 0:
        return (0,0)

    if dist < q_max :
        x = beta * (1/q_max - 1/distx) * 1/ distx/distx
        y = beta * (1/q_max - 1/disty) * 1 / disty/disty

    return (x, y)


def potential_field(grid, goal, alpha, beta, q_max):
    x = []
    y = []
    fx = []
    fy = []

    obs_i, obs_j = np.where(grid == 1)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 0:
                forcex = 0
                forcey = 0
                # add attraction force
                forcex, forcey = attraction([i, j], goal, alpha)

                for (oi, oj) in zip(obs_i, obs_j):
                    if np.linalg.norm(np.array([i, j]) - np.array([oi, oj])) < q_max:
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

goal  = [15, 18]

# constants
alpha = 1.0
beta = 10.0
q_max = 5.0

x, y, fx, fy = potential_field(grid, goal, alpha, beta, q_max)

plt.imshow(grid, cmap = 'Greys', origin='lower')
plt.plot(goal[1], goal[0], 'ro')
plt.quiver(y, x, fy, fx)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

