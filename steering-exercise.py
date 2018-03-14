import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 6,6


def simulate(state, angle, v, dt):
    x = state[0]
    y = state[1]
    theta = state[2]

    nx = x + v * np.cos(theta) * dt
    ny = y + v * np.sin(theta) * dt
    ntheta = theta + v * np.tan(angle) * dt

    return [nx, ny, ntheta]

# limit the steering angle range
MAX_STEERING_ANGLE = np.deg2rad(30)
# km/h
MAX_VELOCITY = 1

def steer(x1, x2):
    # TODO: return the steering angle and velocity
    velocity = MAX_VELOCITY
    slope = (x2[1] - x1[1]) / (x2[0] - x1[0])

    angle_target = np.arctan(slope)
    angle = angle_target - x1[2] #adjusted by current heading

    #limit steer
    if angle_target > MAX_STEERING_ANGLE :
        angle_target = MAX_STEERING_ANGLE
    elif angle_target < -MAX_STEERING_ANGLE:
        angle_target = - MAX_STEERING_ANGLE



    return [angle, velocity]

# feel free to play around with these
dt = 0.1
total_time = 20

# initial state
start = [0, 0, 0]

# the goal location, feel free to change this ...
goal = [10, -15, 0]
states = [start]

for _ in np.arange(0, total_time, dt):
    current_state = states[-1]
    angle, velocity = steer(current_state, goal)
    state = simulate(current_state, angle, velocity, dt)
    states.append(state)

states = np.array(states)

plt.plot(states[:, 0], states[:, 1], color='blue')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()


