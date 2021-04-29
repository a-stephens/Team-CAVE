import numpy as np
import matplotlib.pyplot as plt
import math

def add_goal(X, Y, s, r, goal):
    delx = np.zeros_like(X)
    dely = np.zeros_like(Y)
    for i in range(X.shape[0]):
        for j in range(Y.shape[1]):
            d = math.hypot(goal[0] - X[i][j], goal[1] - Y[i][j])
            theta = np.arctan2(goal[1] - Y[i][j], goal[0] - X[i][j])
            if d < r:
                delx[i][j] = 0
                dely[i][j] = 0
            elif d > r + s:
                delx[i][j] = alpha * s * np.cos(theta)
                dely[i][j] = alpha * s * np.sin(theta)
            else:
                delx[i][j] = alpha * (d - r) * np.cos(theta)
                dely[i][j] = alpha * (d - r) * np.sin(theta)
    return delx, dely


def plot_graph(X, Y, delx, dely, ax, loc, r, i, color, start_goal=np.array([[0, 0]])):
    ax.quiver(X, Y, delx, dely)
    ax.add_patch(plt.Circle(loc, r, color=color))
    ax.set_title(f'Robot path with {i} obstacles ')
    return ax


def add_obstacle(X, Y, delx, dely, goal, obs, r):

    for i in range(X.shape[0]):
        for j in range(Y.shape[1]):

            d_goal = math.hypot(goal[0] - X[i][j], goal[1] - Y[i][j])
            d_obs = math.hypot(obs[0] - X[i][j], obs[1] - Y[i][j])
            theta_goal = np.arctan2(goal[1] - Y[i][j], goal[0] - X[i][j])
            theta_obs = np.arctan2(obs[1] - Y[i][j], obs[0] - X[i][j])

            if d_obs < r:
                delx[i][j] = -1 * np.sign(np.cos(theta_obs)) * 5 + 0
                dely[i][j] = -1 * np.sign(np.cos(theta_obs)) * 5 + 0
            elif d_obs > r + s:
                delx[i][j] += 0 - (alpha * s * np.cos(theta_goal))
                dely[i][j] += 0 - (alpha * s * np.sin(theta_goal))
            elif d_obs < r + s:
                delx[i][j] += -beta * (s + r - d_obs) * np.cos(theta_obs)
                dely[i][j] += -beta * (s + r - d_obs) * np.sin(theta_obs)
            if d_goal < r + s:
                if delx[i][j] != 0:
                    delx[i][j] += (alpha * (d_goal - r) * np.cos(theta_goal))
                    dely[i][j] += (alpha * (d_goal - r) * np.sin(theta_goal))
                else:

                    delx[i][j] = (alpha * (d_goal - r) * np.cos(theta_goal))
                    dely[i][j] = (alpha * (d_goal - r) * np.sin(theta_goal))

            if d_goal > r + s:
                if delx[i][j] != 0:
                    delx[i][j] += alpha * s * np.cos(theta_goal)
                    dely[i][j] += alpha * s * np.sin(theta_goal)
                else:

                    delx[i][j] = alpha * s * np.cos(theta_goal)
                    dely[i][j] = alpha * s * np.sin(theta_goal)
            if d_goal < r:
                delx[i][j] = 0
                dely[i][j] = 0

    return delx, dely, obs, r


def get_lanes_from_file(file_name):
    data = []
    with open(file_name) as f:
        line = f.readline()
        while line:
            data.append(line.strip().split())
            line = f.readline()
    all_lines = []
    for i in range(len(data)): # change this back when you get the new file
        current_line = np.zeros((2,2))
        current_line[0, 0] = float(data[i][0])
        current_line[0, 1] = float(data[i][1])
        current_line[1, 0] = float(data[i][2])
        current_line[1, 1] = float(data[i][3])
        all_lines.append(current_line)
    return all_lines




# Field Dimensions
x = np.arange(-72, 72, 1)
y = np.arange(-47, 47, 1)
X, Y = np.meshgrid(x, y)
# Radius of attractive force
r = 2
# Defining starting and goal point
start_pos = np.array([[0, 0]])
goal = [20, 20]
# Gains
alpha = 50
beta = 150
s = 5

all_lines = get_lanes_from_file("world_lines.txt")
mid_lines = []


fig, ax = plt.subplots(figsize=(10, 10))
delx, dely = add_goal(X, Y, s, r, goal)
plot_graph(X, Y, delx, dely, ax, goal, r, 0, 'b')
delx, dely, loc, r = add_obstacle(X, Y, delx, dely, goal,[10,8],1)
plot_graph(X, Y, delx, dely, ax, loc, r, 1, 'r')


ax.streamplot(X, Y, delx, dely, start_points=start_pos, linewidth=2)
plt.show()