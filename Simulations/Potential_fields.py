import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from findClosestLine import get_lanes_from_file

import time

################################################################################

def get_attr_force(pt_agent, pt_goal, coeff):
    return coeff*np.linalg.norm(pt_agent - pt_goal)

def get_obs_rep_force(pt_agent, pt_obs, p_max, length):
    g = (pt_obs[0] - length/2.0 - pt_agent[0]) + \
        np.abs(pt_obs[0] - length/2.0 - pt_agent[0]) + \
        (pt_agent[0] - pt_obs[0] - length/2.0 + 1) + \
        np.abs(pt_agent[0] - pt_obs[0] - length/2.0 + 1) + \
        (pt_obs[1] - length/2.0 - pt_agent[1]) + \
        np.abs(pt_obs[1] - length/2.0 - pt_agent[1]) + \
        (pt_agent[1] - pt_obs[1] - length/2.0 + 1) + \
        np.abs(pt_agent[1] - pt_obs[1] - length/2.0 + 1)

    return p_max / (1 + g)

################################################################################

def main():
    start = time.time()

    P_AGENT_START = np.array([-25, -20])
    P_GOAL = np.array([25, 25])
    P_OBS = []
    OBSTACLE_SIZE = 1*12
    MAX_P = 50
    ATTR_COEFF = 5
    SCAN_ZONE = 20

    all_lines = get_lanes_from_file("world_lines.txt")
    for line in all_lines:
        P_OBS.append(line[0])
    P_OBS = np.array(P_OBS)

    X, Y = np.meshgrid(np.linspace(-6*12, 6*12, 100), np.linspace(-6*12, 6*12, 100))

    test_F = np.zeros(X.shape)
    min_F = 1000
    i_min = 0
    j_min = 0

    goals_gen = []
    P_AGENT = P_AGENT_START
    while np.linalg.norm(P_GOAL - P_AGENT) > 3:
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                pt = np.array([X[i,j], Y[i,j]])
                if np.linalg.norm(pt - P_AGENT) < SCAN_ZONE:
                    attr = get_attr_force(pt, P_GOAL, ATTR_COEFF)
                    rep = 0
                    for obs in P_OBS:
                        rep += get_obs_rep_force(pt, obs, MAX_P, OBSTACLE_SIZE)

                    test_F[i, j] = attr + rep
                    if test_F[i, j] < min_F:
                        i_min = i
                        j_min = j
                        min_F = test_F[i, j]
                
        P_AGENT = np.array([X[i_min, j_min], Y[i_min, j_min]])
        goals_gen.append(P_AGENT)

    print(time.time() - start)

    print("Final Dest: {}".format(P_AGENT))
    print("Sequential goals:")
    for pt in goals_gen:
        print(pt)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    surf = ax.plot_surface(X, Y, test_F, cmap=cm.coolwarm)
    for pt in goals_gen:
        ax.scatter(pt[0], pt[1], 3000, color='m')
    ax.scatter(P_AGENT_START[0], P_AGENT_START[1], 3000, color='b')
    ax.view_init(elev=90.0, azim=-90)
    fig.colorbar(surf)
    plt.show()

if __name__ == "__main__":
    main()