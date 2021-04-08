import matplotlib.pyplot as plt
import numpy as np
from findClosestLine import Closest_Two_Lines

# trajectory generation given sets of lines
def Get_Traj(pose, dst, lines):
    mid_line = []
    pt = [pose[0][0], pose[1][0]]

    i = 0
    while(np.linalg.norm(np.array(pt) - np.array(dst)) > 0.5):
        closest_two = Closest_Two_Lines(pt, lines)
        line1 = closest_two[0]
        line2 = closest_two[1]
        mid_line.append((line1 + line2)/2)
        pt = mid_line[i][1]
        i += 1

    return mid_line

def main():
    # example lines
    LINE1 = np.array([
        [0, 2],
        [3, 2]
    ])
    LINE2 = np.array([
        [4, 0],
        [4, 5]
    ])
    LINE3 = np.array([
        [3, 2],
        [3, 4]
    ])
    LINE4 = np.array([
        [0, 0],
        [4, 0]
    ])
    LINE5 = np.array([
        [4, 5],
        [1, 5]
    ])
    LINE6 = np.array([
        [3, 4],
        [1, 4]
    ])
    DST_PT = [1, 4.5]

    vehicle_pose = np.array([
        0, # problems when in certain positions like 2,1
        1,
        0,
        0
    ]).reshape(4,1)

    all_lines = [LINE1, LINE2, LINE3, LINE4, LINE5, LINE6]

    # get midline given two sets of connected lines
    mid_line = Get_Traj(vehicle_pose, DST_PT, all_lines)

    # plotting
    plt.figure(1)
    for line in all_lines:
        plt.plot(line[:,0], line[:,1], "r-")
    plt.plot(vehicle_pose[0][0], vehicle_pose[1][0], 'b*')
    plt.plot(DST_PT[0], DST_PT[1], 'y*')
    for line in mid_line:
        plt.plot(line[:,0], line[:,1], "g-")
    plt.show()

if __name__ == "__main__":
    main()