import numpy as np
import matplotlib.pyplot as plt

def Closest_Two_Lines(pt, lines):
    
    nearest_dists = []
    for line in lines:
        dist_tmp = np.linalg.norm(line[0] - pt)
        nearest_dists.append(dist_tmp)
    nearest_dists = np.array(nearest_dists)
    i_others = [*range(0, nearest_dists.shape[0])]
    i1 = np.where(np.min(nearest_dists) == nearest_dists)[0][0]
    i_others.remove(i1)
    i2 = np.where(np.min(nearest_dists[i_others]) == nearest_dists)[0]
    i2 = i2[i2!=i1][0]

    closest = [lines[i1], lines[i2]]
    return closest

def main():
    # example lines
    LINE1 = np.array([
        [0, 2],
        [3, 2]
    ])
    LINE2 = np.array([
        [0, 0],
        [4, 0]
    ])
    LINE3 = np.array([
        [3, 2],
        [3, 4]
    ])
    LINE4 = np.array([
        [4, 0],
        [4, 5]
    ])

    vehicle_pose = np.array([
        0, # problems when in certain positions like 2,1
        1,
        0,
        0
    ]).reshape(4,1)

    closest = Closest_Two_Lines([vehicle_pose[0][0], vehicle_pose[1][0]],
                                [LINE1, LINE2, LINE3, LINE4])

    # plotting
    plt.figure(1)
    plt.plot(LINE1[:,0], LINE1[:,1], "r-", LINE3[:,0], LINE3[:,1], "r-")
    plt.plot(LINE2[:,0], LINE2[:,1], "r-", LINE4[:,0], LINE4[:,1], "r-")
    plt.plot(vehicle_pose[0][0], vehicle_pose[1][0], 'o')
    for line in closest:
        plt.plot(line[:,0], line[:,1], 'g-')
    plt.show()

if __name__ == "__main__":
    main()