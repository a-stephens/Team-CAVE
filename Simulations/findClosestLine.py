import numpy as np
import matplotlib.pyplot as plt

def Closest_Two_Lines(pt, lines, angle):
    # NEED TO FIX
    # PLZ
    # FIX

    # currently: finds first line correctly, but second line is ass
    # might need to try to shoot out lines and check for intersections
    # maybe: find first line, detect side of pose that first line is on, then
    #       ignore that side, detect second line from remaining closest lines
    nearest_dists = []
    for line in lines:
        dist_tmp = np.linalg.norm(line[0] - pt)
        nearest_dists.append(dist_tmp)

    heading_vector = np.array([np.cos(angle), np.sin(angle)])
    
    nearest_dists = np.array(nearest_dists)
    i_others = [*range(0, nearest_dists.shape[0])]
    
    i1 = np.where(np.min(nearest_dists) == nearest_dists)[0][0]

    orth = True
    while orth:
        vec1 = np.array([lines[i1][1][0] - lines[i1][0][0], lines[i1][1][1] - lines[i1][0][1]])
        lineMid = np.array([(lines[i1][1][0] + lines[i1][0][0])/2,(lines[i1][1][1] + lines[i1][0][1])/2])
        num = np.dot(vec1, heading_vector)
        den = np.linalg.norm(vec1)
        ang = np.arccos(num / den)
        angle_val1 = pt - lineMid

        if abs(angle) < np.pi/4 or abs(angle) > 5*np.pi/4:
            vec1dir = angle_val1[1] < 0
        else:
            vec1dir = angle_val1[0] < 0

        if np.abs(ang) < np.pi/6:
            i_others.remove(i1)
            orth = False
        else:
            i_others.remove(i1)
            i1 = np.where(np.min(nearest_dists[i_others]) == nearest_dists)[0][0]



    orth = True
    while orth:
        i2 = np.where(np.min(nearest_dists[i_others]) == nearest_dists)[0]
        i2 = i2[i2!=i1][0]

        vec2 = np.array([lines[i2][1][0] - lines[i2][0][0], lines[i2][1][1] - lines[i2][0][1]])
        lineMid2 = np.array([(lines[i2][1][0] + lines[i2][0][0]) / 2, (lines[i2][1][1] + lines[i2][0][1]) / 2])
        num = np.dot(heading_vector, vec2)
        den = np.linalg.norm(heading_vector) * np.linalg.norm(vec2)
        angle1 = np.arccos(num / den)
        angle_val = pt - lineMid2
        if abs(angle) < np.pi/4 or abs(angle) > 5*np.pi/4:
            vec2dir = angle_val[1] < 0
        else:
            vec2dir = angle_val[0] < 0

        if np.abs(angle1) < np.pi/6:
            if not vec2dir and vec1dir or not vec1dir and vec2dir:
                orth = False
            else:
                i_others.remove(i2)
        else:
            i_others.remove(i2)

    closest = [lines[i1], lines[i2]]
    return closest

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
        20, # problems when in certain positions like 2,1
        27,
        np.pi/2,
        0
    ]).reshape(4,1)

    all_lines = get_lanes_from_file("world_lines.txt")

    closest = Closest_Two_Lines([vehicle_pose[0][0], vehicle_pose[1][0]],
                                all_lines, vehicle_pose[2][0])

    # plotting
    plt.figure(1)
    for line in all_lines:
        plt.plot(line[:,0], line[:,1], "r-")
    plt.plot(vehicle_pose[0][0], vehicle_pose[1][0], 'o')
    for line in closest:
        plt.plot(line[:,0], line[:,1], 'g-')
    plt.show()

if __name__ == "__main__":
    main()