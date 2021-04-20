# Team CAVE steering control sim

import numpy as np
import matplotlib.pyplot as plt
import time
import msvcrt

################################################################################
WHEELBASE = 10 # in
LOOK_DIST = 12 # in
ANGLE_RES = 100
TOLERABLE_DIST = 10 # in
################################################################################

overall_start = time.time()

pose = np.array([
    -7*12,
    5*12,
    np.pi,
    0
], dtype="float64").reshape(4,1)

lin_spd = 0

# waypoint bounds: -144 to 144 in
# these would come from master commands
way_x = np.array([
    -8*12, 6*12, 4*12, 10*12, 0, 0
], dtype="float64")
way_x = way_x.reshape(way_x.shape[0], 1)
way_y = np.array([
    4*12, -2*12, -8*12, -5*12, 0, 8*12
], dtype="float64")
way_y = way_y.reshape(way_y.shape[0], 1)
way_pts = np.hstack((way_x, way_y))
tracker = 0

# just for plotting
act_path_x = np.array([])
act_path_y = np.array([])

tracking_x = np.array([])
tracking_y = np.array([])
itr = 0

dist_i = 0

while 1:
    # generate semicircle
    # angle_start = pose[2] - np.pi
    # angle_end   = pose[2] + np.pi

    # angles = np.linspace(angle_start, angle_end, ANGLE_RES)

    # x_circ = LOOK_DIST * np.cos(angles) + pose[0]
    # y_circ = LOOK_DIST * np.sin(angles) + pose[1]

    # circ = np.hstack((x_circ, y_circ))

    # find tracking point on circle compared to current tracking way_pt
    # pt_i = 0
    # min_dist = 1000000
    # for cnt, c_pt in enumerate(circ):
    #     dist = np.linalg.norm(way_pts[tracker] - c_pt)
    #     if dist < min_dist:
    #         pt_i = cnt
    #         min_dist = dist
    # tracking_x = np.append(tracking_x, circ[pt_i,0])
    # tracking_y = np.append(tracking_y, circ[pt_i,1])

    
    vec1 = np.array([np.cos(pose[2][0]), np.sin(pose[2][0])])
    intermed_vec = np.array([pose[0][0], pose[1][0]])
    vec2 = way_pts[tracker] - intermed_vec

    dist_to_goal = np.linalg.norm(pose[0:2].reshape(1,2) - way_pts[tracker])

    ang2 = np.arctan2(vec2[1], vec2[0]) - np.arctan2(vec1[1], vec1[0])


    dist_i += dist_to_goal
    lin_spd = 0.1 * dist_to_goal + 0.003 * dist_i
    
    if lin_spd > 20.8:
        lin_spd = 20.8

    e_ld = LOOK_DIST * np.sin(ang2)

    steering_angle = np.arctan2(2 * WHEELBASE * e_ld, LOOK_DIST**2)

    #saturate steering angle
    if steering_angle >= np.pi/4:
        steering_angle = np.pi/4
    if steering_angle <= -np.pi/4:
        steering_angle = -np.pi/4

    # calculate new state vectors
    speeds = np.array([
                lin_spd * np.cos(pose[2]),
                lin_spd * np.sin(pose[2]),
                lin_spd * pose[3] / WHEELBASE
    ], dtype="float64").reshape(3,1)
    pose = np.array([
        speeds[0] * 0.1 + pose[0],
        speeds[1] * 0.1 + pose[1],
        speeds[2] * 0.1 + pose[2],
        steering_angle
    ], dtype="float64").reshape(4,1)
    #print([pose[0][0], pose[1][0]])

    # determine if tracking way_pt is close enough to change current way_pt
    if tracker == way_pts.shape[0] - 1:
        tol_dist = 2.5
    else:
        tol_dist = TOLERABLE_DIST
    
    if dist_to_goal < tol_dist and tracker < way_pts.shape[0]:
        tracker = tracker + 1

    act_path_x = np.append(act_path_x, pose[0])
    act_path_y = np.append(act_path_y, pose[1])

    # out of way_pts to use
    if tracker >= way_pts.shape[0]:
        break

    itr += 1

    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'q':
            break

print(time.time() - overall_start)

plt.plot(act_path_x, act_path_y, 'r*')
#plt.plot(tracking_x, tracking_y, 'y*')
plt.plot(way_x, way_y, 'b*')
plt.xlim([-144, 144]) # represents a 12'x12' field
plt.ylim([-144, 144])
plt.show()