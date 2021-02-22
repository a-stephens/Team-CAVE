# Team CAVE steering control sim


import numpy as np
import matplotlib.pyplot as plt
################################################################################
WHEELBASE = 0.1
LOOK_DIST = 0.5
ANGLE_RES = 100
TOLERABLE_DIST = 0.5
################################################################################
pose = np.array([
    0,
    0,
    0,
    0
]).reshape(4,1)

lin_spd = 0.1

# these would come from master commands
way_x = np.array([
    3, -2, -1, 2
])
way_x = way_x.reshape(way_x.shape[0], 1)
way_y = np.array([
    0, 4, -4, -3
])
way_y = way_y.reshape(way_y.shape[0], 1)
way_pts = np.hstack((way_x, way_y))
tracker = 0

# just for plotting
act_path_x = np.array([])
act_path_y = np.array([])

#tracking_x = np.array([])
#tracking_y = np.array([])

while 1:
    # generate semicircle
    angle_start = pose[2] - np.pi/2
    angle_end   = pose[2] + np.pi/2

    angles = np.linspace(angle_start, angle_end, ANGLE_RES)

    x_circ = LOOK_DIST * np.cos(angles) + pose[0]
    y_circ = LOOK_DIST * np.sin(angles) + pose[1]

    circ = np.hstack((x_circ, y_circ))

    # find tracking point on circle compared to current tracking way_pt
    pt_i = 0
    min_dist = 1000000
    for cnt, c_pt in enumerate(circ):
        dist = np.linalg.norm(way_pts[tracker] - c_pt)
        if dist < min_dist:
            pt_i = cnt
            min_dist = dist
    #tracking_x = np.append(tracking_x, circ[pt_i,0])
    #tracking_y = np.append(tracking_y, circ[pt_i,1])

    alpha = angles[pt_i] - pose[2]
    e_ld = LOOK_DIST * np.sin(alpha)

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
    ]).reshape(3,1)
    pose = np.array([
        speeds[0] * 0.1 + pose[0],
        speeds[1] * 0.1 + pose[1],
        speeds[2] * 0.1 + pose[2],
        steering_angle
    ]).reshape(4,1)

    # determine if tracking way_pt is close enough to change current way_pt
    dist_to_goal = np.linalg.norm(way_pts[tracker] - pose[0:2].reshape(1,2))
    if dist_to_goal < TOLERABLE_DIST and tracker < way_pts.shape[0]:
        tracker = tracker + 1

    act_path_x = np.append(act_path_x, pose[0])
    act_path_y = np.append(act_path_y, pose[1])

    # out of way_pts to use
    if tracker >= way_pts.shape[0]:
        break

plt.plot(act_path_x, act_path_y, 'r*')
#plt.plot(tracking_x, tracking_y, 'y*')
plt.plot(way_x, way_y, 'b*')
plt.xlim([-5, 5])
plt.ylim([-5, 5])
plt.show()