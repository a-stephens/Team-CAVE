#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray, Bool
from StepperControl import Stepper_28BYJ_48
from DCMotorControl import FIT_Motor
from time import sleep
from time import time
import numpy as np
from gpiozero import DistanceSensor

def goal_listener_cb(data):
    global way_x
    global way_y
    global way_pts

    x_vals = []
    y_vals = []
    for i in range(len(data.data)/2):
        x_vals.append(data.data[2*i])
        y_vals.append(data.data[2*i+1])

    way_x = np.array(x_vals, dtype="float64")
    way_x = way_x.reshape(way_x.shape[0], 1)

    way_y = np.array(y_vals, dtype="float64")
    way_y = way_y.reshape(way_y.shape[0], 1)

    way_pts = np.hstack((way_x, way_y))

def nav_listener_cb(data):
    global pose
    pose = np.array([
        data.data[0],
        data.data[1],
        data.data[2] * np.pi/180.0,
        pose[3]
    ]).reshape(4,1)
#    rospy.loginfo("pose in cb: {} {} {}".format(pose[0], pose[1], pose[2]))

def traffic_listener_cb(data):
    global stop_cmd
    stop_cmd = data.data

def nav_talker():
    WHEELBASE = 9.75 # in
    LOOK_DIST = 12 # in
    ANGLE_RES = 100
    TOLERABLE_DIST = 24 # in
    STEPS_TO_DEG = 0.0202
    MAX_STEERING_ANGLE = 27 # deg
    ID = 22 # READ FROM FILE IDEALLY

    pub = rospy.Publisher("status", String, queue_size=1)
    rate = rospy.Rate(10)
    pub.publish("Initializing vehicle {}".format(ID))

    global pose
    pose = np.array([
        0,
        0,
        0,
        0
    ]).reshape(4,1)

    rospy.Subscriber("pose{}".format(ID), Float64MultiArray, nav_listener_cb)
    rospy.Subscriber("goals{}".format(ID), Float64MultiArray, goal_listener_cb)
    rospy.Subscriber("traffic_stop{}".format(ID), Bool, traffic_listener_cb)

    lin_spd = 0

    global way_x
    global way_y
    global way_pts
    way_x = np.array([], dtype="float64")
    way_x = way_x.reshape(way_x.shape[0], 1)

    way_y = np.array([], dtype="float64")
    way_y = way_y.reshape(way_y.shape[0], 1)
    way_pts = []

    global stop_cmd
    stop_cmd = False

    tracker = 0

    # create objects for all hardware
    dc_motor = FIT_Motor(16,12,10)
    stepper = Stepper_28BYJ_48(4,17,27,22)
    stepper._update()
    ultra1 = DistanceSensor(echo=19, trigger=26, max_distance=1, threshold_distance=0.2)
    ultra2 = DistanceSensor(echo=6, trigger=13, max_distance=1, threshold_distance=0.2)
    ultra3 = DistanceSensor(echo=5, trigger=21, max_distance=1, threshold_distance=0.2)

    ultrasonics = [ultra1, ultra2, ultra3]

    dist_i = 0
    step_err = 0

    tol_dist = TOLERABLE_DIST

    while len(way_pts) == 0:
	try:
            print("waiting for master")
            sleep(0.001)
	except KeyboardInterrupt:
	    break

    overall_time = time()

    pub.publish("Vehicle {} navigating.".format(ID))
    while True:
        time_start = time()
        
        rospy.loginfo("{}".format(step_err))

        vec1 = np.array([np.cos(pose[2][0]), np.sin(pose[2][0])])
        intermed_vec = np.array([pose[0][0], pose[1][0]])
        vec2 = way_pts[tracker] - intermed_vec

        dist_to_goal = np.linalg.norm(pose[0:2].reshape(1,2) - way_pts[tracker])

        ang2 = np.arctan2(vec2[1], vec2[0]) - np.arctan2(vec1[1], vec1[0])

        dist_i += (dist_to_goal - tol_dist)
        lin_spd = 1.35 * (dist_to_goal - tol_dist) + 0.003 * dist_i
        
        if lin_spd > 20.8:
            lin_spd = 20.8
        if lin_spd < 1:
            lin_spd = 0

        e_ld = LOOK_DIST * np.sin(ang2)

        steering_angle = np.arctan2(2 * WHEELBASE * e_ld, LOOK_DIST**2)

        #saturate steering angle
        if steering_angle >= MAX_STEERING_ANGLE*np.pi/180.0:
            steering_angle = MAX_STEERING_ANGLE*np.pi/180.0
        if steering_angle <= -MAX_STEERING_ANGLE*np.pi/180.0:
            steering_angle = -MAX_STEERING_ANGLE*np.pi/180.0

        # control vehicle

        time_in_loop = time()
        for i, sensor in enumerate(ultrasonics):
           while sensor.distance < 0.3:
               dc_motor.setStop()
               pub.publish("Object too close to sensor{} on vehicle {}, stopping.".format(i, ID))
               rate.sleep()

	while stop_cmd:
	    dc_motor.setStop()
	    pub.publish("Stopping due to traffic status on vehicle {}.".format(ID))
	    rate.sleep()

        time_loop_break = time()
            
        total_loop_time = time_loop_break - time_in_loop

        dc_motor.setSpeed(lin_spd)
        stepper_angle = stepper._direction * stepper.steps * STEPS_TO_DEG
        step_err = (steering_angle * 180.0 / np.pi) - stepper_angle
        while np.abs(step_err) >= 5:
            dc_motor.setStop()
            if step_err < 0:
                stepper.changeDirection()
                stepper.step()
                stepper.changeDirection()
            else:
                stepper.step()
            stepper_angle = stepper._direction * stepper.steps * STEPS_TO_DEG
            step_err = (steering_angle * 180.0 / np.pi) - stepper_angle
            sleep(0.001)
        if np.abs(step_err) < 5:
            if step_err < 0:
                stepper.changeDirection()
                stepper.step()
                stepper.changeDirection()
            else:
                stepper.step()
        # stepper_angle = stepper_angle * np.pi / 180.0

        # tot_time = time() - time_start - total_loop_time
        # calculate new state vectors
        # speeds = np.array([
        #             lin_spd * np.cos(pose[2]),
        #             lin_spd * np.sin(pose[2]),
        #             lin_spd * pose[3] / WHEELBASE
        # ], dtype="float64").reshape(3,1)
        # pose = np.array([
        #     speeds[0] * tot_time + pose[0],
        #     speeds[1] * tot_time + pose[1],
        #     speeds[2] * tot_time + pose[2],
        #     stepper_angle
        # ], dtype="float64").reshape(4,1)

        # determine if tracking way_pt is close enough to change current way_pt
        if tracker == way_pts.shape[0] - 1:
            tol_dist = 3
        else:
            tol_dist = TOLERABLE_DIST
        if dist_to_goal < tol_dist and tracker < way_pts.shape[0]:
            dist_i = 0
            tracker = tracker + 1

        if tracker >= way_pts.shape[0]:
            break

        if np.abs(stepper.steps) > 1500:
            print("STEPS EXCEEDED LIMIT, EMERGENCY BREAK")
            break
        sleep(0.001) # may not be needed (find out)
    
    pub.publish("Vehicle {} successfully navigated.".format(ID))
    pub.publish("Total time to navigate vehicle {}: {}".format(ID, time() - overall_time))

    dc_motor.setStop()
    while np.abs(stepper.steps) != 0:
        if stepper.steps < 0:
            stepper.changeDirection()
            stepper.step()
            stepper.changeDirection()
        else:
            stepper.step()
        sleep(0.001)

    while not rospy.is_shutdown():
        sleep(0.001)


if __name__ == "__main__":
    rospy.init_node("Navigator{}".format(22))
    nav_talker()
