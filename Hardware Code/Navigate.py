from StepperControl import Stepper_28BYJ_48
from DCMotorControl import FIT_Motor
from time import sleep
from time import time
import numpy as np

def main():
    WHEELBASE = 9.75 # in
    LOOK_DIST = 12 # in
    ANGLE_RES = 100
    TOLERABLE_DIST = 10 # in
    STEPS_TO_DEG = 0.0202

    pose = np.array([
        0,
        0,
        np.pi/2,
        0
    ], dtype="float64").reshape(4,1)

    lin_spd = 0

    way_x = np.array([
        0*12, 2*12
    ], dtype="float64")
    way_x = way_x.reshape(way_x.shape[0], 1)

    way_y = np.array([
        2*12, 2*12
    ], dtype="float64")
    way_y = way_y.reshape(way_y.shape[0], 1)

    way_pts = np.hstack((way_x, way_y))
    tracker = 0

    dc_motor = FIT_Motor(16,12,10)
    stepper = Stepper_28BYJ_48(4,17,27,22)
    stepper._update()

    dist_i = 0
    step_err = 0

    raw_input("Press any key to start")

    while 1:
        time_start = time()
        angle_start = pose[2] - np.pi/2
        angle_end = pose[2] + np.pi/2

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
        dist_i += dist
        lin_spd = 0.1 * min_dist + 0.003 * dist_i - 1.5 * step_err
        
        if lin_spd > 20.8:
            lin_spd = 20.8
        if lin_spd < 0:
            lin_spd = 0

        alpha = angles[pt_i] - pose[2]
        e_ld = LOOK_DIST * np.sin(alpha)

        steering_angle = np.arctan2(2 * WHEELBASE * e_ld, LOOK_DIST**2)

        #saturate steering angle
        if steering_angle >= 27*np.pi/180.0:
            steering_angle = 27*np.pi/180.0
        if steering_angle <= -27*np.pi/180.0:
            steering_angle = -27*np.pi/180.0

        # control vehicle
        tot_time = time() - time_start
        dc_motor.setSpeed(lin_spd)
        stepper_angle = -1*stepper._direction * stepper.steps * STEPS_TO_DEG
        step_err = (steering_angle * 180.0 / np.pi) - stepper_angle
        if np.abs(step_err) > 0.001:
            if step_err < 0:
                stepper.changeDirection()
                stepper.step()
                stepper.changeDirection()
            else:
                stepper.step()
        stepper_angle = stepper_angle * np.pi / 180.0

        # calculate new state vectors
        speeds = np.array([
                    lin_spd * np.cos(pose[2]),
                    lin_spd * np.sin(pose[2]),
                    lin_spd * pose[3] / WHEELBASE
        ], dtype="float64").reshape(3,1)
        pose = np.array([
            speeds[0] * tot_time + pose[0],
            speeds[1] * tot_time + pose[1],
            speeds[2] * tot_time + pose[2],
            stepper_angle
        ], dtype="float64").reshape(4,1)

        # determine if tracking way_pt is close enough to change current way_pt
        if tracker == way_pts.shape[0] - 1:
            tol_dist = 1
        else:
            tol_dist = TOLERABLE_DIST
        dist_to_goal = np.linalg.norm(way_pts[tracker] - pose[0:2].reshape(1,2))
        if dist_to_goal < tol_dist and tracker < way_pts.shape[0]:
            tracker = tracker + 1

        if tracker >= way_pts.shape[0]:
            break

        #print([pose[0][0], pose[1][0]])
        print("{} {}".format(pose[0][0], pose[1][0]))
        print(stepper.steps)
        sleep(0.001)
    
    dc_motor.setStop()
    print("navigation complete")
    raw_input("press any key to stop")

if __name__ == "__main__":
    main()