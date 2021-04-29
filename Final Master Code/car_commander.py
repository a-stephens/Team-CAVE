#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray, Int64MultiArray, Bool
import numpy as np

def ids_cb(data):
    global ids
    ids = data.data
    print(ids)

def light_cb(data):
    global poses
    global light_status
    light_status = data.data

def get_pose0(data):
    global poses
    print(data.data)
    poses[0][0] = data.data[0]
    poses[0][1] = data.data[1]
    poses[0][2] = data.data[2]
    
def get_pose22(data):
    global poses
    poses[1][0]=data.data[0]
    poses[1][1]=data.data[1]
    poses[1][2]=data.data[2]

def car_commanding():
    global ids
    global poses
    global light_status

    LEFT_LINE = -2*12
    RIGHT_LINE = 2*12
    TOP_LINE = 2*12
    BOTTOM_LINE = -2*12

    light_status = False
    poses = [[0,0,0], [0,0,0]]
    publishers = []
    ids = []

    print("Ready to publish now")
    pub = rospy.Publisher("traffic_stop0", Bool, queue_size=1)
    publishers.append(pub)
    pub = rospy.Publisher("traffic_stop22", Bool, queue_size=1)
    publishers.append(pub)
    while True:
        try:
            for i, pose in enumerate(poses):
                msg = Bool()
                msg.data = False
                if light_status == True:
		    print("Light is on")
                    if pose[0] < LEFT_LINE and np.abs(pose[2]) < 30:
                        msg.data = True
                    elif pose[0] > RIGHT_LINE and np.abs(pose[2] - 180) < 30:
                        msg.data = True
                if light_status == False:
		    print("Light is off")
                    if pose[1] < BOTTOM_LINE and pose[2] > 0:
                        msg.data = True
                    elif pose[1] > TOP_LINE and pose[2] < 0:
                        msg.data = True
                publishers[i].publish(msg)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    rospy.init_node("traffic_interpreter", disable_signals=True)
    rospy.Subscriber("traffic_status", Bool, light_cb)
    rospy.Subscriber("ids", Int64MultiArray, ids_cb)
    rospy.Subscriber("pose0", Float64MultiArray, get_pose0)
    rospy.Subscriber("pose22", Float64MultiArray, get_pose22)
    car_commanding()
