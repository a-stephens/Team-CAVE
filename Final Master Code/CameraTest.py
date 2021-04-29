import cv2
import numpy as np
import os
import glob
import rospy
from std_msgs.msg import String

################################################################################

def publisher_test():
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    check, img = capture.read()

    rospy.init_node("camtest")
    pub = rospy.Publisher("pub_test", String, queue_size=1)

    while True:
        ret, frame = capture.read()
        cv2.imshow('pls wrk', frame)
        pub.publish("I'm working!")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.DestroyAllWindows()
    capture.release

if __name__ == "__main__":
    publisher_test()
