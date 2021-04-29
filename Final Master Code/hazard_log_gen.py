#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os

class File_Log(object):
    def __init__(self):
        fpath = os.path.join(os.path.dirname(__file__), "log", "Hazard_log_{}.txt")
        self.file = open(fpath.format(rospy.get_rostime()), 'w')
        self._sub = rospy.Subscriber("status", String, self.callback)

    def callback(self, data):
        self.file.write("{}: {}\n".format(rospy.get_rostime(), data.data))

if __name__ == "__main__":
        rospy.init_node("logger")
        file_logger = File_Log()
        rospy.spin()