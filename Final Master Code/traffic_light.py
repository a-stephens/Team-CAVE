#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool

def publisher_light_status():
    pub = rospy.Publisher("traffic_status", Bool, queue_size=1)
    stat_pub = rospy.Publisher("status", String, queue_size=1)
    traffic_state = False
    while True:
        try:
            traffic_state = not traffic_state
            rospy.loginfo(traffic_state)
            if traffic_state == True:
                pub.publish(traffic_state)
                stat_pub.publish("Traffic Light Vert GO: ON, Traffic Light Horiz GO: OFF")
            else:
                pub.publish(traffic_state)
                stat_pub.publish("Traffic Light Vert GO: OFF, Traffic Light Horiz GO: ON")
            rospy.sleep(10)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    rospy.init_node("traffic_light", disable_signals=True)
    publisher_light_status()
