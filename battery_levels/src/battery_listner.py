#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(msg):
    rospy.loginfo("Battery Level: %.1f%%" % msg.data)
    if msg.data < 20.0:
        rospy.logwarn("Low Battery!")

def listener():
    rospy.init_node('battery_listener', anonymous=True)
    rospy.Subscriber("battery_percentage", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
