#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import tty
import termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def send_command():
    pub = rospy.Publisher('teleop', String, queue_size=10)
    rospy.init_node('teleop_keyboard', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo("Use 'w', 'a', 's', 'd' to move. Press 'x' to stop.")
    
    while not rospy.is_shutdown():
        key = get_key()
        rospy.loginfo("Command received: {}".format(key))
        pub.publish(key)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass

