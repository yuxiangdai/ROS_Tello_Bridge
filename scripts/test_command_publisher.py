#!/usr/bin/env python
# encoding: UTF-8
# date: March 4, 2020
# author: Yuxiang Dai
# description: Publish keystrokes to ROS topic

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    TOPIC_NAME = 'tello/command'
    key_pub = rospy.Publisher(TOPIC_NAME, String, queue_size=1)
    rospy.init_node("command_pub")
    rate = rospy.Rate(0.5)
    rate.sleep() # allow subscriber to link up

    cmds = ["takeoff", "right", "left", "land"]

    i = 0
    print "Publishing commands"
    while i < len(cmds) and not rospy.is_shutdown():
        key_pub.publish(cmds[i])
        print(cmds[i])
        i += 1
        rate.sleep()

