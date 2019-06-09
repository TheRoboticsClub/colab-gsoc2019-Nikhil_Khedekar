#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import PoseStamped

s = 0
counter = 0

def cb(msg):
    global s, counter
    s += msg.pose.position.z
    counter += 1
    rospy.loginfo('Current Average: %f', s/counter)

rospy.init_node('av_calc')
rospy.Subscriber('mavros/local_position/pose', PoseStamped, cb)
while not rospy.is_shutdown():
    rospy.spin()