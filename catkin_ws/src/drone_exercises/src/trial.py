#!/usr/bin/env python

import rospy
import time
from drone_wrapper import DroneWrapper

def wait(secs):
	start = rospy.Time.now()
	while rospy.Time.now() - rospy.Duration(secs) < start:
		rospy.sleep(1)

drone = DroneWrapper(True)


print '***********************************Rectangle************************************'

print 'Calling Takeoff'
drone.takeoff(3)
print 'Forward'
drone.set_cmd_vel(vx = 1)
wait(5)
print 'Left'
drone.set_cmd_vel(vy = 1)
wait(5)
print 'Reverse'
drone.set_cmd_vel(vx = -1)
wait(5)
print 'Right'
drone.set_cmd_vel(vy = -1)
wait(5)
print 'Landing'
drone.land()
wait(5)

print '***********************************Circle************************************'

print 'Calling Takeoff'
drone.takeoff(3)
print 'Doing circle'
drone.set_cmd_vel(vx = 0.5, az = 0.5)
wait(100)
print 'Landing'
drone.land()
wait(5)
