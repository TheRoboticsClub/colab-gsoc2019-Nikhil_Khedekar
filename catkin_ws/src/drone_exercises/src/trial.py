#!/usr/bin/env python

import rospy
from drone_wrapper import DroneWrapper

def wait(secs):
	end = rospy.Time.now() + rospy.Duration(secs)
	while rospy.Time.now() < end:
		rospy.sleep(1)

def doRectange(times, side_time = 5, side_vel = 1):
	for i in range(times):
		print 'Forward'
		drone.set_cmd_vel(vx = side_vel)
		wait(side_time)
		print 'Left'
		drone.set_cmd_vel(vy = side_vel)
		wait(side_time)
		print 'Reverse'
		drone.set_cmd_vel(vx = -side_vel)
		wait(side_time)
		print 'Right'
		drone.set_cmd_vel(vy = -side_vel)
		wait(side_time)

drone = DroneWrapper(verbose = False)

print '***********************************Rectangle************************************'

print 'Calling Takeoff'
drone.takeoff(2)
wait(7)
doRectange(5)
wait(5)
print 'Landing'
drone.land()
wait(3)

print '***********************************Circle************************************'

print 'Calling Takeoff'
drone.takeoff(2)
print 'Doing circle'
drone.set_cmd_vel(vx = 0.5, az = 0.5)
wait(20)
print 'Landing'
drone.land()
wait(5)
