#!/usr/bin/env python

from drone_wrapper import DroneWrapper
import rospy

drone = DroneWrapper()

rospy.sleep(1)

if drone.get_position().z < 0.1:
	print 'taking off due to ', drone.get_position().z
	drone.takeoff(3)
else: 
	drone.take_control()

while True:
	vels = raw_input('Enter space seperated velocities as vx vy vz az and then press enter\nValues: ')
	if vels == '':
		break
	vx, vy, vz, az = map(float, vels.strip().split(' '))
	drone.set_cmd_vel(vx, vy, vz, az)