#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from drone_wrapper import DroneWrapper
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image

code_live_flag = False

def gui_takeoff_cb(msg):
	drone.takeoff()

def gui_land_cb(msg):
	drone.land()

def gui_play_stop_cb(msg):
	global code_live_flag, code_live_timer
	if msg.data == True:
		if not code_live_flag:
			code_live_flag = True
			code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	else:
		if code_live_flag:
			code_live_flag = False
			code_live_timer.shutdown()
		
def gui_alt_slider_cb(msg):
	global drone
	drone.set_cmd_vel(vz = msg.data)

def gui_rotation_dial_cb(msg):
	global drone
	drone.set_cmd_vel(az = msg.data)

def set_image_filtered(img):
	gui_filtered_img_pub.publish(img)

def set_image_threshed(img):
	gui_threshed_img_pub.publish(img)

def execute(event):
	global drone
	img_frontal = drone.get_frontal_image()
	img_ventral = drone.get_ventral_image()
	pass

if __name__ == "__main__":
	drone = DroneWrapper()
	rospy.Subscriber('gui/takeoff_land', Bool, gui_takeoff_cb)
	rospy.Subscriber('gui/play_stop', Bool, gui_play_stop_cb)
	rospy.Subscriber('gui/alt_slider', Float64, gui_alt_slider_cb)
	rospy.Subscriber('gui/rotation_dial', Float64, gui_rotation_dial_cb)
	gui_filtered_img_pub = rospy.Publisher('interface/filtered_img', Image)
	gui_threshed_img_pub = rospy.Publisher('interface/threshed_img', Image)
	code_live_flag = False
	code_live_timer = rospy.Timer(rospy.Duration(nsecs=50000000), execute)
	code_live_timer.shutdown()
	while not rospy.is_shutdown():
		rospy.spin()