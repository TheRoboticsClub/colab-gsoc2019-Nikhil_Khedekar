import threading
import time
from datetime import datetime
import cv2
import numpy as np
import math

from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80

hmin = 20
smin = 0
vmin = 0


class MyAlgorithm(threading.Thread):

	def __init__(self, camera, navdata, pose, cmdvel, extra):
		self.camera = camera
		self.navdata = navdata
		self.pose = pose
		self.cmdvel = cmdvel
		self.extra = extra

		self.minError = 10
		self.prev_section = 0

		self.height = 240
		self.width = 320

		self.first_execute = True

		self.image = None

		self.stop_event = threading.Event()
		self.kill_event = threading.Event()
		self.lock = threading.Lock()
		threading.Thread.__init__(self, args=self.stop_event)

	def setImageFiltered(self, image):
		self.lock.acquire()
		self.image = image
		self.lock.release()

	def getImageFiltered(self):
		self.lock.acquire()
		tempImage = self.image
		self.lock.release()
		return tempImage

	def run(self):

		self.stop_event.clear()

		while (not self.kill_event.is_set()):

			start_time = datetime.now()

			if not self.stop_event.is_set():
				self.execute()

			finish_Time = datetime.now()

			dt = finish_Time - start_time
			ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
			#print (ms)
			if (ms < time_cycle):
				time.sleep((time_cycle - ms) / 1000.0)

	def stop(self):
		self.stop_event.set()

	def play(self):
		if self.is_alive():
			self.stop_event.clear()
		else:
			self.start()

	def kill(self):
		self.kill_event.set()

	def execute(self):
		input_image = self.camera.getImage().data
		if input_image is not None:
			# Add your code here
			img = np.copy(input_image)
			
			# Convert to HSV
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

			# Create filter. Values from colorTuner
			lower_road = np.array([0, 0, 45])
			upper_road = np.array([197, 50, 80])
			mask = cv2.inRange(hsv, lower_road, upper_road)

			# # Only need to use lower half of the image
			mask_lower = mask[120:,:]

			# Find the contours
			_, contours, __ = cv2.findContours(mask_lower, 1, 2)
			
			# Select longest contour and fit an ellipse on it
			sizes = map(len, contours)
			cnt = contours[sizes.index(max(sizes))]
			
			# Method using moments - Ellipse works better
			# M = cv2.moments(cnt)
			# cx = int(M['m10']/M['m00'])
			# cy = int(M['m01']/M['m00'])
			# img_lower = img[120:, :]
			# cv2.drawContours(img_lower, contours, sizes.index(max(sizes)), (0,0, 255), 3)
			# cv2.circle(img_lower, (cx, cy), 5, (0,255,0))

			ellipse = cv2.fitEllipse(cnt)
			cx = int(ellipse[0][0])
			cy = int(ellipse[0][1])
			img_lower = img[120:, :]
			cv2.drawContours(img_lower, contours, sizes.index(max(sizes)), (0,0, 255), 3)

			# Display ellipse
			cv2.ellipse(img_lower,ellipse,(0,255,0),4)
			cv2.circle(img_lower, (cx, cy), 5, (0,255,0))
			cv2.line(img_lower,(160,0),(160, 240),(255,0,0),4)
			self.setImageFiltered(img_lower)
			
			error = (160 - cx)
			kp = 0.0005 #0.1/160
			az = np.clip(kp*error, -0.3, 0.3)
			self.cmdvel.sendCMDVel(0.3,0,0,0,0,az)
			
			print 'center is at ', cx
			print 'error is ', error 
			print 'correction is', az

			'''
			If you want show a thresold image (black and white image)
			self.camera.setThresholdImage(bk_image)
			'''