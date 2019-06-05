#!/usr/bin/env python

import rospy
import tf
import threading
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, SetMavFrame, SetMavFrameRequest, CommandTOL, CommandTOLRequest

class DroneWrapper():
	def state_cb(self, msg):
		self.state = msg
		rospy.logdebug('State updated')

	def pose_stamped_cb(self, msg):
		self.pose_stamped = msg
		rospy.logdebug('Pose updated')
	
	def global_position_cb(self, msg):
		self.global_position = msg
		rospy.logdebug('Global position updated')
	
	def stay_armed_stay_offboard_cb(self, event):
		if self.state.mode != 'OFFBOARD':
			if self.request_mode('OFFBOARD'):
				rospy.loginfo("OFFBOARD requested")
		elif not self.state.armed:
			if self.arm(True):
				rospy.loginfo("Vehicle Armed")
			
	def arm(self, value = True):
		req = CommandBoolRequest()
		req.value = value
		if self.arm_client(req).success:
			rospy.loginfo('Arming/Disarming successful')
			return True
		else:
			rospy.logwarn('Arming/Disarming unsuccessful')
			return False
		
	def request_mode(self, mode = 'OFFBOARD'):
		rospy.sleep(2)
		rospy.loginfo('Current mode: %s', self.state.mode)
		req = SetModeRequest()
		req.custom_mode = mode
		if self.mode_client(req).mode_sent:
			rospy.loginfo('Mode change request successful')
			return True
		else:
			rospy.logwarn('Mode change request unsuccessful')
			return False

	def set_vel_frame(self, frame = 8):
		req = SetMavFrameRequest()
		req.mav_frame = frame
		if self.vel_frame_client(req).success:
			rospy.loginfo('Velocity frame change successful')
			return True
		else:
			rospy.logwarn('Velocity frame change unsuccessful')
			return False

	def set_cmd_vel(self, vx = 0, vy = 0, vz = 0, ax = 0, ay = 0, az = 0):
		self.cmd_vel = Twist()
		self.cmd_vel.linear.x = vx
		self.cmd_vel.linear.y = vy
		self.cmd_vel.linear.z = vz
		self.cmd_vel.angular.x = ax
		self.cmd_vel.angular.y = ay
		self.cmd_vel.angular.z = az
		self.cmd_vel_publisher.publish(self.cmd_vel)
	
	def set_cmd_pose(self, x = 0, y = 0, z = 2, psi = 0):
		self.cmd_pose = PoseStamped()
		self.cmd_pose.pose.position.x = x
		self.cmd_pose.pose.position.y = y
		self.cmd_pose.pose.position.z = z
		q = tf.transformations.quaternion_from_euler(0, 0, psi)
		self.cmd_pose.pose.orientation.x = q[0]
		self.cmd_pose.pose.orientation.y = q[1]
		self.cmd_pose.pose.orientation.z = q[2]
		self.cmd_pose.pose.orientation.w = q[3]
		self.local_pose_stamped_publisher.publish(self.cmd_pose)

	def takeoff(self, altitude = 2):
		self.set_cmd_vel()
		self.hold_cmd_vel()
		self.arm(True)
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
		while True:
			while not (self.state.armed and self.state.mode == 'OFFBOARD'):
				self.rate.sleep()
			rospy.loginfo('Sleeping 3 secs to confirm change')
			rospy.sleep(3)
			if self.state.mode == 'OFFBOARD':
				break
		self.set_cmd_vel(vz=1)
		rospy.loginfo('Taking off!!!')
		rospy.sleep(3)
		self.set_cmd_vel()
		
		# Takeoff through position control
		# self.set_cmd_pose(0,0,0,0)
		# self.hold_cmd_pose()
		# self.arm(True)
		# self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
		# while True:
		# 	while not (self.state.armed and self.state.mode == 'OFFBOARD'):
		# 		self.rate.sleep()
		# 	rospy.logdebug('Sleeping 3 secs to confirm change')
		# 	rospy.sleep(3)
		# 	if self.state.mode == 'OFFBOARD':
		# 		break
		# self.set_cmd_pose(0, 0, altitude, 0)
		# rospy.loginfo('Taking off!!!')
		# rospy.sleep(10)
		# # self.cmd_pose_timer.shutdown()
		
	def land(self):
		self.cmd_pose_timer.shutdown()
		self.cmd_vel_timer.shutdown()
		self.stay_armed_stay_offboard_timer.shutdown()
		req = CommandTOLRequest()
		req.latitude = self.global_position.latitude
		req.longitude = self.global_position.longitude
		self.land_client(req)
		
	def hold_cmd_pose(self):
		if self.cmd_vel_flag:
			self.cmd_vel_flag = False
			self.cmd_vel_timer.shutdown()
		self.cmd_pose_flag = True
		self.cmd_pose_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_cmd_pose)
		
	def hold_cmd_vel(self):
		if self.cmd_pose_flag:
			self.cmd_pose_flag = False
			self.cmd_pose_timer.shutdown()
		self.cmd_vel_flag = True
		self.cmd_vel_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_cmd_vel)
	
	def repeat_cmd_pose(self, event):
		self.local_pose_stamped_publisher.publish(self.cmd_pose)

	def repeat_cmd_vel(self, event):
		self.cmd_vel_publisher.publish(self.cmd_vel)

	def __init__(self, verbose):
		if verbose:
			rospy.init_node('drone', log_level = rospy.DEBUG)
		else:
			rospy.init_node('drone')
		
		self.state = State()
		self.pose_stamped = PoseStamped()
		self.rate = rospy.Rate(20)
		self.cmd_vel = Twist()
		self.cmd_pose = PoseStamped()
		self.cmd_vel_flag = False
		self.cmd_pose_flag = False
		
		self.cmd_pose_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_cmd_pose)
		self.cmd_pose_timer.shutdown()
		self.cmd_vel_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_cmd_vel)
		self.cmd_vel_timer.shutdown()
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(5), self.stay_armed_stay_offboard_cb)
		self.stay_armed_stay_offboard_timer.shutdown()

		rospy.wait_for_service('mavros/cmd/arming')
		self.arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		rospy.wait_for_service('mavros/set_mode')
		self.mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
		rospy.wait_for_service('mavros/setpoint_velocity/mav_frame')
		self.vel_frame_client = rospy.ServiceProxy('mavros/setpoint_velocity/mav_frame', SetMavFrame)
		rospy.wait_for_service('mavros/cmd/land')
		self.land_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
		
		rospy.Subscriber('mavros/state', State, self.state_cb)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
		rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_cb)

		self.local_pose_stamped_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size = 5)
		self.cmd_vel_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 5)

		while not self.set_vel_frame(8):
			rospy.logwarn('Unable to change velocity frame')
			self.rate.sleep()