#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

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
	
	def cam_frontal_cb(self, msg):
		self.frontal_image = msg
		rospy.logdebug('Frontal image updated')
	
	def cam_ventral_cb(self, msg):
		self.ventral_image = msg
		rospy.logdebug('Ventral image updated')
	
	def stay_armed_stay_offboard_cb(self, event):
		if self.state.mode != 'OFFBOARD':
			if self.request_mode('OFFBOARD'):
				rospy.loginfo("OFFBOARD requested")
		elif not self.state.armed:
			if self.arm(True):
				rospy.loginfo("Vehicle Armed")
			
	def get_frontal_image(self):
		return self.frontal_image
	
	def get_ventral_image(self):
		return self.ventral_image
			
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

	def set_cmd_vel(self, vx = 0, vy = 0, vz = 0, az = 0, z = 2):
		self.setpoint_raw.type_mask = int('0b010111000011', 2)
		self.setpoint_raw.coordinate_frame = 8
		self.setpoint_raw.velocity.x = -vy
		self.setpoint_raw.velocity.y = vx
		self.setpoint_raw.velocity.z = vz
		self.setpoint_raw.position.z = z
		self.setpoint_raw.yaw_rate = az
		self.setpoint_raw_publisher.publish(self.setpoint_raw)

	def repeat_setpoint_raw(self, event):
		self.setpoint_raw_publisher.publish(self.setpoint_raw)

	def hold_setpoint_raw(self):
		if not self.setpoint_raw_flag:
			self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)

	def takeoff(self, altitude = 2):
		self.set_cmd_vel(0, 0, 0, 0, 0)
		self.hold_setpoint_raw()
		self.arm(True)
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
		while True:
			while not (self.state.armed and self.state.mode == 'OFFBOARD'):
				self.rate.sleep()
			rospy.loginfo('Sleeping 3 secs to confirm change')
			rospy.sleep(3)
			if self.state.mode == 'OFFBOARD':
				break
		self.set_cmd_vel(z = altitude)
		rospy.loginfo('Taking off!!!')
		rospy.sleep(5)
		
	def land(self):
		self.setpoint_raw_timer.shutdown()
		self.stay_armed_stay_offboard_timer.shutdown()
		req = CommandTOLRequest()
		req.latitude = self.global_position.latitude
		req.longitude = self.global_position.longitude
		self.land_client(req)

	def __init__(self, verbose = False):
		if verbose:
			rospy.init_node('drone', log_level = rospy.DEBUG)
		else:
			rospy.init_node('drone')
		
		self.state = State()
		self.pose_stamped = PoseStamped()
		self.rate = rospy.Rate(20)
		self.setpoint_raw = PositionTarget()
		self.setpoint_raw_flag = False
		
		self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)
		self.setpoint_raw_timer.shutdown()
		self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(5), self.stay_armed_stay_offboard_cb)
		self.stay_armed_stay_offboard_timer.shutdown()

		rospy.wait_for_service('mavros/cmd/arming')
		self.arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		rospy.wait_for_service('mavros/set_mode')
		self.mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
		rospy.wait_for_service('mavros/cmd/land')
		self.land_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
		
		rospy.Subscriber('mavros/state', State, self.state_cb)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
		rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_cb)
		rospy.Subscriber('iris/camera_frontal/image_raw', Image, self.cam_frontal_cb)
		rospy.Subscriber('iris/camera_ventral/image_raw', Image, self.cam_ventral_cb)

		self.setpoint_raw_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size = 1)