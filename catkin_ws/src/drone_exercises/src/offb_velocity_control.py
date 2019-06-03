#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, SetMavFrame, SetMavFrameRequest

current_state = State()
goal_vel = Twist()

def state_cb(msg):
	global current_state
	current_state = msg
	rospy.loginfo('State Updated')

def goal_vel_cb(msg):
	global goal_vel
	goal_vel = msg
	rospy.loginfo('Goal Velocity updated')

rospy.init_node('offb_node', anonymous=True)

rospy.Subscriber('mavros/state', State, state_cb)
rospy.Subscriber('set_goal_vel', Twist, goal_vel_cb)
vel_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
frame_client = rospy.ServiceProxy('mavros/setpoint_velocity/mav_frame', SetMavFrame)

# Rate must be much higher than 2 Hz to avoid timeout
rate = rospy.Rate(20)

# Wait for FCU to connect to MAVROS
while not rospy.is_shutdown() and not current_state.connected:
	rate.sleep()
	rospy.loginfo('FCU not connected')

req = SetMavFrameRequest()
req.mav_frame = 8
frame_client(req)

rospy.loginfo('Publishing initial velocities')

# Setpoints need to be streaming before commanding a switch to OFFBOARD Mode
for i in range(50):
	if not rospy.is_shutdown():
		vel_publisher.publish(Twist())
		rate.sleep()

offb_mode_req = SetModeRequest()
offb_mode_req.custom_mode = 'OFFBOARD'
arm_req = CommandBoolRequest()
arm_req.value = True

last_req = rospy.Time.now()

rospy.loginfo('Entering Main Loop')

while not rospy.is_shutdown():
	if current_state.mode != 'OFFBOARD' and rospy.Time.now() - last_req > rospy.Duration(5.0):
		if set_mode_client(offb_mode_req).mode_sent:
			rospy.loginfo("OFFBOARD enabled")
		last_req = rospy.Time.now()
	elif not current_state.armed and rospy.Time.now() - last_req > rospy.Duration(5.0):
		if arm_client(arm_req).success:
			rospy.loginfo("Vehicle Armed")
		last_req = rospy.Time.now()
	vel_publisher.publish(goal_vel)
	rate.sleep()
