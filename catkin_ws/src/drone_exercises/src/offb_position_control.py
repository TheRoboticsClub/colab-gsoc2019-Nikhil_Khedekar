#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
goal_pose = PoseStamped()
goal_pose.pose.position.z = 2

def state_cb(msg):
	global current_state
	current_state = msg
	rospy.loginfo('State Updated')

def goal_pose_cb(msg):
	global goal_pose
	goal_pose = msg
	rospy.loginfo('Goal Updated')

if __name__ == "__main__":
	global current_state, goal_pose
	rospy.init_node('offb_node', anonymous=True)
	
	rospy.Subscriber('mavros/state', State, state_cb)
	rospy.Subscriber('set_goal_pose', PoseStamped, goal_pose_cb)
	pose_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

	arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
	
	# Rate must be much higher than 2 Hz to avoid timeout
	rate = rospy.Rate(20)

	# Wait for FCU to connect to MAVROS
	while not rospy.is_shutdown() and not current_state.connected:
		rate.sleep()
		rospy.loginfo('FCU not connected')

	rospy.loginfo('Publishing initial waypoints')

	# Setpoints need to be streaming before commanding a switch to OFFBOARD Mode
	for i in range(50):
		if not rospy.is_shutdown():
			pose_publisher.publish(goal_pose)
			rate.sleep()

	poshold_mode_req = SetModeRequest()
	poshold_mode_req.custom_mode = 'POSCTL'

	offb_mode_req = SetModeRequest()
	offb_mode_req.custom_mode = 'OFFBOARD'
	arm_req = CommandBoolRequest()
	arm_req.value = True

	# Switch to Position Hold before OFFBOARD as on losing connection, the failsafe enables the previous mode

	if set_mode_client(poshold_mode_req).mode_sent:
		rospy.loginfo('POSCTL enabled')

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
		pose_publisher.publish(goal_pose)
		rate.sleep()