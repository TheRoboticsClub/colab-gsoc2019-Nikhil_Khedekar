#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, SetMavFrame, SetMavFrameRequest, CommandTOL, CommandTOLRequest
from sensor_msgs.msg import NavSatFix

current_state = State()
goal_pose = PoseStamped()
goal_pose.pose.position.z = 2
current_pose = NavSatFix()

def state_cb(msg):
	global current_state
	current_state = msg
	# rospy.loginfo('State Updated')

def goal_pose_cb(msg):
	global goal_pose
	goal_pose = msg
	rospy.loginfo('Goal Updated')

def global_pose_cb(msg):
    global current_pose
    current_pose = msg

def takeoff(client, altitude = 3):
    global current_pose
    req = CommandTOLRequest()
    req.latitude = current_pose.latitude
    req.longitude = current_pose.longitude
    req.altitude = altitude
    client(req)

def cmd_vel_to_twist(vx = 0, vy = 0, vz = 0, ax = 0, ay = 0, az = 0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.linear.z = vz
    msg.angular.x = ax
    msg.angular.y = ay
    msg.angular.z = ax
    return msg

def timer_cb(event):
    global current_state, last_req
    if current_state.mode != 'OFFBOARD' and rospy.Time.now() - last_req > rospy.Duration(5.0):
        if set_mode_client(offb_mode_req).mode_sent:
            rospy.loginfo("OFFBOARD enabled")
        last_req = rospy.Time.now()
    elif not current_state.armed and rospy.Time.now() - last_req > rospy.Duration(5.0):
        if arm_client(arm_req).success:
            rospy.loginfo("Vehicle Armed")
        last_req = rospy.Time.now()

rospy.init_node('offb_node', anonymous=True)

rospy.Subscriber('mavros/state', State, state_cb)
rospy.Subscriber('set_goal_pose', PoseStamped, goal_pose_cb)
rospy.Subscriber('mavros/global_position/global', NavSatFix, global_pose_cb)
pose_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
vel_publisher = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)


arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
frame_client = rospy.ServiceProxy('mavros/setpoint_velocity/mav_frame', SetMavFrame)
takeoff_client = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

req = SetMavFrameRequest()
req.mav_frame = 8
frame_client(req)

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
        pose_publisher.publish(PoseStamped())
        rate.sleep()

offb_mode_req = SetModeRequest()
offb_mode_req.custom_mode = 'OFFBOARD'
arm_req = CommandBoolRequest()
arm_req.value = True
arm_client(arm_req)
last_req = rospy.Time.now()

while not rospy.is_shutdown():
    while not current_state.armed and current_state.mode != 'OFFBOARD':
        timer_cb(None)
        rate.sleep()

    rospy.Timer(rospy.Duration(5), timer_cb)

    rospy.loginfo('Taking off')
    takeoff(takeoff_client, 3)
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < time_start + rospy.Duration(3):
        rate.sleep()

    rospy.loginfo('Straight')

    # Go straight
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < time_start + rospy.Duration(5):
        vel_publisher.publish(cmd_vel_to_twist(vx = 1))
        rate.sleep()

    rospy.loginfo('Left')

    # Go Left 
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < time_start + rospy.Duration(5):
        vel_publisher.publish(cmd_vel_to_twist(vy = 1))
        rate.sleep()

    rospy.loginfo('back')

    # Go Back
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < time_start + rospy.Duration(5):
        vel_publisher.publish(cmd_vel_to_twist(vx = -1))
        rate.sleep()

    rospy.loginfo('Right')

    # Go Right
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < time_start + rospy.Duration(5):
        vel_publisher.publish(cmd_vel_to_twist(vy = -1))
        rate.sleep()

    rospy.loginfo('Trajectory complete, going to start position')
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < time_start + rospy.Duration(3):
        pose_publisher.publish(goal_pose)
        rate.sleep()
    
    break