import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class FollowRoad(Plugin):
	def __init__(self, context):
		super(FollowRoad, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('FollowRoad')

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which should be in the "resource" folder of this package
		ui_file = os.path.join(rospkg.RosPack().get_path(
			'rqt_follow_road'), 'resource', 'follow_road.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('FollowRoadUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.

		if context.serial_number() > 1:
			self._widget.setWindowTitle(
				self._widget.windowTitle() + (' (%d)' % context.serial_number()))

		# Setup pubs
		self.takeoff_pub = rospy.Publisher('gui/takeoff_land', Bool, queue_size=1)
		self.play_stop_pub = rospy.Publisher('gui/play_stop', Bool, queue_size=1)
		self.alt_slider_pub = rospy.Publisher('gui/alt_slider', Float64, queue_size=1)
		self.rotation_dial_pub = rospy.Publisher('gui/rotation_dial', Float64, queue_size=1)

		# Setup variables
		self.play_code_flag = False
		self.takeoff = False

		# Add widget and controls to the user interface
		context.add_widget(self._widget)
		self._widget.takeoffButton.clicked.connect(self.call_takeoff_land)
		self._widget.playButton.clicked.connect(self.call_play)
		self._widget.stopButton.clicked.connect(self.call_stop)
		self._widget.resetButton.clicked.connect(self.call_reset)
		self._widget.altdSlider.valueChanged.connect(self.alt_slider_val_changed)
		self._widget.rotationDial.valueChanged.connect(self.rotation_val_changed)

		# Setup subs
		rospy.Subscriber('interface/filtered_img', Image, self.filtered_img_cb)
		rospy.Subscriber('interface/threshed_img', Image, self.threshed_img_cb)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	def call_takeoff_land(self):
		if self.takeoff == True:
			self._widget.takeoffButton.setText("Take Off")
			rospy.loginfo('Landing')
			self.takeoff_pub.publish(Bool(False))
			self.takeoff = False
		else:
			self._widget.takeoffButton.setText("Land")
			rospy.loginfo('Taking off')
			self.takeoff_pub.publish(Bool(True))
			self.takeoff = True

	def call_play(self):
		if not self.play_code_flag:
			rospy.loginfo('Executing student code')
			self.play_stop_pub.publish(Bool(True))
			self.play_code_flag = True
		else:
			rospy.loginfo('Already playing student code')

	def call_stop(self):
		if self.play_code_flag:
			rospy.loginfo('Stopping student code')
			self.play_stop_pub.publish(Bool(False))
			self.play_code_flag = False
		else:
			rospy.loginfo('Student code not running')

	def call_reset(self):
		rospy.loginfo('Resetting environment - Not yet implemented')

	def alt_slider_val_changed(self, value):
		value = (1.0/(self._widget.altdSlider.maximum()/2)) * (value - (self._widget.altdSlider.maximum()/2))
		self._widget.altdValue.setText('%.2f' % value)
		rospy.logdebug('Altitude slider value changed to: %.2f', value)
		self.alt_slider_pub.publish(Float64(value))

	def rotation_val_changed(self, value):
		value = (1.0/(self._widget.rotationDial.maximum()/2)) * (value - (self._widget.rotationDial.maximum()/2))
		self._widget.rotValue.setText('%.2f' % value)
		rospy.logdebug('Rotational dial value changed to: %.2f', value)
		self.rotation_dial_pub.publish(Float64(value))

	# Figure out how to use the rqt_image_view
	def filtered_img_cb(self, msg):
		pass

	def threshed_img_cb(self, msg):
		pass

	def pose_stamped_cb(self, msg):
		# self.XValue.setText('%.2f' % newX)
        # self.YValue.setText('%.2f' % newY)
        # self.cmdvel.setVX(-newY)
        # self.cmdvel.setVY(-newX)
        # self.cmdvel.sendVelocities()
		pass


	#def trigger_configuration(self):
	# Comment in to signal that the plugin has a way to configure
	# This will enable a setting button (gear icon) in each dock widget title bar
	# Usually used to open a modal configuration dialog
