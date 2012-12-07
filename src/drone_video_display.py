#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

import roslib; roslib.load_manifest('ardrone_tutorials_getting_started')
import rospy

from sensor_msgs.msg import Image    # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback (only state is used)

import cv_bridge, cv

from drone_status import DroneStatus

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# Furthermore, when it receives a new video frame, it saves it locally
# It also tracks the drone state based on navdata feedback

class DroneVideoDisplay(object):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# holds the current video frame
		self.image = None

		# holds the current status
		self.status = None

		# are we receiving from the drone?
		self.connected = False

		# subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# a timer to check whether we're still connected
		self.connectionTimer = rospy.Timer(rospy.Duration(0.5), self.ConnectionCallback)
		self.communicationSinceTimer = False

		# setting up cv to handle the image window
		self.cvBridge = cv_bridge.CvBridge()
		self.windowName = 'AR.Drone Video Stream'
		cv.NamedWindow(self.windowName)

		# holds a list of keyboard callbacks
		self.keyboardCallbacks = []

		# and finally, the timer which runs the main redraw loop and dispatches keypresses to callbacks
		self.redrawTimer = rospy.Timer(rospy.Duration(0.05), self.RedrawCallback)

	# called at a rate of 20Hz (50ms period), draws the current image on the screen and handles keypresses
	def RedrawCallback(self, event):
		cv.ShowImage(self.windowName,self.image) # draw the last received frame in our window
		
		if len(self.keyboardCallbacks)>0:
			key = cv.WaitKey(10) # wait 10ms for a keypress (timer period is 50ms)
			for callback in self.keyboardCallbacks:
				callback(key)

	# allows a custom callback to be added to handle keypresses
	def AddKeyboardCallback(self, callback):
		self.keyboardCallbacks.append(callback)

	# allows the removal of custom keypress callbacks
	def RemoveKeyboardCallback(self, callback):
		if callback in self.keyboardCallbacks:
			self.keyboardCallbacks.remove(callback)

	# called every 0.5 seconds, if we haven't received anything since the last call, will assume we are having network troubles
	def ConnectionCallback(self, event):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	# called when a new image is received
	def ReceiveImage(self,image):
		# store the new image when received
		self.image = self.cvBridge.imgmsg_to_cv(image,'bgr8')
		self.communicationSinceTimer = True

	def ReceiveNavdata(self,navdata):
		# although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		self.communicationSinceTimer = True

