#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

import rospy

from sensor_msgs.msg import Image    # for receiving the video feed
from geometry_msgs.msg import Twist  # for sending commands to the drone
from std_msgs.msg import Empty       # for land/takeoff/emergency
from ardrone_autonomy import navdata # for receiving navdata feedback (only state is used)

from drone_status import DroneStatus

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# Furthermore, when it receives a new video frame, it saves it locally
# It also tracks the drone state based on navdata feedback

class BasicDroneController(object):
	def __init__(self):
		# holds the current status
		self.status = -1

		# are we receiving from the drone?
		self.connected = False

		# subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',navdata,self.ReceiveNavdata) 
		
		# allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		
		# allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# finally, lets implement a timer to check whether we're still connected
		self.connectionTimer = rospy.Timer(rospy.Duration(0.5), self.ConnectionCallback)
		self.communicationSinceTimer = False

	# called every 0.5 seconds, if we haven't received anything since the last call, will assume we are having network troubles
	def ConnectionCallback(self, event):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def ReceiveNavdata(self,navdata):
		# although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		self.communicationSinceTimer = True

	def SendTakeoff(self):
		# send a takeoff message to the ardrone driver
		# note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# send a landing message to the ardrone driver
		# note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SendCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# send a control command to the ardrone
		msg = Twist()
		msg.linear.x = pitch
		msg.linear.y = roll
		msg.linear.z = z_velocity
		msg.angular.z = yaw_velocity
		self.pubCommand.publish(msg)

