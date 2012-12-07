#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

import roslib
import rospy
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# these two need to be initialized after the call to rospy.init_node, but also need to be accessible from the handler functions
controller = None
display = None

def handleKeypress(key):
	print key
	if controller is not None: # it has been initialized
		if key == ord(' '):
			controller.SendEmergency()
		elif key == ord('p'):
			controller.SendTakeoff()
		elif key == ord('l'):
			controller.SendLand()
		else:
			roll = 0
			pitch = 0
			yaw_velocity = 0
			z_velocity = 0

			if key == ord('q'):
				yaw_velocity = -1
			elif key == ord('e'):
				yaw_velocity = 1
			elif key == ord('w'):
				pitch = 1
			elif key == ord('s'):
				pitch = -1
			elif key == ord('a'):
				roll = 1
			elif key == ord('d'):
				roll = -1
			elif key == ord('r'):
				z_velocity = 1
			elif key == ord('f'):
				z_velocity = -1

			controller.SendCommand(roll, pitch, yaw_velocity, z_velocity)


if __name__=='__main__':
	roslib.load_manifest('ardrone_keyboard_controller')
	rospy.init_node('ardrone_keyboard_controller')

	controller = BasicDroneController()
	display = DroneVideoDisplay()

	display.AddKeyboardCallback(handleKeypress)