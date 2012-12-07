#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

import roslib; roslib.load_manifest('ardrone_tutorials_getting_started')
import rospy
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# these two need to be initialized after the call to rospy.init_node, but also need to be accessible from the handler functions
controller = None
display = None

class KeyMapping(object):
	PitchForward = ord('w')
	PitchBackward = ord('s')
	RollLeft = ord('a')
	RollRight = ord('d')
	YawLeft = ord('q')
	YawRight = ord('e')
	IncreaseAltitude = ord('r')
	DecreaseAltitude = ord('f')
	Takeoff = ord('p')
	Land = ord('l')
	Emergency = ord(' ')
	Exit = 27 # Esc key


def handleKeypress(key):
	if controller is not None: # it has been initialized
		if key == KeyMapping.Exit:
			controller.SendLand()
			rospy.signal_shutdown('Great job, you flew like an ace!')
		elif key == KeyMapping.Emergency:
			controller.SendEmergency()
		elif key == KeyMapping.Takeoff:
			controller.SendTakeoff()
		elif key == KeyMapping.Land:
			controller.SendLand()
		else:
			roll = 0
			pitch = 0
			yaw_velocity = 0
			z_velocity = 0

			if key == KeyMapping.YawLeft:
				yaw_velocity = 1
			elif key == KeyMapping.YawRight:
				yaw_velocity = -1

			elif key == KeyMapping.PitchForward:
				pitch = 1
			elif key == KeyMapping.PitchBackward:
				pitch = -1

			elif key == KeyMapping.RollLeft:
				roll = 1
			elif key == KeyMapping.RollRight:
				roll = -1

			elif key == KeyMapping.IncreaseAltitude:
				z_velocity = 1
			elif key == KeyMapping.DecreaseAltitude:
				z_velocity = -1

			controller.SetCommand(roll, pitch, yaw_velocity, z_velocity)


if __name__=='__main__':
	rospy.init_node('ardrone_keyboard_controller')

	controller = BasicDroneController()
	display = DroneVideoDisplay()

	display.AddKeyboardCallback(handleKeypress)

	while not rospy.is_shutdown():
		rospy.sleep(1.0)