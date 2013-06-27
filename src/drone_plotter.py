#!/usr/bin/env python

# A data plotting ROS Node for the tutorial "Up and flying with the AR.Drone and ROS | AR.Drone Feedback"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This node listens for feedback from the AR.Drone and updates graphs accordingly.

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist      # for sending commands to the drone
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

import numpy as np
import matplotlib
#If you get the error: AttributeError: 'FigureCanvasMac' object has no attribute 'copy_from_bbox', uncomment the following line
#matplotlib.use('TkAgg')
import matplotlib.pylab as pl

# We need to use resource locking to handle synchronization between plot thread and ROS topic callbacks
from threading import Lock

PLOT_RATE = 10
EULER_ANGLE_MAX = 0.35

lockCmd = Lock()
lockEst = Lock()

timeCmd = np.array([])
timeEst = np.array([])
pitchCmd = np.array([])
pitchEst = np.array([])
rollCmd = np.array([])
rollEst = np.array([])

def plotLoop():
    fig,ax = pl.subplots(2,1)
    fig.show()
    fig.canvas.draw()
    
    # setting up the initial two plots
    ax[0].set_title = 'AR.Drone Pitch'
    ax[1].set_title = 'AR.Drone Roll'
    ax[0].set_ylabel = 'Angle [deg]'
    ax[1].set_ylabel = 'Angle [deg]'
    ax[0].set_xlabel = 'Time [s]'

    # caching the figure background for faster updates
    bg = [fig.canvas.copy_from_bbox(a.bbox) for a in ax]

    # plotting the initial data
    pltPitchEst = ax[0].plot(timeEst,pitchEst,'b')[0]
    pltPitchCmd = ax[0].plot(timeCmd,pitchCmd,'r')[0]
    pltRollEst = ax[1].plot(timeEst,rollEst,'b')[0]
    pltRollCmd = ax[1].plot(timeCmd,rollCmd,'r')[0]

    # loop with the defined frequency until ROS quits
    loop_rate = rospy.Rate(PLOT_RATE)
    while not rospy.is_shutdown():

        #set the new plot data
        lockCmd.acquire()
        lockEst.acquire()
        pltPitchEst.set_data(timeEst,pitchEst)
        pltPitchCmd.set_data(timeCmd,pitchCmd)
        pltRollEst.set_data(timeEst,rollEst)
        pltRollCmd.set_data(timeCmd,rollCmd)
        lockCmd.release()
        lockEst.release()

        # restore background
        for b in bg: fig.canvas.restore_region(b)

        # redraw just the new data, not the whole figure
        ax[0].draw_artist(pltPitchEst)
        ax[0].draw_artist(pltPitchCmd)
        ax[1].draw_artist(pltRollEst)
        ax[1].draw_artist(pltRollCmd)

        # fill in the axes rectangle
        for a in ax: fig.canvas.blit(a.bbox)

        # sleep for long enough to achieve the required loop frequency
        loop_rate.sleep()

def ReceiveNavdata(data):
    global timeEst, pitchEst, rollEst
    lockEst.acquire()
    timeEst = np.append(timeEst, rospy.get_rostime().to_sec())
    pitchEst = np.append(pitchEst, np.rad2deg(data.rotY))
    rollEst = np.append(rollEst, np.rad2deg(data.rotX))
    lockEst.release()

def ReceiveCmdVel(data):
    global timeCmd, pitchCmd, rollCmd
    lockCmd.acquire()
    timeCmd = np.append(timeCmd, rospy.get_rostime().to_sec())
    pitchCmd = np.append(pitchCmd, np.rad2deg(data.linear.x*EULER_ANGLE_MAX))
    rollCmd = np.append(rollCmd, np.rad2deg(data.linear.y*EULER_ANGLE_MAX))
    lockCmd.release()

if __name__=='__main__':
    # initialize the ROS node
    rospy.init_node('ardrone_plotter')

    # Read the AR.Drone's maximum angle setting. This is used to scale the commanded angle for comparison to the estimate
    EULER_ANGLE_MAX = float ( rospy.get_param("/ardrone_driver/euler_angle_max", EULER_ANGLE_MAX) )

    ############################################
    # Add your subscribers and publishers here #
    ############################################

    #             rospy.Subscriber( <topic> , <message type> , <callback function> )
    sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveNavdata)
    sub_CmdVel  = rospy.Subscriber('/cmd_vel', Twist, ReceiveCmdVel)

    plotLoop()