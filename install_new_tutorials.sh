#!/bin/bash

# This file will be updated when new tutorials are available for download!

# first, a fix for the update_tutorials script found in the ARDroneUbuntu VM
if [ -f "~/bin/update_tutorials" ];
then
 echo "Detected ARDroneUbuntu" 
 rm ~/bin/update_tutorials
 cp ARDroneUbuntu_Updates/update_tutorials ~/bin/update_tutorials
 chmod +x ~/bin/update_tutorials
fi

roscd

if [ -d "ardrone_tutorials_joystick_control" ];
then
 cd ardrone_tutorials_joystick_control
 git pull
 roscd
else
 git clone https://github.com/mikehamer/ardrone_tutorials_joystick_control.git
fi