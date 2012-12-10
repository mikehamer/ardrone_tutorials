#!/bin/bash

roscd

if [ -f "ardrone_autonomy" ];
then
 cd ardrone_autonomy
 git pull
 ./build_sdk.sh
 roscd
else
 git clone https://github.com/AutonomyLab/ardrone_autonomy.git
 cd ardrone_autonomy
 ./build_sdk.sh
 roscd
fi

if [ -f "ardrone_tutorials_getting_started" ];
then
 cd ardrone_tutorials_getting_started
 git pull
 roscd
else
 git clone https://github.com/mikehamer/ardrone_tutorials_getting_started.git
fi

rospack profile
rosstack profile

rosmake -a

read -p "Update complete. Press [Enter] to continue."