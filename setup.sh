#!/bin/bash

# This file, when sourced, sets your ROS environment variables.  It
# should be placed in the root of the catkin workspace, since it also
# sources the catkin setup.bash file.  It takes a ROBOT= enviornment
# variable which it uses to set things.  You may need to edit this
# file depending on your network configuration to set ROS_IP or
# ROS_HOSTNAME for your machine. 

if [ -z ${ROBOT} ]; then
export ROBOT=localhost
fi 

source install/setup.sh
#export ROS_IP=192.168.42.1
export SPOT_IP=donner
export BOSDYN_CLIENT_USERNAME=user
export BOSDYN_CLIENT_PASSWORD=bbbdddaaaiii
export PS1="\[\033[00;33m\][spot - ${SPOT_IP}]\[\033[00m\] $PS1"

# If you are using baxter, make this file source baxter.sh and remove
# the othe rstuff above, which is redundant if you are using
# baxter.sh.  

# source baxter.sh
