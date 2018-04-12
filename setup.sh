# This is an example from Stefanie's ethernet environment.  Copy to
# your ws root directory and change the variables appropriately.  This
# is supposed to be like ./baxter.sh.  It sets environment variables
# for your environment.
#
# The pi.screenrc file requires this to be in the ws root.  You may
# also set any other ROS environment variables here.
if [ -n ${ROBOT} ]; then
export ROBOT=localhost
fi 

source devel/setup.bash
#export ROS_IP=192.168.42.1
export ROS_HOSTNAME=`hostname`
export ROS_MASTER_URI=http://$ROBOT:11311
export PS1="\[\033[00;33m\][jaco - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"

# If you are using baxter, make this file source baxter.sh
# source baxter.sh
