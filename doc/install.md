---
layout: page
title: Install
permalink: /install/
order: 0
---


This page describes how to create a catkin workspace and install Ein.

Install [Ubuntu Trusty 14.04 LTS](http://releases.ubuntu.com/14.04/).

Install ROS Indigo as per http://wiki.ros.org/indigo/Installation/Ubuntu

If you are on Trusty 14.04 LTS with Indigo, you need the non-free
functionality of opencv.

Install other Ein dependencies: 

```
sudo apt-get install qt5-default python-wstool ros-indigo-object-recognition-msgs libgsl0-dev ros-indigo-serial ros-indigo-object-recognition-msgs ros-indigo-pcl-ros libgsl0-dev qt5-default screen
```

Create your catkin workspace:

```
mkdir -p ~/catkin_ws/src
```

Put Ein in it.

```
cd ~/catkin_ws/src
git clone http://github.com/h2r/ein
```

Use wstool to check out the Baxter SDK.  These commands check out the
most recent version of the SDK; if you are running an older version
you will need to check out the correct branch. 

```
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
```

Next run catkin_make for the first time:

```
cd ~/catkin_ws/src 
source /opt/ros/indigo/setup.bash
catkin_make
```

The catkin_make command fail, but will create the devel directory so that the
Rethink baxter.sh script will work.   
Copy baxter.sh from the Ein directory into the top of your workspace:
```
cd ~/catkin_ws/src  && cp src/ein/baxter.sh . 
```
You can also use the one distributed by Rethink, but we have modified theirs to include some nice environment variables to set your ROS_HOSTNAME or ROS_IP and
ROS_MASTER_URI.

```
cd ~/catkin_ws
./baxter.sh
```

Now build again; this build should succeed:

```
catkin_make
```


Before running Ein, you should sync the time on your workstation with
your baxter:

```
sudo ntpdate <baxter name>
```

Note that Baxter syncs its time using ntp to pool.ntp.org, and this
server cannot be changed (as per Rethink's instructions).  See our
[FAQ](../faq/#i-am-getting-strage-tf-errors-about-lookup-would-require-extrapolation-into-the-past-or-lookup-would-require-extrapolation-into-the-future) entry for more information.


To run the program, from the root of your catkin workspace, run the
following command:

```
catkin_make && gdb --args ./devel/lib/ein/ein  _data_directory:="$(rospack find ein)/default" physical left
```

And enter 'r' to start the program from within gdb. For safety
purposes, the arm should not move at startup.


We strongly recommend running ein from inside of GNU Screen.  We have
provided a screen configuration file to make this easy.  Go to the
root of your catkin workspace and run

``` 
BAXTER=<your_baxter> screen -c src/ein/ein_baxter.screenrc
```

Replease <your_baxter> with the hostname or IP address of your Baxter;
this will be used to set the $ROS_MASTER_URI environment variable
required by ROS.  You may also need to adjust baxer.sh to change the
$ROS_HOSTNAME or $ROS_IP.  If your network is set up so that the
$HOSTNAME environment variable can be used as your $ROS_HOSTNAME, then
you do not need to change baxter.sh; otherwise you need to adjust it.
(If you ssh into Baxter, you should be able to ping your machine from
the Baxeter robot.  See the ROS instructions and tutorials for more
information.)  This command will start a screen session preloaded with
useful windows.  The ein program for the left arm is primed in window
8; for the right arm is primed in window 0.  The repl is primed in
window 1 and 7.  You can switch windows by using backtick-number.  For
example, typing `` `1`` switches to window 1.

If at any time you need to quit, you can type `` ` :quit `` in the
screen session.

At this point you should be able to run Rethink's tools; for example
to print the status of the robot:

``` 
rosrun baxter_tools  enable_robot.py -s
```

Commands such as `rostopic echo`, `rostopic list` and the like should
all work.  If these do not work, you may have network problems or ROS
configuration problems preventing you from connecting to Baxter.  A
common problem is that the version of the SDK is a different version
from the run running on your robot.  See the [Rethink
SDK](http://sdk.rethinkrobotics.com/wiki/Main_Page) for more
information.  Another problem is that your ROS_MASTER_URI or
ROS_HOSTNAME or ROS_IP are set incorrectly.

