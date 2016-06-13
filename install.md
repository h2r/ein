---
layout: page
title: Install
permalink: /install/
order: 0
---


This page describes how to create a catkin workspace and install Ein.

Install Ubuntu Trusty 14.04 LTS.

Install ROS Indigo as per http://wiki.ros.org/indigo/Installation/Ubuntu

If you are on Trusty 14.04 LTS with Indigo, you need the non-free
functionality of opencv.

Install the ppa at https://launchpad.net/~xqms/+archive/ubuntu/opencv-nonfree:

```
sudo add-apt-repository ppa:xqms/opencv-nonfree
```

Then update apt-get 

```
sudo apt-get update
```

Then run

```
sudo apt-get install libopencv-nonfree-\*
```

Next install other Ein dependencies: 

```
sudo apt-get install qt5-default python-wstool ros-indigo-object-recognition-msgs libgsl0-dev exuberant-ctags  ros-indigo-serial
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

Edit baxter.sh to set your ROS_HOSTNAME or ROS_IP and ROS_MASTER_URI
following the instructions in that file.  Then enter the Baxter ROS
workspace by running the baxter.sh file:

```
cd ~/catkin_ws
cp ~/catkin_ws/src/ein/baxter.sh ./
./baxter.sh
```

Next build for the first time:

```
cd .. 
source /opt/ros/indigo/setup.bash
catkin_make
catkin_make
```

At this point you should be able to run Rethink's tools; for example
to print the status of the robot:

``` 
rosrun baxter_tools  enable_robot.py -s
```

Commands such as `rostopic echo`, `rostopic list` and the like should
all work.  If these do not work, you may have network problems or ROS
configuration problems preventing you from connecting to Baxter.  See
the [Rethink SDK](http://sdk.rethinkrobotics.com/wiki/Main_Page) for
more information.


Some other stuff you might need to do when installing fresh:

```
sudo apt-get install ros-indigo-object-recognition-msgs
sudo apt-get install ros-indigo-pcl-ros
sudo apt-get install libgsl0-dev
sudo apt-get install qt5-default
```

Before running Ein, you should sync your workstation with your baxter:

```
sudo ntpdate <baxter name>
```

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
BAXTER=your_baxter screen -c src/ein/ein_baxter.screenrc
```

This command will start a screen session preloaded with useful
windows.  The ein program for the left arm is primed in window 8; for
the right arm is primed in window 0.  The repl is primed in window 1
and 7.  You can switch windows by using backtick-number.  For example,
typing `` `1`` switches to window 1.

If at any time you need to quit, you can type `` ` :quit `` in the
screen session.