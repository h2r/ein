---
layout: page
title: Install
permalink: /install/
order: 0
---


This is how to install ein and create a catkin workspace. 

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

and finally 

```
sudo apt-get install qt5-default python-wstool ros-indigo-object-recognition-msgs libgsl0-dev exuberant-ctags  ros-indigo-serial
```

**Install** the package h2r/ein by going to catkin_ws/src and then cloning.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone http://github.com/h2r/ein
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
cp baxter/baxter.sh ..
```

edit baxter.sh following the instructions in that file.

```
./baxter.sh
```

Then build:

```
cd .. 
source /opt/ros/indigo/setup.bash
catkin_make
source devel/setup.bash
catkin_make
```

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


We usually run ein from inside of screen.  Go to the root of your
catkin workspace and run 

``` 
screen -c src/ein/ein_baxter.screenrc
```

This command will start a screen session preloaded with useful
windows.  The ein program for the left arm is primed in window 8; for
the right arm is primed in window 0.  The repl is primed in window 1
and 7.  You can switch windows by using backtick-number.  For example,
typing "`1" switches to window 1.
