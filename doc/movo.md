---
layout: page
title: Movo
permalink: /movo/
order: 98
---

Ein contains an interface to connect to and program the Kinova
[MOVO](http://www.kinovarobotics.com/innovation-robotics/movobeta/).
To install, create a new workspace with Ein, and the Kinova packages,
following the [Kinova Setup
Instructions](https://github.com/Kinovarobotics/kinova-movo/wiki/Setup-Instructions).
Ein will automatically detect the Kinova MOVO packages and compile the
MOVO library.  You should see a statement to this effect when you run
`catkin_make` in the workspace.  If Ein defaults to some other
library, this is because it detected another robot first in the
ROS_PACKAGE_PATH, such as Baxter or AIBO.  In that case, either change
the ROS_PACKAGE_PATH to exclude this package, or edit the Ein
CMakefile to only build the MOVO package.  

Copy the setup.sh file to the root directory of the workspace.  Then
run `ROBOT=movo2 screen -c src/ein/ein_movo.screenrc`.  This command
will start a screen session that is initialized with the relevant
commands and packages.

The relevant words are defined in ein_movo.cpp.

Ein has a concept of a focused end effector (either the left or right
arm).  You can change the focused end effector with
`incrementEndEffector`.  Words such as `xUp` and the current and
target pose displayed in the status bar affect only the focused end
effector.  All movement words, such as `xUp`, `localXUp`, `oXUp` and
`waitUntilAtCurrentPosition` should work.

TODO:

* open and close the gripper; set joint targets.

* light fields with the kinect 2 (Our kinect 2 frame rate is realllllly slow.)

* interfacing with the speaker and text to speech system.

* better base movement, so you can do it with the map.




