---
layout: page
title: System Architecture
permalink: /architecture/
order: 7
---


Ein is a C++ ROS program that does perception for robots.  It is a C++
ROS node with an associated QT5 gui.  When run, it opens up
subscriptions to various perception topics.  It starts a callback loop
(timercallback1 in ein.cpp) that does the following things:

* processes QT gui events
* updates various GUI windows with the latest data (such as the wrist view image and current pose).
* executes the word currently at the top of the callstack.  There is a
"stack collapse" mode where it may execute more than one word per
cycle.  This mode allows for increased computation speed because
threads and gui events are not serviced, but at the expense of
freezing message passing and the gui.
* publishes a state message about the current status of Ein (current call stack, data stack, etc). 
* publishes control messages to the robot.  Ein continuiesly drives
  the robot to assume the pose in `currentPose`.  On Baxter, it runs
  inverse kinematics to find joint angles (using the service or
  IKFast), then publishes those joint angles to the robot to assume
  that pose.  It also publishes commands to assume the state of the
  lights, the sonar, etc.  These messages can be toggled with `zeroGMode` and `publishCommandsMode`. 
* it then renders some more things. 

Ein has an associated programming language, Back, that is used to
instruct Ein to do things.  Back is inspired by FORTH and RPL and is
documented elsewhere on this website.  Ein takes Back commands on a
ROS topic and executes them. These commands can do anything from
turning the lights on Baxter to engaging very high-level long-term
actions.  Ein is a ROS program and more information about its [ROS
API](../ros/) is available.


Each instance of Ein can control one or both arms on Baxter. However,
because Ein is single threaded, if you use one Ein to control both
arms, it will run twice as slow.  As a result, we usually run two
separate instances of Ein, one for each arm.  The screen is set up
accordingly with separate windows for each arm (windows 0 and 9).  