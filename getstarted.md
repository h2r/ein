---
layout: page
title: Get Started
permalink: /getstarted/
order: 1
---

In your first ten minutes with Ein, we will teach you how to move the
robot around and write simple programs.  Before following these
instructions, you should install Ein into a catkin workspace and
verify that you can connect to your Baxter.  Many common problems
arise because of networking misconfigurations, where your
$ROS_MASTER_URI, $ROS_HOSTNAME, and/or $ROS_IP are set incorrectly.
Start the screen session using the commands given in
[Install](../install).  Choose an arm (left or right) to use.  Type ``
`8 `` to tell screen to go to the left arm's main Ein window, or `` `0
`` to go to the right arm.  The screen should be primed with a build
and run command; when it starts, a number of windows should open.  Go
to the main window and verify you can see the wrist view.  The robot's
status will appear in text.  The frame rate for Ein varies betweeun
10Hz and 25Hz on our machines; if it is much slower than 10Hz, Ein
will be hard to use and you should get a faster computer.

![Ein Main Window](../assets/mainwindow_screenshot.jpg)

Verify you can see the wrist view image in the console, and that it is
updating as the arm moves.  The middle window shows a map of the
robot's IK workspace, with the robot's body and end effector position
marked.  Double clicking on the map moves the end effector to the
appropriate location; verify that the robot moves when you click.

Next, go back to the screen session and change to the Ein console
window by typing `` `1 `` for the right arm or `` `2 `` for the left
arm.

![Ein Console Screenshot](../assets/console_screenshot.jpg)

The console is the main way to interact with Ein and control the
robot.  You type commands at the prompt.  You can view the robot's
status in the upper left window.  The middle two windows show the
stack (don't worry about that now).  The right window shows Ein's
console output as you run commands.

First, go home.  The home position is set toward the side of the robot
near the center of its IK workspace in the crane position.

```
goHome
```

After running this command, the arm should move to the crane position
off to one side, as shown in the following picture.

![Robot in Home Position](../assets/baxter_athome.jpg)

