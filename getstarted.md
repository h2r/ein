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

Try the following commands for moving to other canonical
positions.

```
assumeCrane1
assumeBackScanningPose
```

Move back to home position.

```
xUp
xDown
```

Now try moving in the y and z directions.

By default, the robot is keyed to move in 1 cm increments.
You can change the increment size to 5 cm by issuing

```
0.05 setGridSize
```

Note that the grid size is specified in meters. To go back to 1 cm,
call

```
0.01 setGridSize
```

Try moving a long distance, like 20 cm. How fast does the robot move? To move slowly,
issue 

```
0.05 setSpeed
```

The speed is specified with a number between 0.0 and 1.0. There are some shortcut words for 
certain speeds:

```
fullImpulse
halfImpulse
quarterImpulse
tenthImpulse
```

Large movements should generally be carried out in `quarterImpulse` or `tenthImpulse`. 
Speeds greater than or equal to `halfImpulse` or `0.5` should be used with caution; even the safest
robot demands attention.


setSpeed
quarterImpulse

waitUntilAtCurrentPosition

oZUp, setGridSize

localXUp
Local frame remains the same until a rotation is issued.

openGripper, closeGripper
waitUntilGripperNotMoving

wrist view, reticles, keyboard bindings

mission 1: teleop some picks.
mission 2: stack some blocks.
hint: can you make phrases that make your life easier?
mission 3: knock over some blocks.


