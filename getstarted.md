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

![Ein Main Window](../assets/einmainwindow_screenshot.jpg)

Verify you can see the wrist view image in the console, and that it is
updating as the arm moves.  The middle pane (Object Map View) shows a
map of the robot's IK workspace for the arm in crane pose at a
particular height.  Attainable poses are marked in green and yellow;
unattainable poses are marked in red.  The robot's body and end
effector position are marked with circles and a line indicating
orientation.  Double clicking on the map moves the end effector to the
appropriate location; verify that the robot's arm moves when you
click.

### Teleoperation

Next, go back to the screen session and change to the Ein console
window by typing `` `1 `` for the right arm or `` `2 `` for the left
arm.  You should see a view like this:

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

![Robot in Home Position](../assets/baxter_crane.jpg)

Try the following commands for moving to other canonical
positions.  (Be careful that the robot's workspace is clear.)

```
assumeCrane1
assumeBackScanningPose
```

Move back to home position: `goHome`. 

You can also incrementally move the arm in the global x, y, and z
frames.  For example, try:
```
xUp
xDown
```

Similarly, `yUp`, `yDown` moves in the Y frame, and `zUp` and `zDown` move
the arm vertically.

By default, the robot is keyed to move in 1 cm
increments.  You can change the increment size to 5 cm by issuing:
```
0.05 setGridSize
```

Note that the repl uses a post-fix FORTH-like language called Back.
So the argument 0.05 comes before the function (called "word"
following Forth conventions), named setGridSize.

The grid size is specified in meters. To go back to 1 cm,
call
```
0.01 setGridSize
```

Next try running `openGripper` and `closeGripper`.  You should now be
able to teleoperate the arm to pick up an object that is axis-aligned
with the gripper.   

#### Exercise: Teleoperate some picks.

Teleoperate the robot to pick up a few objects.  Try doing this first
    by watching the arm.  Then try putting your back to the robot and
    use only information from the wrist camera image.

waitUntilAtCurrentPosition


#### Exercise:  Teleoperate some non-axis aligned picks.

You may have noticed in exercise 1, that we have not yet given you
commands to rotate the gripper.  `oZUp` (and its related words `oXUp`
and `oYUp`) rotate the gripper in the corresponding frame by one
degree.  This amount of rotation is quite small, so you will need to
issue many of these commands.  To simplify this process, we created a
word that duplicates words on the stack.  You can run: `( oZUp ) 10
replicateWord` to run `oZUp` 10 times (or whatever compound word is in
parentheses.)  Note that tokenization is based on white space so you need a space before and after every parentheses.

### Fun Words

Ein contains many many other words to control all parts of the robot.
Some fun ones to try are `torsoFanOn`, `torsoFanOff`, `torsoFanAuto`
(the default).  You can also try `lightsOn` and `lightsOff`,
`happyFace`, `sadFace`, and `neutralFace`.

#### Exercise: Blink the lights. 

Write a program to blink the lights a few times.  You will need to wait in between each execusion by running `1 waitForSeconds`.

>! ( torsoFanOn 1 waitForSeconds torsoFanOff 1 waitForSeconds ) 10 replicateWord

### Timing

By default words run as quickly as possible, without waiting for
actions to complete.  There are a variety of words that wait until
certain conditions are met.  Most useful is
`waitUntilAtCurrentPosition`.

#### Exercise:  Wave the arm.

Writing a program to move the arm back and forth a few times in a row
using `waitUntilAtCurrentPosition` and `replicateWord`.


