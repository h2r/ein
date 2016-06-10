---
layout: page
title: Calibration
permalink: calibration/
order: 2
---

In your second ten minutes with Ein, we will calibrate the wrist
camera on one arm.

First, print out about 10 pages of [magic paper](
https://github.com/h2r/ein/raw/master/images/calibration/magicpaper.pdf).
Send the gripper to the home position by running `tempGoHome` Then
drive down to close to the table height using `zDown`.  Create the
magic circle underneath the gripper as depicted here:

![Magic Circle](../assets/magic_circle.jpg)

The goal is for the magic circle to fill the wrist camera's field of
view as the arm moves up.  The magic paper contains a superposition of
three plane waves at different orientations and colors.  It is
textured at every point, allowing the robot to set its calibration
paramters by making known movements.

Next calibrate the table height.  Run `setTable` to use the IR sensor
to set the table height.