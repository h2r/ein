---
layout: page
title: Calibration
permalink: calibration/
order: 2
---

Next we will calibrate the robot so that it can map between image
coordinates and world coordinates.  This process takes several
steps. 

### Make a magic circle
First, print out about 10 pages of [magic paper](
https://github.com/h2r/ein/raw/master/images/calibration/magicpaper.pdf).
Send the gripper to the home position by running `goHome`. (If you
would like to change the home position, follow [these
instructions](../movement/#changing-the-home-position).)  Then drive
down to close to the table height using `zDown`.  Create a magic
circle underneath the gripper as depicted here:

![Magic Circle](../assets/magic_circle.jpg)

The goal is for the magic circle to fill the wrist camera's field of
view as the arm moves up.  The magic paper contains a superposition of
three plane waves at different orientations and colors.  It is
textured at every point, allowing the robot to set its calibration
parameters by making known movements.

### Set the table height

Ein assumes there is a planar horizontal table and stores the height
of this table for use in various calculations and motions.  It can set
the table height using the robot's range sensor. To set the table
height, first `goHome`, then use `zDown` to move the arm so that the
IR sensor is approximately 10cm from the table.  (The short grippers
should be a centimeter or two above the table.) Run `setTable` to use
the IR sensor to set the table height.  You should see the console
print information about the process; it takes multiple readings and
waits for them to stablize.  At the end it will print the table
height.  The numbers should be changing slightly, and the final delta
should be small.

### Set the camera parameters

Baxter's wrist camera uses automatic adjustment of gain, exposure, and
color settings.  However these automatic adjustments prevent our
system from reliably detecting objects it has previously scene.  Our
solution is to set them once to good values and fix them.  To set these parameters, start with this command:

```80 25 1124 1024 2048 fixCameraLighting```

The parameters are:
```<gain> <exposure> <red> <green> <blue> fixCameraLighting```

First tune the gain, then the exposure, and then adjust the color if
necessary.  You can set it to the automatic parameters with
`fixCameraLightingToAutomaticParameters`.  This word will set the
camera to automatic control, then fix the parameters to the
automatically adjusted values.  You can observe these values with the
variables `cameraGain`, `cameraExposure`, `cameraWhiteBalanceRed`,
`cameraWhiteBalanceGreen`, `cameraWhiteBalanceBlue`.    

To save these settings to disk (so they will be loaded automatically every time Ein starts), run:
```
saveCalibration
```

An example of a
reasonably well adjusted (slightly yellow) wrist view image of magic
paper is here:  

![Wrist View](../assets/wristview.jpg)

### Set the gripper mask

Ein needs to know the location of the grippers in its wrist camera
image in order to ignore those pixels when mapping its tabletop
environment.  To set the gripper mask run `setGripperMaskWithMotion`.
View the progress in the window "Object Viewer."  At the beginning the
window will be blue; as it moves, high-variance parts of the averaged
image are considered not part of the gripper mask; low variance parts
are considered as part of the mask.  You should see something like
what appears below:

<img src="../assets/grippermask1.jpg" width="225" alt="Gripper Mask 1">
<img src="../assets/grippermask2.jpg" width="225" alt="Gripper Mask 2">
<img src="../assets/grippermask3.jpg" width="225" alt="Gripper Mask 3">


Verify that the blue region completely covers the grippers when they
are open, and that there is no extra blue areas remaining.  If this
does not work, recenter the gripper (`goHome`) and try again.  You may
need to rearrange the magic circle or adjust the lighting or colors on
your camera.  When the mask looks good, you can stop the process by
running `clearStacks`.  It saves after every iteration.



### Calibrate the height reticles

Next we need to set the projection of the gripper in the image at
different heights.  This allows Ein to map between pixel space and
global space given the camera pose.    We set the magnification using
interpolation based on several sample points.  The target is obtained
by rotating the gripper in a plan and finding the minimum variance
point at each point.  

To do this process first `goHome` and then run:
```
calibrateRGBCameraIntrinsics
```

The gripper will move to four different heights and take a series of
measurements at different orientations.