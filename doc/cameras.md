---
layout: page
title: Cameras
permalink: cameras/
order: 20
---

Ein supports any number of cameras attached to an arm.  Each camera
has associated calibration parameters, a stream buffer, and can be
used to create synthetic photographs.  You can see `kinect2.back` for
example commands that add the Kinect 2 as an additional camera.  By
default, the Baxter wrist camera is configured for each arm, but if
you mount other cameras on the arm, they can also be used to observe
the scene.

The word `cameraCreate` adds a camera, specifying its ROS image topic,
the name for the camera, and its TF link.


There is a focused camera which represents the current active camerea.
The variable `cameraName` has the name of the current focused camera.
You can run `incrementCamera` to change the camera and it will change
what is viewed in the wrist view, the stream buffer, etc.

For the Kinect 2, each image stream is treated as a separate camera in
Ein: the RGB camera, the IR camera, and the depth return.  In the
cameras class, `cam_img` contains the raw image as received from a ROS
topic (grayscale if that is what you got from the camera.)
`cam_bgr_img` contains the image converted to bgr (even if it started
out grayscale), and `cam_ycrcb_img` contains the image as ycrcb.