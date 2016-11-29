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
