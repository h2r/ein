---
layout: page
title: Picking Objects
permalink: /pickingobjects/
order: 3
---

Now we will talk about tabletop picking. Restricting our picking to a table
about Baxter allows us to make some assumptions about the world. In particular,
that there is a flat ground surface parallel to the xy-plane whose constant z
coordinate is known and that gravity pulls toward that plane. 

A consequence of these assumptions is that we can forgo labeling grasp d1epth
and close the fingers when they encounter an obstruction. When moving toward an
object for a pick, stabbing our fingers outward from a pre-pick position, our
fingertips will either press the object against the tabletop and trigger a
grasp or go around the object (spoon handle, small block) and hit the table on
either side of the target, triggering a grasp. Grasping in free space without a
backing or resistor is less reliable because tipping is more likely to occur.

Another consequence is that most objects will come to rest on the ground plane.
This is important because in order to see an object correctly, Ein needs to know
(or finds out in the process) how far away from the camera it is.  Most computer
vision systems use individual RGB frames, or calculations performed on sequences of
frames and summarized with statistics, to perform their operations. Ein captures
tens to thousands of RGB frames and refocuses the rays from those images into
computed 2D photographs and performs operations on those instead. Refocusing the light
is very similar to adjusting the focus on traditional camera, and the aperture of 
our virtual camera is quite large, which means that we can achieve a very narrow
depth of field. Knowing the position of the ground plane is a strong hint for the 
search problems involved and so speeds up the process dramatically.





The basic workflow for object picking is:

( calibrate once )

Train workspace background  model
Train object model
  Automatic grasp annotation
    Manual grasp annotat

( change workspace or recompute background if table shifts or lighting changes )

Map workspace and compute discrepancy (background subtraction)
Detect object(s) cascaded on discrepancy

( perform other robotic actions )

Pick objects based on detections.



Now you need to train a workspace background model.



