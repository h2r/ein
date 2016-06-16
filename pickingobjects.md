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

A consequence of these assumptions is that we can forgo labeling grasp depth
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





Now you need to train a workspace background model.  Move the arm to a
tentative workspace. The workspace should contain a square of two feet. It
should be as flat as possible. The system can accomodate altitude changes but
shadows and occlusion induce variance in the maps and require careful sampling
for accurate results.

Move the arm to the center of the workspace. Issue

```
1 changeToHeight
shiftIntoGraspGear1
currentPose "zzAPlace" store
```

The last line stores your position in a variable named "zzAPlace". Prepending
variables with "zz" can be useful because it will show up at the end of the tab
complete list, so you can more easily remember which variables have been defined this session.
You can return to this, which will be the center of your workspace, by issuing
```
zzAPlace moveEeToPoseWord
```


The wrist should now be pointing straight down with the camera should be about
38 cm from the table.  If it is substantially higher or lower and this is the
same space in which you calibrated, you should reset your table height and make
sure you saved your calibration.

As usual, unless otherwise stated, leave the grippers in open position.

After making sure any object that can move is free from the workspace and that the lighting is how
you will want it during picking, issue

```
tableUpdateBg
```

The arm should move in a spiral, return to its starting position, your CPU should 
go under load, and a few seconds after the CPU relaxes the formed image should show up
first in the Observed Map window and next in the Background window. The table and objects
shorter than a few centimeters should appear crisp up to the resolution of the map, which
is 1mm to a pixel at this height with a good calibration. Object taller than a few centimeters
will start to blur due to defocus. 


When picking objects, you can 
change the gripper gap and retake the gripper mask by pointing the arm at magic paper as during calibration 
and running

```
setGripperMaskWithMotion
```

Make sure the gripper gap is at its narrowest setting. Equip the rectangular rubber tips; we will be trying
for a flush grasp.  Find the allen wrench from a Baxter parallel gripper kit and place it in the workspace.
Issue

```
endArgs "allenWrench" setClassLabels
```

and check terminal and console output to verify the model loads. 

```
tableInfiniteDribbleBest
```

That should pick a bunch.



Next, find an object you want to pick, and make sure it fits in the grippers. 
Place the object at the center of the workspace and return the arm as well with 

```
zzAPlace moveEeToPoseWord
```

Clear the space of any other object and run

```
tableQuickScan
```

This generates an object model capable of localizing the object once moved and rotated within
the plane of the table. Now you need to annotate a grasp. Issue

```
tableLock3dGrasp
```
and wait for it to finish. Once the stack is clear, by whatever means you choose
and without touching or moving the object from its locked position, drive the arm to a position which will result in a valid
grasp when the fingers close. Recall that the arm will move to a pre-pick position and advance towards the object until it encounters
resistence, at which point the grippers will close. Keep this in mind when choosing a grasp. If you move the object, `clearStacks` and 
start over from `tableLock3dGrasp`.

Issue

```
clearClass3dGrasps
``` 
to clear the grasps for this object, removing any default or previously
assigned grasps. If you add a bad grasp or knock the object and have to start
over, make sure to clear your grasps.

When you are ready to add the current pose as a grasp, issue

```
add3dGrasp
``` 
You can and should add multiple grasps so that, if the first grasp or pre-grasp position is infeasible, Ein can select
an alternative grasp.

Now it's time to test the object. Return to your workspace center and run 

```
tableInfiniteDribbleBest
```




