---
layout: page
title: Poses
permalink: /pose/
order: 7
---


Ein supports pose words, for example `currentPose`, which contains the
targetted pose for the end effector, and `truePose`, which contains
the last return for the real pose of the end effector.  It also
contains words for pose getters and setters, such as `setEEPose*` and `eePose*`,
which returns and sets the px, py, pz (position) and qx, qy, qz, qw (quaternion)
for the pose values.

For example,

`trueEEPose eePosePX` pushes the double value of the x position
coordinate of the end effector.  


`currentEEPose 1 setEEPosePx` pushes a new pose on the stack whose px
value is the double value `1`.

`moveEEToPoseWord` moves the end effector.  So a workflow for many
applications is to create poses somehow (from object locations; from
annotations) and then send the end effector to these poses.

#### Exercise:  Wave!

Write a program that makes Baxter wave, by moving back and forth from
the current starting pose.  So the robot should move repeatedly back
and forth relative to its starting position.  You will need to use the
`truePose`, as well as the getters and setters above.  You may also
find the `dup` word useful, which duplicates the current value at the
top of the stack, `replicateWord` to repeat an action more than once,
and `store` to store pose words as variables.

Select to see our answer:


truePose "x" store <br/>
x dup eePosePX 0.1 + setEEPosePX "y" store <br/>
( x moveEeToPoseWord waitUntilAtCurrentPosition y moveEeToPoseWord waitUntilAtCurrentPosition  ) 5 replicateWord
{: style="color:white;" }


