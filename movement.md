---
layout: page
title: Movement
permalink: /movement/
order: 6
---


The robot maintains its true pose in the variable `truePose`.  Its
current target pose is currentPose.  It always drives its true pose to
its target pose, and generally converges to within a millimeter.  

You can push the current pose on the stack, then move the arm, and
then assume the pose with `moveEeToPoseWord`.  For example, the
following program pushes the current pose, moves the arm to crane
pose, then moves back to the current pose:

```currentPose assumeCrane1 waitUntilAtCurrentPosition moveEeToPoseWord```

When Ein is in zero gravity mode, the current position is constantly
set to the true position, so the arm stays where you move it.
Otherwise, if the arm is moved, it will move back to the current pose.
You can toggle zero gravity with `zeroGOff` and `zeroGOn` or pressing
the circular grey button on the arm (the "OK" button).


### Stopping

`waitUntilAtCurrentPosition` causes the robot to pause until the end
effector is within a threshold of its target position.  `comeToStop`
will pause until the robot has actually reached a stop (waiting until
the current position has stabilized to within a threshold).

### Movement Speed

Try moving a long distance, like 20 cm. How fast does the robot move?
To move slowly, issue

```
0.05 setSpeed
```

The speed is specified with a number between 0.0 and 1.0. There are
some shortcut words for certain speeds:

```
fullImpulse
halfImpulse
quarterImpulse
tenthImpulse
```

Large movements should generally be carried out in `quarterImpulse` or
`tenthImpulse`.  Speeds greater than or equal to `halfImpulse` or
`0.5` should be used with caution; even the safest robot demands
attention.


### Changing the Home Position

You can change the home position by editing init.back.  Remember to
set separate values for each arm.  First drive the arm to the desired
new position. We recommend using a point in the green part of the IK
workspace so the arm can move freely around the home position for
servoing and mapping and the like.  Then run truePose to obtain a
value on the stack for the desired pose.  Copy this pose from the
console into init.back.  Be sure to reload init.back to see the new
pose by running `"init" import`.

