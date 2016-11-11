---
layout: page
title: Movement
permalink: /movement/
order: 6
---


The robot maintains its true pose in the variable `truePose`.  Its
current target pose is `currentPose`.  It always drives its true pose to
its target pose, and generally converges to within a millimeter.  

You can push the current pose on the stack, then move the arm, and
then assume the pose with `moveEeToPoseWord`.  For example, the
following program pushes the current pose, moves the arm to crane
pose, then moves back to the current pose:

```
currentPose assumeCrane1 waitUntilAtCurrentPosition moveEeToPoseWord
```

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
The properties `w1GoThresh` and `w1AngleThresh` control how close
`waitUntilAtCurrentPosition must get before deciding it is at the
current position; they have equivalent setters.  


#### Exercise: Waiting

Change `w1GoThresh` and observe how close `waitUntilAtCurrentPosition`
needs to get while stopping.  While changing this value, set
`w1AngleThresh` to a very large number to avoid conflating position
and angle.  For very small values for `w1GoThresh`,
`waitUntilAtCurrentPosition` may never return.

Default value:
`0.003 setW1GoThresh zUp waitUntilAtCurrentPosition`  

Takes noticably longer: 
`0.0005 setW1GoThresh zUp waitUntilAtCurrentPosition`

Never terminates:
`0.000005 setW1GoThresh zUp waitUntilAtCurrentPosition`

### Grid Size

The grid size determines how much the arm moves for the various
commands like `xUp`.  It is set in centimeters, so you can use `0.01
setGridSize` so that each `xUp` moves 1 centimeter, or `0.1
setGridSize` so that each `xUp` moves ten centimeters.

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

#### Exercise:  Try different speeds. 

Try moving the arm up and down with different movement speeds and grid
sizes.  


### Changing the Home Position

You can change the home position with build in getter and setter commands.  Remember to
set separate values for each arm.  First drive the arm to the desired
new position. We recommend using a point in the green part of the IK
workspace so the arm can move freely around the home position for
servoing and mapping and the like. Issue

``` 
beeHome print
currentPose setBeeHome
beeHome print
currentPose print
``` 

and then move the arm somewhere else. Run

```
currentPose print
assumeBeeHome
currentPose print
```
and verify the output is what you would expect.


### Force feedback

Ein can use the robot's torque sensors to press down until it senses
force on the table.  To do this feedback loop, move the gripper until
it is at a position as close as you can manage to the object.  Moving
down is slow, because the robot must sense torque at each step.  The
robot will gradually move down until it senses a force.  To call the
"all in one" word, use `pressUntilEffortAllInOne` which first comes to
stop, then initializes variables, then slowly moves until it touches.
It will then back up slightly to unwind the springs and then gently
move down to touch the table.  It uses `localZDown` to move, so make
sure it is in crane pose if you wish to touch the table.


The word `pressUntilEffortCombo` touches the table coarsly, then backs
up and presses again until it is lightly touching the table.  It must
be called after calling `pressUntilEffortInit` which sets various
variables needed for `pressUntilEffort`.  You can call
`pressUntilEffotInit` to set up default values, and then individually
set specific things, such as the movement speed.

The following parameters are set in `pressUntilEffortInit`:

* `setSpeed` which sets the arm's movement speed.
* `setGridSize` which sets the grid size during movement.
* `setW1GoThresh` which is the distance that `waitUntilCurrentPosition` waits to be within before deciding it is at the current position.
* `setEffortThresh` which is how much force will trigger a detection of effort.

#### Exercise:  Press on different surfaces.

Use `pressUntilEffortAllInOne` to press on different surfaces, and
observe how much force is needed for various actions.  Change
`effortThresh` and observe how that affects the force.


#### Exercise:  Explore the press until effort parameters

Use `pressUntilEffort` and `pressUntilEffortInit` to explore the
various parameters.  Try different speeds, grid sizes, and W1
thresholds and effort thresholds.