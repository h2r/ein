---
layout: page
title: Movo
permalink: /movo/
order: 98
---

Ein contains an interface to connect to and program the Kinova
[MOVO](http://www.kinovarobotics.com/innovation-robotics/movobeta/).
To install, create a new workspace with Ein, and the Kinova packages,
following the [Kinova Setup
Instructions](https://github.com/Kinovarobotics/kinova-movo/wiki/Setup-Instructions).

### Installing
Ein will automatically detect the Kinova MOVO packages and compile the
MOVO library.  You should see a statement to this effect when you run
`catkin_make` in the workspace.  If Ein defaults to some other
library, this is because it detected another robot first in the
ROS_PACKAGE_PATH, such as Baxter or AIBO.  In that case, either change
the ROS_PACKAGE_PATH to exclude these other packages.  Alternatively,
edit the Ein CMakefile to only build the MOVO package.  (There is an
`if` statement to decide what library to build; just change it to
always build MOVO.)

Copy the setup.sh file to the root directory of the workspace.  Then
run `ROBOT=movo2 screen -c src/ein/ein_movo.screenrc`.  This command
will start a screen session that is initialized with the relevant
commands and packages.  It will also set the ROS_MASTER_URI to
`movo2`.  You may need to set your ROS_IP or ROS_HOSTNAME depending on
how your network is configured.

### Get Started

The relevant words are defined in ein_movo.cpp.  Useful ones:
`movoKill` kills the robot (as if you hit the kill switch).  The full
list of all Ein words can be seen in the [words](Word Index).  Example
programs using the movo functionality can be seen in movo.back.  Below
is a brief high-level overview.

The two words `moveToTuck` and `moveToHome` move the upper body to the
home and tucked position respectively.  The words `torsoUp` and
`torsoDown` move the torso up and down, controlled by `torsoGridSize`.
The words `panUp`, `panDown`, `tiltUp` and `tiltDown` change the
pan/tilt angle by the amount `panTiltGridSize`.

### Arms

Ein has a concept of a focused end effector (either the left or right
arm). Ein also has a concept of zero gravity.  When zeroGToggle is on,
Ein does not set the pose and continuously updates the end effector
position of the focused end effector.  (It does not update the
not-focused end effector, although we could change this.)  You can
change the focused end effector with `incrementEndEffector`.  Ein has
a rich library of words for interacting with the focused end effector,
such as `xUp`, `xDown`, `yUp`, `yDown`.  All movement words, such as
`xUp`, `localXUp`, `oXUp` and `waitUntilAtCurrentPosition` work to
move the focused end effector using MoveIt.  Many of the tutorial
instructions for the [[getstarted]] page apply directly to MOVO to
move the end effector.

### Gripper

The gripper is sort of complicated. Our Movo has the Jaco kg3 gripper.
So there is a joint states topic that has the three finger positions.
This is available in Ein as `finger1JointPosition`,
`finger1JointVelocity` and `finger1JointEffort`, for each of the three
fingers.  However there does not appear to be a Movo topic to move the
finger joints individually (yet).  I have asked them about this on
github.

What does exist is an action client that opens and closes the gripper,
treating the gripper state as a single double, gripperPosition.  This
position is computed from the joint angles from two of the three
fingers and ranges between 0.01 and 0.165.  These are all defined in
the movo stack in `movo_jtas.py`.  Ein clones these constants as well
as the function that computes the double value you use to command the
gripper and has a `gripperPosition` reactive variable that reflects
the current gripper position, computed from the finger joint states.

Using these primitives, it defines `openGripper` and `closeGripper` as
words, just like Baxter.

### Moving the Base in the Odom Frame

To move the base in the odometry frame, you must first run `baseGoCfg`
to send the configuration command to move the base.  All these words
begin with the string `base`.  `0.1 baseSendXVel` sends one twist
message with 0.1 in the `x` direction and 0 in all other fields.  The
words `baseSendYVel` sends velocities in the `y` direction.  To rotate
the base use `0.1 baseSendOZVel` which sends a twist message with the
angular speed.  Finally, `lx ly lz ax ay az baseSendTwist` sends a
complete twist message.  (Note that `ax` and `ay` are probably ignored
since the base is constrainted to move in the plane.)


### Moving the Base in the Map Frame

This is not yet supported, but we plan to add this.  The plan is to
make the base a third focused end effector, with the true pose
obtained from the pose in the map frame, and the target pose set via
the end effector movement words like `xUp` and `xDown`.

### Movo Status

There are a variety of reactive variable words that record the robot's
current status.  For example, `batterySoc` contains the battery's
current state of charge from the associated ROS topic.  It is
continuously updated by reading the ROS topic, so whenever it is
accessed, it will automatically reflect the most recent value.  (This
aspect is what makes it a reactive variable.)  There are undoubtaly
more such words that should be added.  If more are needed, it is
straightforward to add a new subscriber and new reactive variable,
following the existing examples.  Feel free to send a patch.

#### Moveit Configuration

There are a variety of words to read the current MoveIt state (and a
smaller number of setters).  These all begin with the string `Moveit`
and refer to the Move Group associated with the current focused end
effector.  For example, `moveitPlanningFrame` is the string of the
current planning frame.  The word `moveitGoalPositionTolerance` is the
tolerance on the goal position, etc.

### Sound

The `say` word takes a string and sends it to the sound_play action.
It would be easy to add more words to more directly control the sound
playback, but all that exists now is `say`.  For example `"Hello
world" say` causes MOVO to say, "Hello world."

### Light Fields

We plan to connect the Kinect 2 camera to Ein in order to create light
field synthetic photographs using Movo.  The problem is the frame rate
when offboard over wireless is quite slow, so this feature is a pain
to add.


### TODO

* open and close the gripper; set joint targets and positions for the fingers.

* light fields with the kinect 2. (Our kinect 2 frame rate is realllllly slow.)

* better base movement, so you can do it with the map.

* moveit seems to only work if I start moveit by sshing into movo2 and
 running `roslaunch movo_7dof_moveit_config
 movo_moveit_planning_execution.launch debug:=true` I am not sure why
 this should be since moveit starts when the base starts, and seems to
 work as part of the powerup sequence.

* higher-level words like turn around.

* figuring useful "default" poses for the arms for various tasks.

* integrating better with moveit.  

* Try to teleop opening a door.

* Try to teleop playing the ukulele.

