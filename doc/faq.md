---
layout: page
title: FAQ
permalink: /faq/
order: 100
---

* TOC
{:toc}



### What if my console stops updating?

Sometimes the console stops updating.  This problem can occur if you
reboot Baxter without restarting the console as well.  It can also
happen if you restart Ein (for unknown reasons).  To fix it, try
restarting the console.  Type ``` ` ` :quit ``` (note the double screen
escape character, due to the subscreen inside the main Ein screen).
Then go up one command in the command history to restart the screen
session.


### What if the robot won't move?

First verify you are receiving state messages; the state window in the
console should be updating as the robot's pose changes.  If not, you
may be connecting to the wrong arm. Make sure if you are running on
the left arm, you are using the left console window.  Alternately,
your ROS environment may be misconfigured.  Verify that Baxter
commands such as ` rosrun baxter_tools enable_robot.py -s` work
correctly, and that `rostopic list` and `rostopic echo` work.  A
common problem is that the $ROS_IP xor $ROS_HOSTNAME environment
variables are not correctly set.  You should verify that one of these
is set in the client machine, and that your Baxter can ping your
client machine using the exact IP address/hostname that is set via the
environment variable.  One might think that a networking
misconfiguration would result in no messages being sent, but in fact,
often your client machine will receive messages from Baxter, but not
be able to send messages, if your $ROS_MASTER_URI is correct, but not
your $ROS_HOSTNAME or $ROS_IP.


Verify that you are not in zero gravity mode by checking the status
window; it should be set to 0. Type `zeroGOff` to turn off zero
gravity.  You can also press the grey circular button on the arm to
toggle zero gravity (the "Ok" button).



### What if the wrist camera image is not updating? 

Verify that your ROS environment is set up correctly ($ROS_IP,
$ROS_HOSTNAME).  For some reason, when these variables are not set,
the client often receives one wrist camera image, and then not any
others.  (One would think it would receive zero images or all of them;
but consistently it receives one and then stops.  We are unsure why,
but it happens all the time when the network is not set up correctly.)


### What if Ein is not receiving wrist camera images? 

The Baxter robot only allows two of the three cameras to be open at
one time: left hand, right hand, and head.  This is due to limitations
in the badwidth of USB 2.0.  More information about this is available
[here](http://sdk.rethinkrobotics.com/wiki/Camera_Control_Tool).

Ein tries to make this happen, but sometimes the robot doesn't listen
to camera control commands. We think this is a Rethink problem, but we
haven't set up a good bug report.

So before starting Ein, you should use camera_control.py to make sure
that the camera you want to use with Ein is open (left or right). You
should close the head camera and open the other two. Sometimes it
takes several tries to get the robot to do this.

This command lists the cameras:
```
rosrun baxter_tools camera_control.py  -l
```

Run `camera_control.py` with no arguments to see usage.  Open and
close the cameras until only the left and right hand cameras are open.
When it is set up correctly you should see this:  

```
$ rosrun baxter_tools camera_control.py  -l
head_camera
left_hand_camera  -  (open)
right_hand_camera  -  (open)
```

Sometimes we have needed to reboot the robot to make this go away.
We have not been able to figure out why this happens or a reliable
reproduce.  It happens fairly rarely for us, but it is annoying when
it does.


### Ein appears to be frozen, with no movement, and the gui screen darkens and freezes.

Ein is a single threaded program, which means that robot callbacks,
inferences, and gui callbacks are all serviced with a single thread.
As a result, when computationally intensive tasks are being run, the
gui will freeze.  This effect occurs during calibrating the height
reticles, as well as when running `fillIkMapAtCurrentHeight`.  One way
to see what is happening when this occurs is to run "Ctrl-C" to Ein in
gdb and backtrace, to see what part of the code is getting stuck.  



### What is the difference between `assumeBeeHome` and `goHome`?

The `assumeBeeHome` word sets the target position to the home position
but doesn't wait.  The `goHome` blocks until the arm arrives at the
position.  `goHome` does other good stuff such as "shoring up" so the
arm is in a nice crane pose. `goHome` is what you should use for most
things.



### How do I save a home position longer than a session? 

You can put arbitrary commands in the file init.back, which will be
executed whenever Ein starts up, after all other initialization is
complete.

### I am getting strage TF errors about "Lookup would require extrapolation into the past" or "Lookup would require extrapolation into the future."    

This error most often means that Baxter's system time is out of sync
with client computers.  Note that Baxter syncs its time using ntp to
pool.ntp.org, and this server cannot be changed (as per Rethink's
instructions).  We have had issues at sites that block pool.ntp.org,
causing Baxter to stop updating its time and fall out of sync with
other computers that were updating their time.  We had to have our IT
staff open a special port in their firewall to allow Baxter to sync
up. 



