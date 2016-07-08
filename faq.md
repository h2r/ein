---
layout: page
title: FAQ
permalink: /faq/
order: 7
---

### What if my console stops updating?

Sometimes the console stops updating.  This problem can occur if you
reboot Baxter without restarting the console as well.  It can also
happen if you restart Ein (for unknown reasons).  To fix it, try
restarting the console.  Type ``` ` ` :quit ``` (note the double screen
escape character, due to the subscreen inside the main Ein screen).
Then go up one command in the command history to restart the screen
session.


### What if the robot won't move?

Verify that you are not in zero gravity mode by checking the status
window; it should be set to 0. Type `zeroGOff` to turn off zero
gravity.  You can also press the grey circular button on the arm to
toggle zero gravity (the "Ok" button).

This problem also occurs if your ROS environment is misconfigured.  A
common problem is that the $ROS_IP xor $ROS_HOSTNAME environment
variables are not correctly set.  You should verify that one of these
is set in the client machine, and that your Baxter can ping your
client machine using the exact IP address/hostname that is set via the
environment variable.  One might think that a networking
misconfiguration would result in no messages being sent, but in fact,
often your client machine will receive messages from Baxter, but not
be able to send messages.  


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

Sometimes we have needed to reboot the roboto to make this go away.
We have not been able to figure out why this happens or a reliable
reproduce.  It happens fairly rarely for us, but it is annoying when
it does.