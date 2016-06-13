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