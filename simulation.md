---
layout: page
title: Simulation Mode
permalink: /simulation/
order: 7
---

Ein has a simulation mode that allows it to run without being
connected to any robot.  This mode can be used to view data from the
Million Object Challenge and render photographs.  To start Ein in
simulation mode, first change "physical" to "simulated" in the command
line arguments.  This will cause Ein to start with fake sensor data
from Baxter.  It will generate fake camera images and poses so that
various parts of the system will not crash.

Here is an example:  `catkin_make && gdb --args ./devel/lib/ein/ein  physical left`. 

You will need to start your own ROS core: `roscore` in another
terminal, and make sure that your ROS_MASTER_URI is set appropriately.
If you run the screen session with no arguments, it will default to
localhost so everything should work.  If you encounter problems, most
likely your $ROS_MASTER_URI or $ROST_HOSTNAME or $ROS_IP are set
incorrectly.
