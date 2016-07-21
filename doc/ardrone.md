---
layout: page
title: AR Drones
permalink: /ardrone/
order: 7
---


This page contains instructions for using Ein with the AR Drone platform.

First you need to add the [AR
Drone](http://wiki.ros.org/ardrone_autonomy) ros packages to your
workspace where Ein is installed:

```git clone https://github.com/AutonomyLab/ardrone_autonomy```

You will need to install dependencies: `apt-get install
ros-indigo-vrpn-client-ros libsdl1.2-dev daemontools libiw-dev`.

Run:
``` rosrun ardrone_autonomy ardrone_driver  ```

``` roslaunch vrpn_client_ros sample.launch server:=192.168.1.2```

(But we will take this one out:)
``` rosrun great_ardrones pid_controller ```



Motion capture topic:
/vrpn_client_node/ardrone/pose

In order to use this package, use [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros) and name the rigid
body that represents your AR Drone "ardrone".



To run ein, use "ardrone" instead of "left" or "right":

```
catkin_make && gdb --args ./devel/lib/ein/ein  _data_directory:="$(rospack find ein)/default" physical ardrone
```


Similarly to use the console, use `` ` `quit `` to quit the screen
session that is running the console.  (Note that you must escape twice
to quit the inner screen.)  Then to restart the console pointing at the AR Drone topics run:

```
ARM=ardrone screen -m -c src/ein/ein_client.screenrc
```
