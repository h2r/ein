---
layout: page
title: ROS
permalink: /ros/
order: 7
---

Ein integrates with ROS (Robot Operating System).  It takes input on
feeds such as ROS Image topics.  It outputs on standard ROS topics
such as
[object_recognition_msgs](http://wiki.ros.org/object_recognition_msgs).

Ein does not have to use ROS; indeed one of the first robots we
integrated it with besides Baxter was the Aibo, and it connects over a
TCP/IP connection.  However data structures such as ros::Time and the
like are embedded into Ein's code, so it must have a ROS environment
to compile, currently ROS Indigo.


### Sending Commands to Ein

You can send commands to Ein by publishing on the topic
`/ein/right/forth_commands` or `/ein/left/forth_commands` for each
hand.  These topics take a std_msgs/String which is a valid Back
program.  You can use any ROS program to publish to this topic,
including `rostopic pub`.  For an example, see `sendToBothArms.sh`
which sends commands to each arm.



Document color calibration


Write about the file system and where objects are stored

which one is focused? 

setTargetClass (but change it to focused class)

Write a word to map the whole table


tableMap

add pictures of predicted map


scenePredictBestObject tableUpdateMaps



To add objects to a map:

tableTakeScene (move the arm to take the map)
scenePredictBestObject  (predict the best object) 
tableUpdateMaps (visualize what you predicted)


(random things I don't understand that seem to result in a recognized object array being published)
0 sceneMapSceneObject
recordPreTargetLock
recordPostTargetLock
publishRecognizedObjectArrayFromBlueBoxMemory (rename this)

Topic:  /ein_right/blue_memory_objects


Bug: 
predict orientation in recognized object array


Write about Ein's architecture (ros, not ros, qt, everything else)
