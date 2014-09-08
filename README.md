oberlin_detection


Make sure you have the following standard components installed:

opencv
cv_bridge
image_transport
openni

Install the non-standard package h2r/bing.

To run:

```
roslaunch openni_launch openni.launch depth_registration:=true
rosrun oberlin_detection oberlin_detection
```

In the future there will be three executables for handling separate tasks:

```
rosrun oberlin_detection save_blueboxes file_prefix:=collection_round_11
rosrun oberlin_detection save_detections file_prefix:=collection_round_11
rosrun oberlin_detection publish_detections
```

In the future, the program will snoop the classCrops folder and base object categories on what it finds.
For now, classes are manually specified in the API.

