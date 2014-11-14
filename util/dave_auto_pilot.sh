#!/bin/bash

rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
#rosrun baxter_tools camera_control.py -c head_camera

rosrun baxter_tools camera_control.py -o right_hand_camera -r 640x400
rosrun baxter_tools camera_control.py -o left_hand_camera -r 640x400

array=( "capture_object" "publish_detections" "capture_annotated_objects" "capture_hard_class" )
for i in {0..3}
do
  echo ${array[i]}
  rosparam set /${array[i]}/gray_box_left 150
  rosparam set /${array[i]}/gray_box_right 150
  rosparam set /${array[i]}/gray_box_top 10
  rosparam set /${array[i]}/gray_box_bot 60
  rosparam set /${array[i]}/threshold_fraction .5
  rosparam set /${array[i]}/density_decay .3
done


for i in {0..3}
do
  echo ${array[i]}
  rosparam set /${array[i]}_left/gray_box_left 150
  rosparam set /${array[i]}_left/gray_box_right 150
  rosparam set /${array[i]}_left/gray_box_top 10
  rosparam set /${array[i]}_left/gray_box_bot 60
  rosparam set /${array[i]}_left/threshold_fraction .5
  rosparam set /${array[i]}_left/density_decay .3
done


for i in {0..3}
do
  echo ${array[i]}
  rosparam set /${array[i]}_right/gray_box_left 150
  rosparam set /${array[i]}_right/gray_box_right 150
  rosparam set /${array[i]}_right/gray_box_top 10
  rosparam set /${array[i]}_right/gray_box_bot 60
  rosparam set /${array[i]}_right/threshold_fraction .5
  rosparam set /${array[i]}_right/density_decay .3
done
