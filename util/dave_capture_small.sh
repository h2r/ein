#!/bin/bash

array=( "capture_object" "publish_detections" "capture_annotated_objects" "capture_hard_class" )
for i in {0..3}
do
  echo ${array[i]}
  rosparam set /${array[i]}/gray_box_left 150
  rosparam set /${array[i]}/gray_box_right 150
  rosparam set /${array[i]}/gray_box_top 50
  rosparam set /${array[i]}/gray_box_bot 150
done


