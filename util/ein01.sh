#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix $chirality

#rosrun node publish_detections _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="" _left_or_right_arm:="$chirality" _add_blinders:=1 $chirality 


rosrun node ein _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="" _left_or_right_arm:="$chirality" _gray_box_top:=30 _gray_box_bot:=30 _gray_box_right:=60 _gray_box_left:=60 _mask_gripper:=1 _add_blinders:=1 $chirality 



