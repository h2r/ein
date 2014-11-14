#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix $chirality

mkdir $(rospack find oberlin_detection)/$givenDataSubdir 
mkdir $(rospack find oberlin_detection)/$givenDataSubdir/$givenClassName 

rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix" _add_blinders:=1 _left_or_right_arm:="$chirality" $chirality 


#rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix" _image_topic:="/cameras/right_hand_camera/image" _left_or_right:="right" right
