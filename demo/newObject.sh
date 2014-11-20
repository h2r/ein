#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix

mkdir $(rospack find node)/$givenDataSubdir 
mkdir $(rospack find node)/$givenDataSubdir/$givenClassName 

rosrun node capture_object _data_directory:="$(rospack find node)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix" _image_topic:="/cameras/"$chirality"_hand_camera/image" _mask_gripper:=1 _add_blinders:=1 _left_or_right_arm:="$chirality" $chirality

