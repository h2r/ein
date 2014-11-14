#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix

#rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix" _image_topic:="/cameras/right_hand_camera/image"

#rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix" _image_topic:="/camera/rgb/image_raw"

#rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="signStar" _image_topic:="/cameras/right_hand_camera/image"

#rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="lime" _image_topic:="/camera/rgb/image_raw"


rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="" _image_topic:="/cameras/"$chirality"_hand_camera/image" _mask_gripper:=1 _add_blinders:=1 _left_or_right_arm:="$chirality" $chirality
