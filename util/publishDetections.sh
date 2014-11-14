#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix $chirality

#rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab27.yml" _knn_file:="knn27.yml" _label_file:="labels27.yml" _red_box_list:="" 

rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="" _left_or_right_arm:="$chirality" _add_blinders:=1 $chirality 

#rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="lime" _left_or_right_arm:="$chirality" $chirality 


