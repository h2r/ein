#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix $chirality

#rosrun node publish_detections _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocab27.yml" _knn_file:="knn27.yml" _label_file:="labels27.yml" _red_box_list:="" 
#rosrun node publish_detections _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="lime" _left_or_right_arm:="$chirality" $chirality 

#rosrun node publish_detections _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="" _left_or_right_arm:="$chirality" _add_blinders:=1 $chirality 

rosrun node publish_detections _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocab27NoOpponent.yml" _knn_file:="knn27NoOpponent.yml" _label_file:="labels27NoOpponent.yml" _red_box_list:="" _left_or_right_arm:="$chirality" _add_blinders:=1 _image_topic:="/cameras/"$chirality"_hand_camera/image" _chosen_feature:="1" $chirality 

rosrun node publish_detections _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocab27YesOpponent.yml" _knn_file:="knn27YesOpponent.yml" _label_file:="labels27YesOpponent.yml" _red_box_list:="" _left_or_right_arm:="$chirality" _add_blinders:=1 _image_topic:="/cameras/"$chirality"_hand_camera/image" _chosen_feature:="2" $chirality 

