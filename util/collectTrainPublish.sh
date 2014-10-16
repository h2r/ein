#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3

#givenFileSuffix=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix

#rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix"

#rosrun oberlin_detection train_classifier _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _class_labels:="background mrT greenPepper" _class_pose_models:="B S B"

#rosrun oberlin_detection train_classifier _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _class_labels:="background mrT greenPepper apple artichoke banana blackTape blueBowl brownEgg chile cucumber diaper greenBlock gyroBowl hybridBranch lemon lime metalBowl mug orange petroleum plasticKnife redBlock tanBowl whiteEgg signStar robot" _class_pose_models:="B S B B B S B B B S S B B B B B B B B B B S B B B B B" _retrain_vocab:="1"

rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _red_box_list:="lime" 

#rosrun oberlin_detection capture_annotated_objects _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _red_box_list:="" _run_prefix:="R7"

#rosrun oberlin_detection capture_hard_class _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _red_box_list:="" _run_prefix:="$givenRunPrefix" _class_name:="$givenClassName" 
