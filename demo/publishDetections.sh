#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3

echo $givenDataSubdir $givenClassName $givenRunPrefix

rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _red_box_list:="" 
