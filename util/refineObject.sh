#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3

echo $givenDataSubdir $givenClassName $givenRunPrefix

mkdir "$(rospack find node)/$givenDataSubdir/$givenClassName"Poses 

rosrun node capture_hard_class _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _red_box_list:="" _run_prefix:="$givenRunPrefix" _class_name:="$givenClassName" _table_label_class_name:="mrT" _background_class_name:="background"
 


