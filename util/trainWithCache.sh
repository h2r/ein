#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
givenPoseModel=$4
chirality=$5

echo $givenDataSubdir $givenClassName $givenRunPrefix $chirality

rosrun oberlin_detection train_classifier _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocabPlusCache.yml" _knn_file:="knnPlusCache.yml" _label_file:="labelsPlusCache.yml" _class_labels:="$givenClassName" _class_pose_models:="$givenPoseModel" _retrain_vocab:="0" _cache_prefix:="cache27" _left_or_right_arm:="$chirality" $chirality

