#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3

echo $givenDataSubdir $givenClassName $givenRunPrefix

rosrun oberlin_detection train_classifier _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab25.yml" _knn_file:="knn25.yml" _label_file:="labels25.yml" _class_labels:="background mrT greenPepper apple artichoke banana blackTape blueBowl brownEgg chile cucumber diaper greenBlock gyroBowl hybridBranch lemon lime metalBowl mug orange petroleum plasticKnife redBlock tanBowl whiteEgg signStar robot" _class_pose_models:="B S B B B S B B B S S B B B B B B B B B B S B B B B B" _retrain_vocab:="1"
