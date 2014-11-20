#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3
chirality=$4

echo $givenDataSubdir $givenClassName $givenRunPrefix $chirality

#rosrun node train_classifier _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocab27.yml" _knn_file:="knn27.yml" _label_file:="labels27.yml" _class_labels:="background mrT greenPepper apple artichoke banana blackTape blueBowl brownEgg chile cucumber diaper greenBlock gyroBowl lemon lime metalBowl mug orange petroleum plasticKnife redBlock tanBowl whiteEgg signStar" _class_pose_models:="B S B B B S B B B S S B B B B B B B B B S B B B B" _retrain_vocab:="0" _left_or_right:="$chirality" $chirality

rosrun node train_classifier _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocab27NoOpponent.yml" _knn_file:="knn27NoOpponent.yml" _label_file:="labels27NoOpponent.yml" _class_labels:="background mrT greenPepper apple artichoke banana blackTape blueBowl brownEgg chile cucumber diaper greenBlock gyroBowl lemon lime metalBowl mug orange petroleum plasticKnife redBlock tanBowl whiteEgg signStar" _class_pose_models:="B S B B B S B B B S S B B B B B B B B B S B B B B" _retrain_vocab:="1" _chosen_feature:="1" _left_or_right:="$chirality" $chirality

rosrun node train_classifier _data_directory:="$(rospack find node)/$givenDataSubdir" _vocab_file:="vocab27YesOpponent.yml" _knn_file:="knn27YesOpponent.yml" _label_file:="labels27YesOpponent.yml" _class_labels:="background mrT greenPepper apple artichoke banana blackTape blueBowl brownEgg chile cucumber diaper greenBlock gyroBowl lemon lime metalBowl mug orange petroleum plasticKnife redBlock tanBowl whiteEgg signStar" _class_pose_models:="B S B B B S B B B S S B B B B B B B B B S B B B B" _retrain_vocab:="1" _chosen_feature:="2" _left_or_right:="$chirality" $chirality

