#!/bin/bash

rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/data4" _class_name:="" _run_prefix:="R4"

rosrun oberlin_detection train_classifier _data_directory:="$(rospack find oberlin_detection)/data4" _vocab_file:="vocabT.yml" _knn_file:="knnT.yml" _label_file:="labelsT.yml" _class_labels:="background mrT greenPepper" _class_pose_models:="B S B"

rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/data4" _vocab_file:="vocabT.yml" _knn_file:="knnT.yml" _label_file:="labelsT.yml" _red_box_list:="" _run_prefix:="R4"

