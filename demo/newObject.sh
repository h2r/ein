#!/bin/bash

givenDataSubdir=$1
givenClassName=$2
givenRunPrefix=$3

echo $givenDataSubdir $givenClassName $givenRunPrefix

mkdir $(rospack find oberlin_detection)/$givenDataSubdir 
mkdir $(rospack find oberlin_detection)/$givenDataSubdir/$givenClassName 

rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _class_name:="$givenClassName" _run_prefix:="$givenRunPrefix"

