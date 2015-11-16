#!/bin/bash

rostopic pub -1 /ein/right/forth_commands std_msgs/String "$1"  & rostopic pub -1 /ein/left/forth_commands std_msgs/String "$1"
