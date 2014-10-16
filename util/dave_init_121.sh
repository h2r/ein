#!/bin/bash


# XXX loop over application names


rosparam set /$1/reject_area_scale 16
rosparam set /$1/green_box_threshold 5.0
rosparam set /$1/density_decay .3

