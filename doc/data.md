---
layout: page
title: Data
permalink: /data/
order: 7
---

This page describes how to save and load data with Ein, especially
focusing on calibrated light rays.  One understandig of Ein is as a
tool for collecting calibrated light rays.


* Gripper mask
* Camera calibration parameters
* Camera color information
* table height
* hand camera offset?


### Saving and Loading Scenes



### Data Formats

Ein stores its data in the `default` directory in the ein checkout.
Configuration is stored in a directory named for the serial number.
Objects are stored in the `objects` directory and gaussian maps are
stored in the `maps` directory.  Streams are stored in the `streams`
directory.  We use a combination of custom YAML formats as well as
standard images to store data.  Scenes consist of uuencoded compressed
YAML, and we provide C++ and python code to unpack it into opencv
images. 





