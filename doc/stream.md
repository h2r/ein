---
layout: page
title: Stream Buffers
permalink: /stream/
order: 7
---

This page describes how to use Ein's stream buffer facility.  When
saving data, it is useful to be able to stream sensor information from
the robot directly to disk, as well as to load sensor streams from
disk into the buffers.  For example, after taking a spiral scan, the
robot can merge the images into a synthetic photograph, but also saves
a buffer of the original calibrated images.  This page describes how
to interact and use the lower-level stream buffer.



### Stream Buffer

`populateStreamBuffer` populates them from focused class.  `clearStreamBuffers` empties the stream buffer.  




`activateSensorStreaming` activates sensor streaming, causing the
in-memory buffers to begin filling up with data.
`deactivateSensorStreaming` turns off streaming.

There are six types of streams: images, poses, labels (marked labels
in the stream), words (words executed by Ein during the event), joint
angles, and range sensor readings.  By default, none of these streams
are turned on, so `activateSensorStreaming` will not do anything.  You
can toggle them with `setSisFlags` or individual getters
(`streamSis*`) and setters (`streamSetSis*`).  