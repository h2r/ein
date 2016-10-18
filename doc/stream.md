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

`populateStreamBuffer` populates them from focused class.
