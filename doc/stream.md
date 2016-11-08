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


### Collecting Data in the Stream Buffer

`activateSensorStreaming` activates sensor streaming, causing the
in-memory buffers to begin filling up with data.
`deactivateSensorStreaming` turns off streaming.

There are six types of streams: images, poses, labels (marked labels
in the stream), words (words executed by Ein during the event), joint
angles, and range sensor readings.  By default, none of these streams
are turned on, so `activateSensorStreaming` will not do anything.  You
can toggle them with `setSisFlags` or individual getters
(`streamSis*`) and setters (`streamSetSis*`).  A useful shortcut is
`streamEnableSisImageAndPoses` to stream images and poses.

To stream data, run:

`streamEnableSisImageAndPoses activateSensorStreaming`

It may also be useful to run `shutdownAllNonessentialSystems` which
will turn off rendering and other features, so that Ein can stream
data as quickly as possible, minimizing the number of frames that are
lost.

To turn off streaming, run `deactivateSensorStreaming`.  At this
point, you should probably also run `bringUpAllNonessentialSystems` if
you have shut things down.    

After deactivating streaming, `streamImageBufferSize` will contain the
number of images saved, while `streamPoseBufferSize` will contain the
number of poses.  `clearStreamBuffers` empties the stream buffer.
Streaming is cumulative, so activiting and deactivating streaming will
append to the stream buffer.

#### Exercise:  Stream some images and poses.

Activate sensor streaming to populate the image and pose stream
buffers.  Inspect the size of the two buffers.  Which one is larger?
Why might that be?  Now run `clearStreamBuffers` and observe that the
sizes are back to zero.

Answer (select to see):

Images come in at the camera framerate, approximately 30 Hz from
Baxter's wrist camera.  (Your milage may vary when using a Kinect 2,
and depending on how many cameras you are using.)  Poses come in at
approximately 100Hz.  So you will receive many more images in a given
time period than poses.  This is useful because you can find an
accurate pose for each image.
{: style="color:white;" }


### Saving and Loading Data

Stream data is saved in the focused class.  Stream data itself
consists of the raw data that is saved about an object, most often
pose-annotated images.  To save the stream buffers to disk, use
`streamWriteBuffersToDisk`.  Alternatively, you can
`enableDiskStreaming` to stream as data is loaded into the buffer.


`populateStreamBuffer` populates the stream buffer with data saved
from the focused class.  `initializeAndFocusOnANewClass` creates a new
class for saving stream buffer data.  


### Incrementing and Viewing Data

You can access the current stream buffer index with
`streamImageBufferCurrentIdx` and set it with
`streamSetImageBufferCurrentIdx`.  The "Stream Viewer" window displays
the contents of the stream buffer.  You can run
`streamRenderStreamWindow` to render the current contents of the
buffer.

#### Exercise:  Write a word to play back the stream buffer.

Stream some images and poses.  You will want to use `replicateWord`
and `streamImageBufferSize` to reproduce the rendering command.
Select to see our solution, or also see the word
`streamPlayStreamBuffer`.

( streamIncrementImageStreamBuffer streamRenderStreamWindow 0.01 waitForSeconds ) streamImageBufferSize replicateWord
{: style="color:white;" }


### Rendering Synthetic Photographs

To render synthetic photographs from th
