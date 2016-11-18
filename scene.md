---
layout: page
title: Synthetic Photographs
permalink: /scene/
order: 7
---

This page describes how to use the synthetic photograph infrastructure
with Ein.

A Gaussian Map is a synthetic photograph of a scene.  It consists of
an MxN matrix of cells, where each cell contains an RGB mean and
variance estimate, as well as a height estimate (z).  It also has an
associated anchor pose representing the location of the map.

A Scene is an object consisting of three Gaussian Maps: the background
map, the observed map, and the predicted map, as well as a pose for
the scene.  `currentScene` pushes the current scene on the stack.
`scenePredictedMap`, `sceneObservedMap` and `sceneBackgroundMap`
access the various maps and push them on the stack.  By default, all
maps are pointers.  Memory is not reallocated when they are moved
around on the stack.  Similarly, if you save the current scene as a
variable, and then change the current scene, the variable will also
change as it is a pointer to the current scene.  To perform a deep
copy, use `sceneCopyScene` and `sceneCopyGaussianMap`.

To save and load gaussian maps from file, use `sceneSaveGaussianMap`
and `sceneLoadGaussianMap`.

#### Exercise:  Save multiple background maps.

Make two different maps for different parts of the workspace, and save
them both to disk as bg1 and bg2.  Load them both into variables and
alternate which one is the focused background map.  You can use
`tableShortTakeScene` to create an observed map with just a few frames
of video at the current position to speed up testing.  Note that if
you set the background map in the current scene, it will not be
visible until after calling `sceneRenderScene`.

Select to see our program!

tableShortTakeScene currentObservedMap "bg1"  sceneSaveGaussianMap<br/>
( xUp ) 10 replicateWord waitUntilAtCurrentPosition tableShortTakeScene currentObservedMap "bg2" sceneSaveGaussianMap <br/>
"bg1" sceneLoadGaussianMap "bg1" store <br/>
"bg2" sceneLoadGaussianMap "bg2" store <br/>
currentScene bg1 sceneSetBackgroundMap sceneRenderScene <br/>
currentScene bg2 sceneSetBackgroundMap sceneRenderScene <br/>
{: style="color:white;" }





