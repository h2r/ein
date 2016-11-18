---
layout: page
title: Mapping Objects
permalink: /scanningobjects/
order: 4
---

We will set up infinite scan so the robot automatically learns about
objects.  This process is a giant for loop over objects; in the inner
loop you can do whatever experiments you like.  


There are three poses defined in scan.back designated with functionality for the scan:

```
inputPileWorkspace
playWorkspace
outputPileWorkspace
```
Move the arm to these locations by issuing, for instance:

```
playWorkspace moveEeToPoseWord
```
You can change the values of these variables in the file, save, and reload with

```
"scan" import
```

Check the `inputPile` and `playWorkspace` and make sure there is about 20cm by 20cm of free real estate. The output
pile doesn't really need space as long as objects deposited there can clear the other workspaces.

Once everything is clear, issue

```
catScan5CacheBackgrounds
```

This word invokes scan patterns which cover the default workspaces
slightly better than a square scan. If you change the workspaces you
might need to change the scan patterns.

Check the background maps by calling

```
catScan5LoadInputBg 
catScan5LoadPlayBg
```
and checking the Background View window after each command.


Find three objects that fit in the 4cm gripper and contrast with the background. Place them in the `inputPile` workspace and issue

```
tableInfiniteScan
```

The arm should scan the play workspace to make a new map, scan the input workspace to find an object, move the object to the the play workspace,
play with it, move it to the output pile, and begin again by scanning the play workspace.










