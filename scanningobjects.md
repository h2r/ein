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

Once everthing is clear, issue
```
catScan5UpdateInputBg catScan5UpdatePlayBg 
```

To update the background maps for the two workspaces. These words invoke scan patterns which cover the default workspaces
slightly better than a square scan. If you change the workspaces you might need to change these words.

Check the background maps by calling

```
catScan5LoadInputBg 
catScan5LoadPlayBg
```
and checking the Background View window after each command.
