---
layout: page
title: Picking Objects
permalink: /pickingobjects/
order: 3
---

In your third ten minutes with Ein, we will train a model for an
object, and pick it.



The basic workflow for object picking is:

( calibrate once )

Train workspace background  model
Train object model
  Automatic grasp annotation
    Manual grasp annotat

( change workspace or recompute background if table shifts or lighting changes )

Map workspace and compute discrepancy (background subtraction)
Detect object(s) cascaded on discrepancy

( perform other robotic actions )

Pick objects based on detections.




