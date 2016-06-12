---
layout: page
title: FAQ
permalink: /faq/
order: 7
---

### What if my console stops updating?

Sometimes the console stops updating.  This problem can occur if you
reboot Baxter without restarting the console as well.  It can also
happen if you restart Ein (for unknown reasons).  To fix it, try
restarting the console.  Type ``` ` ` quit ``` (note the double screen
escape character, due to the subscreen inside the main Ein screen).
Then go up one command in the command history to restart the screen
session.


### What if the robot won't move?

Verify that you are not in zero gravity mode by checking the status
window; it should be set to 0. Type `zeroGOff` to turn off zero
gravity.
