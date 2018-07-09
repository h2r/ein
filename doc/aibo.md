---
layout: page
title: Aibos
permalink: /aibo/
order: 98
---

Ein contains an interface to connect to and program Sony [AIBO](https://en.wikipedia.org/wiki/AIBO) robots.


Both the AIBO and the computer running Ein must be on the same
wireless network.  This step requires configuring the compact flash
card on the AIBO to point to the correct network.  You should verify
that your base station can ping the AIBO.  The AIBO must be running
the URBI interface, which Ein uses to send it commands.  One way to
achieve this while on the go is to use a wireless network created by
your cell phone with internet connection sharing.

Start Ein in [simulation mode](../simulation).


Turn the AIBO on.  Look in "aibo.back" and find the word
`dogSummonElla`.  ("Ella" is the name of one of our particular AIBOs;
feel free to copy and make a new word for each AIBO you use.)  This
word connects to and attaches to an AIBO with a specified IP address.
Change the IP address to one pointing to your AIBO.  If you are using
a cell phone network, you can usually see connected devices in your
phone's config.

At the terminal, run `"aibo" import` to import the modified file.

Run `dogSummonElla` to connect to the Aibo.  The Aibo should twitch
and make a noise indicating that it has connected.

There exists a full-featured interface to send individual joint
commands to the AIBO as well as receive images from its camera and read joint states.

To receive images, run `dogStreamSnountImages`.  Images can be viewed
in the "Dog Snount View" window.




### Programming Activity for Young Children

The goal is to teach kids about abstraction (cards map to actions) and
sequencing (the action for each card gets executed one by one in
order), and that robots do exactly what they are programmed to do.

Make cards with individual actions on them, for example:

* Bark
* Sit
* Lie down
* Stand
* Wag your tail
* Walk Forward
* Walk Backward
* Turn Left
* Turn Right

To see a complete list of pre-programmed actions, type `demoDog` and
then `tab` at the repl.

The cards have the actions in words, but also a picture so that the
game is accessible to pre-reading children, and gives reading practice
to older children.  I hand-draw stick figures on the cards.  You deal
out some cards to make a program.  For example, the program might be
"Sit", "Bark", "Stand up," "Walk Forward.  The children get to pick
the what cards and what order.  Then quickly type the program into
Ein's REPL.  To make this easy, all the words begin with `demoDog` so
you can use tab completion.  I have the class read the program to me
as I type it to check it over.  Then have the class say "Ella execute"
(if the AIBO's name is Ella), and then hit "Enter."  Hold up each card
as the AIBO executes its.  I usually do "static" words with short
programs first, then advance to longer programs.  I go around the
circle and let each child pick the next work to get five or six or
seven-word programs.  

After the kids get wiggly, switch to "programming" the kids.  Use the
same cards to make programs for them to execute.  It is super fun
watching kids pretend to be puppies.  
