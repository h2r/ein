---
layout: page
title: Back Language
permalink: /language/
order: 5
---

This page describes the Back programming language and the Ein stack machine.
Back is a stack-based postfix programming language inspired by FORTH, RPL,
Scheme, and Bash.  It runs on the Ein stack machine and can be stepped line by
line for debugging, executed alternately with callbacks during normal operation
(20 Hz or 20 instructions per second), or collapsed to execute segments of
instructions in an accelerated mode during which callbacks are not answered
(kHz range).

Central to Ein and Back are the **call stack** and the **data stack**. Both
stacks are collections of **words**, and each stack has at any given time a **top
element**.  One step in the Ein machine main loop consists of **popping** the top
word from the call stack and **executing** it. Words behave like functions when
executed, affecting some change on to the Ein state, the call stack, or the
data stack. Some words are more like data than functions in that they only push
individual words onto the data stack (`1` and `"something"`) or groups of words
onto the call stack (`( "hello" print )`), while others (like `+`) are more
obviously functions in that they consume words from the data stack, perform a
computation, and leav a result on the data stack.

### Argument Order

Many popular programming languages invoke functions by first naming the function and then listing its
arguments from left to right:

```
/* C code */
printf("Hello world.\n");
```

This is called **prefix order**.

In the same languages one can find functions which take their arguments on either side:

```
/* C code */
int a = b + c;
```

Here both `=` and `+` are written in **infix order**.

Ein uses **postfix order**, which means
that argments are listed first, or to the left, of the functions to which they belong: 

```
/* back code */
"Hello world." print
b c + "a" store
```




### Data and Variables

`store`

`define`

`1 1 +`

### Elementary Operations

### Control Words 

### Domain Words

Call stack and data stack. 

Reactive variables. 






