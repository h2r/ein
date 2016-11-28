---
layout: page
title: Back Language
permalink: /language/
order: 5
---

This page describes the Back programming language and the Ein stack
machine.  Back is a stack-based postfix programming language inspired
by
[FORTH](https://en.wikipedia.org/wiki/Forth_(programming_language)),
[RPL](https://en.wikipedia.org/wiki/RPL_(programming_language)),
[Scheme](https://en.wikipedia.org/wiki/Scheme_(programming_language)),
and [Bash](https://en.wikipedia.org/wiki/Bash_(Unix_shell)).  It runs
on the Ein stack machine and can be stepped line by line for
debugging, executed alternately with callbacks during normal operation
(20 Hz or 20 instructions per second), or collapsed to execute
segments of instructions in an accelerated mode during which callbacks
are not answered (kHz range).  

Central to Ein and Back are the **call stack** and the **data stack**. Both
stacks are collections of **words**, and each stack has at any given time a **top
element**.  One step in the Ein machine main loop consists of **popping** the top
word from the call stack and **executing** it. Words behave like functions when
executed, affecting some change on to the Ein state, the call stack, or the
data stack. Some words are more like data than functions in that they only push
individual words onto the data stack (`1` and `"something"`) or groups of words
onto the call stack (`( "hello" print )`), while others (like `+`) are more
obviously functions in that they consume words from the data stack, perform a
computation, and leave a result on the data stack.

### Argument Order

Many programming languages invoke functions by first naming the function and then listing its
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

Ein, like
[Forth](https://en.wikipedia.org/wiki/Forth_(programming_language))
and [RPL](https://en.wikipedia.org/wiki/RPL_(programming_language)),
uses **postfix order**, which means that argments are listed first, or
to the left, of the functions to which they belong:

```
/* back code */
"Hello world." print
b c + "a" store
```

The reason for this is that words take their arguments from the data stack. In order for 
something to end up on the data stack, something must put it there, and it needs to be put
there before it is used.

#### Exercise:  Calculate with Ein!

Do some arithmetic with Ein!  Push some numbers on the data stack, and
observe them appearing at the console.  Then add them together with
`+`, and observe the result on the data stack.  Press enter after each
word to observe the intermediate states, e.g., `1`, `1`, `+` should
leave `2` on the data stack.  Then try running them all at once:  `1 1 +`. q


#### Exercise:  Hello World!

Write a program to print hello world.  First you push a string on the
data stack.  Then use the `print` word to print it to the console.


### Data and Variables


Ein words can be multiple types.  Primitive types include string,
integer, double, EePose (the pose of the robot's end effector as (x,
y, z, quaterion)).  Additionally, compound words can be defined using
parenthesis as list of other words.

To define a string literal, use quotes: 
```"Hello world"```

This command will push the string "Hello world" on the data stack. 


Variables can be defined using the `store` word, which takes two
arguments: the value (any word), and a string for the variable name.
For example, `0 "x" store` will store the integer word zero in the
variable x.  Aftewards, typing `x` will push a `0` on the data stack,
just as if you had typed `0`.

You can also use store to define new compound words: 

`( 1 + ) "inc" store` definse a new compound word, `inc` which adds
`1` to the argument on the data stack and pushes the result.


Finally, the `define` word takes both a word and help text: 

`( 1 + ) "Increment the value on the data stack" "inc" define`



Ein gets its power from reactive variables, defined in C, such as
truePose, which is always set to the value of the current pose of the end effector.   This reactivity 



### Control Words 

Back includes looping and condition constructs.

`ift` takes two arguments and prints if its argument is true.  Ein
casts words to booleans followin C's semantics, so zero is false, and
all other values are true (including the empty string).

This version prints "hello":
`( "hello" print ) 1 ift`

This version does not:
`( "hello" print ) 0 ift`


`ifte` is "if, then, else". Here is an example:

 0 ( "condition was true" print ) ( "condition was false" print )   ifte

`not` negates its condition.

`while` takes a condition and a compound word and executes until the
condition is true.  While collapses the stack and will freeze ein
unless the condition includes a wait word.

`( 1 ) ( torsoFanOn 1 waitForSeconds )  while`


### Back files


Back programs live in the `ein/back` directory, in <filename>.back.
You can load them by running `"<filename>" import`.  For example,
`"kids" import` will load the words in kids.back.  Text in a back file
is any back program that can be typed at the repl.  It is common for
back files to consist of `define` and `store` commands to create new
compound words.

#### Exercise:  Back files

Create a new back file called `hello.back`.  First put a "hello world"
program in it and run `"hello" import`.  Then write a new word that
prints "hello" and also causes the robot to wave its arm (use the
`truePose` variable to read the arm's position and `moveEeToPoseWord`
to move to a predefined pose) and nod its head (`nod`).


### File Input and Output

Files can be created with `fileOpenInput` and `fileOpenOutput`.
`fileReadLine`, `fileReadAll` take a file on the stack and leave a
string.  `fileWrite` and `fileWriteLine` take a file and another word
and write that word to the file; if the word is a string, the string
is written; otherwise `repr` is used to convert the word to a string.
If you do this, the result can be evaluated with `eval` to recreate
the contents of the file on the stack.  Examples are in fileio.back.
