# Ein

Ein is pick-and-place software for Baxter.  See the installation
instructions on our web site: http://h2r.github.io/ein/.

Ein can be used to control Baxter, make models of objects, detect,
localize, and pick objects using these models.  We can localize
objects with Baxter to within 2mm, and pick objects hundreds of times
in a row without a failure.  The robot can also be used to create a
"for" loop over objects, picking from an input pile, playing with an
object (doing an arbitrary learning behavior that you choose), and
then moving to the output pile.  We have automatically processed
hundreds of objects in this way as part of the Million Object
Challenge.


We follow this branching model for Ein releases:
http://nvie.com/posts/a-successful-git-branching-model/ so the
development branch is develop, and master always contains the latest
release.