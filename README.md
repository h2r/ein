# NODE 
NODE: Native Object Detection / Estimation

Please read this entire document before attempting to use the software. We start
by describing the underlying functionality and ROS nodes and proceed to describe
a convenient operating strategy and some utility scripts which you may find helpful.

If you are on Trusty 14.04 LTS with Indigo, you need the non-free functionality of opencv.

Install the ppa at https://launchpad.net/~xqms/+archive/ubuntu/opencv-nonfree:
```
sudo add-apt-repository ppa:xqms/opencv-nonfree
```
Then update apt-get 
```
sudo apt-get update
```
Then run
```
sudo apt-get install libopencv-nonfree-\*
```

Make sure you clean your build and devel folders before running catkin_make.

Openni may not work for you, in which case you can try freenect.

```
sudo apt-get install libfreenect-bin
freenect-glview
```

then 

```
sudo apt-get install ros-indigo-freenect-stack
roslaunch freenect_launch freenect.launch depth_registration:=true
```

**Install** the non-standard package h2r/bing by going to catkin_ws/src and then cloning.

**Install** the package h2r/node by going to catkin_ws/src and then cloning.

**NOTE** that BING broke upon 14.04 migration so you might need to update your BING code if you
are using an old install.

The basic workflow consists of three steps:

1. Gather data for your objects.
2. Train your classifiers.
3. Publish detections.

This document will walk you through each of these steps.

## Gather Data
First **make a data directory**, which will contain all of the data you collect and models you train.
Consider backing this up. If you change the source and want to check something in, please
tell git to ignore your data directories.
```
roscd node 
mkdir data
```
Next, make a subdirectory which will contain example images for an object you wish to detect.
You will need to create a new subdirectory for each new object.
```
mkdir -p data/object_class_1
```
To gather data for your objects, point your Kinect at your tabletop and run a command similar to:
(NOTE: you may need to run freenect instead of openni)
```
roslaunch openni_launch openni.launch depth_registration:=true
rosrun node capture_object _data_directory:="$(rospack find node)/data" _class_name:="object_class_1" _run_prefix:="june2round1" _left_or_right_arm:="center" center
```

That last "center" **MUST** come last but the rest are ROS params and can be permuted.

Make sure that **data_directory/class_name** is the FULL PATH to an existing directory.

This should open two windows. The first is the view through your Kinect, augmented with information, called "Object Viewer". 
The second is an eerie green interpretation of the view called "Density Viewer". 

Look at the Object Viewer.
You should see one or more blue boxes. Mess with the scene until the only blue box is around your object. The Density
Viewer has two sliders which you can adjust to change the number and size of blue boxes. Notice that blue boxes are drawn
around clusters of bright and dark green boxes. The canny_hi slider changes the number of bright green boxes, and the 
canny_lo slider changes the number of dark green boxes. A blue box will only be drawn around clusters containing at least
one bright green box, and blue boxes will contain entire connected components of dark and bright green boxes. So one way to 
think about this is that canny_hi controls the number of blue boxes you will get and canny_lo controls their size.

Click the Object Viewer
window somewhere in the main part. Check the contents of data_directory/class_name. You should see a saved crop for every
blue box that was on the screen during the 5 consecutive frames after you clicked. Notice that run_prefix is included in the file names.

Move the object around the table and capture crops of many different views. When you have enough crops, stop. If you want to
gather more data for the same object, run the command again but
MAKE SURE to **change the run_prefix** so you don't overwrite your data.

Run capture_object for every object you want to train, using a different subdirectory of data_directory for each class
you want to train.

## Train Classifiers
Next, train the classifier. Run a command similar to:
```
rosrun node train_classifier _data_directory:="$(rospack find node)/data" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _class_labels:="object_class_1 object_class_2" _class_pose_models:="S B" _left_or_right_arm:="center" center
```
This will create three files (for vocab, kNN, and labels) in the locations specified, paths relative to data_directory. The string
class_labels should be a " "-delimited list of the subdirectories you gathered data in, relative to data_directory. The string
class_pose_models is a " "-delimited list of the pose models for your classes. If your object is most like a bowl (i.e. you don't
care about its rotation in the plane of the table) its entry should be a "B", but if it is most like a spoon and you want
rotational estimates, its entry should be an "S".

Note that if you only train one object class, all blue boxes will be reported as that class. So it really doesn't make sense to
train a model for just one class. If there is only one object class you care about in general, train a second class called
"background" which contains plenty of examples of blue boxes that occur in your setting which you do not want to identify as
your object. It is usually a good idea to train a background class even if you have many classes, unless the background
class starts interfering with your real classes.

**Make sure your files were written** and they look sensible.

## Publish Detections
Finally, run something like:
```
rosrun node publish_detections _data_directory:="$(rospack find node)/data" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _left_or_right_arm:="center" center
```
This should bring up two familiar windows. This time, blue boxes should be labeled and table detections should be shown as brown
boxes.

You can subscribe to /publish_detections_center/blue_labeled_objects and /publish_detections_center/blue_object_markers. 
Open RViz and try viewing the markers. The table estimate should appear and should align well with 
the point cloud. Markers should appear for the blue boxes.

## Generic Pose Estimation
Now that you are familiar with the basic interface, you can try training a generic pose estimator. This means memorizing what
an object looks like from various known viewpoints and inferring orientation based on those views.

Suppose you have trained a model for a "book", but now you want to know which way the book is facing on the table so that you
can open and read it. Create a folder "bookPoses" and run something along the lines of
```
rosrun oberlin_detection capture_annotated_objects _data_directory:="$(rospack find oberlin_detection)/$givenDataSubdir" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _run_prefix:="$givenRunPrefix" _class_name:="book" _table_label_class_name:="mrT" _background_class_name:="background" _left_or_right_arm:="center" center
```
This training process is slightly different. Notice that we denote a table label class and a background class. You will need to have trained models for "book", "mrT", and
"background" and they should be included in the specified vocab, label, and kNN files. "mrT" is a marker whose orientation can be determined efficiently. 
Clicking the Object Viewer window will collect examples only for blue boxes labeled as "book", and this time the orientation calculated for "mrT" will be saved and associated with the crop for "book" 
in the same frame. This time examples and associated poses will be saved to the folder "bookPoses"
instead of "book". Take images of the book from all points of view which you are interested in identifying in the future. 
To facilitate this process, put the book and the mrT marker on a the drawing board, and put the drawing board on the lazy susan. You can then rotate the drawing board which rotates
mrT and the book together. Make sure that mrT and the book always maintain the same relative orientations, otherwise your data will be corrupted. As long as they do not move while you
rotate the susan, this property will be maintained.
Note that the table orientation estimate combined
with the "mrT" orientation estimate together form a complete 3D description of the orientation of the object. There is currently no way therefore to collect pose information
from views "underneath" the object, but some intrepid young susanaut might write an extension whereby the presence of a "flipped" marker means that the object has been rotated by some known
amount such as "pi radians about the x axis when mrT says the orientation is the identity transformation", which would mean that the object was upside down in a consistent
way before additional rotations were applied. Please be careful if you decide to do this.

Now, when you go to run train_classifier, specify "G" as the pose model for the class "book". In addition to normal training, the
program will now include the images in the "bookPoses" folder during vocabulary clustering. It will also train a separate kNN
classifier on the "bookPoses" images and save that classifier in the same file as the other classifier.

When you run publish_detections, you will notice that the blue boxes are now annotated with winning pose numbers. The published
detections incorporate the estimated orientation in the reported pose. You can check whether things are working correctly by looking
at the markers in rviz. If the process succeeded, rotating the object slowly will cause the marker in rviz to rotate smoothly.

Take extreme care if you choose to train generic pose models.

## Red Boxes
Pass 
```
_red_box_list:="class1 class2" 
```
when running publish_detections to generate red box pairs for class1 and class2.
Each red box pair consists of a bright red box and a dark red box assigned to a single class. The bright red box is the
current frame's red box detection. The dark red box is the average red box detection.

You can also subscribe to /publish_detections_center/red_labeled_objects and /publish_detections_center/red_object_markers. The
table marker is not included in red_object_markers.

Note that you may only see a bright red box, depending on the state of the code.

Only use red boxes in specific circumstances. They are slow at the moment and require some understanding
and tweaking to use properly.

## Brown Boxes

Brown boxes represent a detected table or flat surface. There are several shades, depending on
whether the point cloud is valid in checked locations. The table estimates at all valid brown
boxes are averaged and used to infer pose for objects using certain pose models.


## All Range Mode
To activate All Range Mode :feelsgood:, pass 
```
_all_range_mode:="0" or "1" 
```
to turn All Range Mode off (Rail Mode) or on. Off is default. Passing
```
_arm_box_right:="10" _arm_box_right:="20" _arm_box_right:="30" _arm_box_right:="40" 
```
sets the top left corner of the ARM box to (10,20) and the bottom right to (640-30,480-40).
When ARM is active, the ARM box is the only blue box.

There are two main situations where this is desirable. First, the single blue box can act
as a **trap**.  Train a class for the background images you are likely to encounter, and
train a class for a target object.  The blue box classification reported for the ARM box
will tell you whether the target entered the area of interest.

Second, red box detections become meaningful in that the system behaves like a **sliding 
window detector** with an adaptive window size and aspect ratio. Thus you should activate 
ARM when you expect heavy clutter and know exactly what you want to track with red boxes.

You may ask yourself "If ARM mode works, why do I ever use Rail Mode?" You might also ask
yourself "Where did my training crops come from?" and "What happens when I have 100 object
classes?":suspect:

## Gray Box
The gray box is a boundary outside of which green boxes are suppressed. That is,
in rail mode the area outside of the grey box is ignored.

Similarly to the ARM box, one can adjust the gray box by passing
```
_gray_box_right:="10" _gray_box_right:="20" _gray_box_right:="30" _gray_box_right:="40" 
```
which sets the top left corner of the gray box to (10,20) and the bottom right to (640-30,480-40).

## Wrist Cameras
Suppose you want to use the wrist cameras on baxter and you have set chirality="left" or chirality="right".
When running a node, pass the following
``` 
_image_topic:="/cameras/"$chirality"_hand_camera/image" _left_or_right_arm:="$chirality" $chirality
```
Note that you can think of all previous examples as using chirality="center" without remapping image_topic,
and that the detection topics will be /publish_detections_$chirality/*.
Since you will not get a point cloud, you cannot train generic pose models and spoon pose models will assume
the table is flat relative to the camera (check this, you may need to change a default value and recompile).
You can, interestingly enough, still get detections for generic pose models.

## Gripper Masking
When using wrist cameras, those pesky grippers can get in the way. To remove them from the green density map
(and thus suppress detections on and behind them), pass
```
_mask_gripper:=1
```

## Add Blinders
Objectness always tries to find objects. So if there aren't many objects or the lighting is bad, it will
begin to hallucinate blue boxes in relatively blank spaces. If you are getting false positives or your
object is poorly defined, you can increase objectness resolution by adding artificial objects on the
boundary of the image. Do this by exclaiming "Whoa there, kemosabe!" and passing
```
 _add_blinders:=1 
```

## ROS Params Can Be Dangerous
Note that whenever you pass an option such as _red_box_list="class1", that value will persist in the 
ROS params until roscore resets. So the red_box_list will remain populated unless you pass _red_box_list="".
The same goes for blinders, etc.

## Filter Time
There is another node provided called filter_time. It publishes to
```
/filter_time/filtered_image
/filter_time/filtered_pointcloud
```
and now is a good time to mention that you can redirect input to node with
```
_image_topic:="/filter_time/filtered_image"
_pc_topic:="/filter_time/filtered_pointcloud"
```

## Cached Training
Warning: at the moment you cannot cache G pose models. So make sure that if you are using G pose models
you are always adding them to cache rather than including them in the cache.

If you have a reasonable number of training examples, it can take a long time (>30 minutes) to cluster
the BoW vocabulary. If you are only adding more examples to existing categories (or if you are adding a
simple object to an already diverse set of objects), you may want to use an a previously trained vocabulary
during rapid model prototyping. Setting 
```
_retrain_vocab:="0"
```
during training will cause the passed vocab file to be loaded rather than overwritten. If later you want to 
retrain the vocab, simply omitting this term will not cause the vocab to retrain since ROS params persist.
You need to set
```
_retrain_vocab:="1"
```
to make sure it retrains.

You should always go back and train using all of your classes (possibly changing the subsampling factor
to make it finish in a reasonable amount of time, possibly training overnight) because you will almost 
certainly get a performance gain.

A good strategy to adjusting the subsampling factor is to make it so small that training finishes instantly.
Then increase it by a constant factor (pick your favorite number between 2 and 10) until it takes 5 minutes
to finish, then increase it once or twice more. To change the sampling factor, go to the main .cpp file,
set bowSubSampleFactor to a number in the interval (0,1], and recompile. Similary, if you have more than 10
classes, you may want to adjust the number of visual words. Something between 1000 and 10000 is good, but
increase it slowly (500 or 1000 at a time). More is not necessarily better. The parameter to change is
vocabNumWords.

Additionally, it is possible to cache the extracted kNN feature vectors by specifying a cache_prefix during
training, such as:
```
_cache_prefix:="cache27"
```
If you do
this you will need to make copies of your kNN and label files and prepend the cache_prefix to
the root file names "knn.yml" and "labels.yml". Having done so, during training you need only specify classes which you wish to 
add to the kNN and label files. If this is confusing, please refer to the main .cpp file. For an
example usage, please see 
```
$(rospack find node)/util/trainWithCache.sh
```

## Utility Scripts
It can feel clunky passing so many parameters when running nodes.  You can find some handy utility scripts
in 
```
$(rospack find node)/util
```
for collecting data, training classifiers, and publishing detections.  Instead of setting ROS params, you
can pass arguments to the scripts and they will invoke the ROS nodes with the appropriate parameters.
Since most people will have separate lists of objects, it is recommended that you create your own utility 
scripts using these as templates. Again, if you plan on checking in code changes, please tell git to
ignore your personal utility scripts.

For example, I have many root data directories. One of them is called "data4". If I want to train a new
object called "gyroBowl" (or add examples if it already exists) with run_prefix "run1" using the Kinect
(as opposed to a wrist camera), I can call
```
$(rospack find node)/util/newObject.sh data4 gyroBowl run1 center
```
which will create the necessary folders if they do not exist and launch capture_object with the appropriate
parameters.

Perhaps the grey box is too big. So I can run
```
$(rospack find node)/util/dave_capture_small.sh publish_detections_center
```
and the grey box will be adjusted. Check out the dave scripts as they do different things.

After inspecting how newObject.sh works, you can take a look at some of the other scripts, it should
be clear how they work.  A complete workflow can be had with
```
$(rospack find node)/util/newObject.sh
$(rospack find node)/util/refreshCache.sh
$(rospack find node)/util/collectPoses.sh
$(rospack find node)/util/trainWithCache.sh
$(rospack find node)/util/publishDetections.sh
```
You should think of refreshCache.sh as a real world example of invoking train_classifier. The script
trainWithCache.sh shows an advanced technique and you can certainly get away with never using it.

## Thanks :metal:
Thanks for reading. :japanese_goblin:

## TODO XXX 

This is a list of stuff that needs to be done. Items in bold are either in the process
of being done or have been done recently SINCE a stability check was issued. That is to
say, if there is a problem in the code, bold items are a good place to start looking
since there were no known problems before items were emboldened.

### General
**Reconsider window layouts and how to manage controls**

Factor bounding box drawing**

### Object Detection Suite
**Caching doesn't handle G pose models**

**update oriented filter pose models**

  **add T pose model for mrT pose marker**

  **add l pose model for mirror symmetric line utensils**

  **change S pose model back to spoon model**

  **add "is oriented filter" check function**
  
  **add "set orientedfilter" function after that**
  

Use blue screen during training

Augment image with blinders so you don't lose real-estate

Blinders on all four sides instead of just two

Try including color information in the BoW model 

Try SVM instead of kNN

Add a sign during training that says "this object is on its side or upside down"
so that generic pose models are more complete

Scripts or code should switch video channels automagically based on chirality

Consider adding McAllester's HOG feature

Investigate offloading feature computation to the GPU or multithread on the CPU

Add a global feature map that refreshes each keypoint only when it is needed and with
a certain probability so that frames are faster but new information trickles in at 
about the same rate

Consider run-time background shunting



### Pilot Baxter Suite
Listen to recognized object arrays instead of dedicated pilot target channel

PID controls should be from within the end effector reference frame rather than the global frame

Add object class awareness and per-object grasp offset

Add orientation awareness and servoing

Add scale servoing and grasp depth inference instead of relying on the range sensor

Add intelligent depth-to-table scan

Add wait-until-destination-reached no-op instruction

Add a facility for extending PDA instruction set


















