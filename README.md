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
2. Train a BoW-kNN classifier.
3. Publish detections.

First **make a data directory**
```
roscd node 
mkdir -p data/object_class_1
```

## Gather Data
To gather data for your objects, point your kinect at your tabletop and run a command similar to:
```
roslaunch openni_launch openni.launch depth_registration:=true
rosrun node capture_object _data_directory:="$(rospack find node)/data" _class_name:="object_class_1" _run_prefix:="june2round1" _left_or_right_arm:="center" center
```

That last "center" **MUST** come last but the rest are ROS params and can be permuted.

Make sure that **data_directory/class_name** is the FULL PATH to an existing directory.

This should open two windows. The first is the view through your kinect, augmented with information, called "Object Viewer". 
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

## Train
Next, train the classifier. Run a command similar to:
```
rosrun node train_classifier _data_directory:="$(rospack find node)/data" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _class_labels:="object_class_1 object_class_2" _class_pose_models:="S B" _left_or_right_arm:="center" center
```
This will create three files (for vocab, knn, and labels) in the locations specified, paths relative to data_directory. The string
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

## Detect
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
"background" and they should be included in the specified vocab, label, and knn files. "mrT" is a marker whose orientation can be determined efficiently. 
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
Note that you can think of all previous examples as using chirality="center" without remapping image_topic.
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

## Thanks :metal:
Thanks for reading. :japanese_goblin:



