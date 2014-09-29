# oberlin_detection


Make sure you have the following standard components installed:

opencv
cv_bridge
image_transport
openni

**Install** the non-standard package h2r/bing.

To get up and running you will need to perform three steps.

1. Gather data for your objects.
2. Train a BoW-kNN classifier.
3. Publish detections.

First **make a data directory**
```
roscd oberlin_detection
mkdir -p data/object_class_1
```

## Gather Data

To gather data for your objects, point your kinect at your tabletop and run a command similar to:
```
roslaunch openni_launch openni.launch depth_registration:=true
rosrun oberlin_detection capture_object _data_directory:="$(rospack find oberlin_detection)/data" _class_name:="object_class_1" _run_prefix:="june2round1"
```
Make sure that **data_directory/class_name** is the FULL PATH to an existing directory.

This should open two windows. The first is the view through your kinect, augmented with information, called "Object Viewer". 
The second is an eerie green interpretation of the view called "Density Viewer". Look at the Object Viewer.

You should see one or more blue boxes. Mess with the scene until the only blue box is around your object. Click the Object Viewer
window somewhere in the main part. Check the contents of data_directory/class_name. You should see a saved crop for every
blue box that was on the screen. Notice that run_prefix is included in the file names.

Move the object around the table and capture crops of many different views. When you have enough crops, stop. If you want to
gather more data for the same object, run the command again but
MAKE SURE to **change the run_prefix** so you don't overwrite your data.

Run capture_object for every object you want to train, using a different subdirectory of data_directory for each class
you want to train.

## Train

Next, train the classifier. Run a command similar to:
```
rosrun oberlin_detection train_classifier _data_directory:="$(rospack find oberlin_detection)/data" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _class_labels:="object_class_1" _class_pose_models:="B"
```
This will create three files (for vocab, knn, and labels) in the locations specified, paths relative to data_directory. The string
class_labels should be a " "-delimited list of the subdirectories you gathered data in, relative to data_directory. The string
class_pose_models is a " "-delimited list of the pose models for your classes. If your object is most like a bowl (i.e. you don't
care about its rotation in the plane of the table) its entry should be a "B", but if it is most like a spoon and you want
rotational estimates, its entry should be an "S".

**Make sure your files were written** and they look sensible.

## Detect

Finally, run something like:
```
rosrun oberlin_detection publish_detections _data_directory:="$(rospack find oberlin_detection)/data" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml"
```
This should bring up two familiar windows. This time, blue boxes should be labeled and table detections should be shown as brown
boxes.

You can subscribe to /oberlin_detection/blue_labeled_objects and /oberlin_detection/blue_object_markers. 
Open RViz and try viewing the markers. The table estimate should appear and should align well with 
the point cloud. Markers should appear for the blue boxes.

## Generic Pose Estimation

Now that you are familiar with the basic interface, you can try training a generic pose estimator. This means memorizing what
an object looks like from various known viewpoints and inferring orientation based on those views.

Suppose you have trained a model for a "book", but now you want to know which way the book is facing on the table so that you
can open and read it. Create a folder "bookPoses" and run capture_object as above, this time pointing to the folder "bookPoses"
instead of "book". Take images of the book from various points of view. It would be a good idea to structure the capturing process
so that you can recover the viewpoint from the shot index.

Now, when you go to run train_classifier, specify "G" as the pose model for the class "book". In addition to normal training, the
program will now include the images in the "bookPoses" folder during vocabulary clustering. It will also train a separate kNN
classifier on the "bookPoses" images and save that classifier in the same file as the other classifier.

When you run publish_detections, you will notice that the blue boxes are now annotated with winning pose numbers. The published
object_recognition_msgs::RecognizedObject.type.key fields for books now contain "book %d" where %d is the winning pose index. 

## Red Boxes

Pass 
```
_red_box_list:="class1 class2" 
```
when running publish_detections to generate red box pairs for class1 and class2.
Each red box pair consists of a bright red box and a dark red box assigned to a single class. The bright red box is the
current frame's red box detection. The dark red box is the average red box detection.

You can also subscribe to /oberlin_detection/red_labeled_objects and /oberlin_detection/red_object_markers. The
table marker is not included in red_object_markers.

## Brown Boxes

Brown boxes represent a detected table or flat surface. There are several shades, depending on
whether the point cloud is valid in checked locations. The table estimates at all valid brown
boxes are averaged and used to infer pose for objects using certain pose models.


## *New* All Range Mode

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

## *New* Gray Box

The gray box is a boundary outside of which green boxes are suppressed. That is,
in rail mode the area outside of the grey box is ignored.

Similarly to the ARM box, one can adjust the gray box by passing
```
_gray_box_right:="10" _gray_box_right:="20" _gray_box_right:="30" _gray_box_right:="40" 
```
which sets the top left corner of the gray box to (10,20) and the bottom right to (640-30,480-40).

## *Experimental* Filter Time

There is another node provided called filter_time. It publishes to
```
/filter_time/filtered_image
/filter_time/filtered_pointcloud
```
and now is a good time to mention that you can redirect input to oberlin_detection with
```
_image_topic:="/filter_time/filtered_image"
_pc_topic:="/filter_time/filtered_pointcloud"
```

## Thanks :metal:
Thanks for reading. I hope this is at least entertaining.

:japanese_goblin:

