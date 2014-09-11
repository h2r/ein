oberlin_detection


Make sure you have the following standard components installed:

opencv
cv_bridge
image_transport
openni

Install the non-standard package h2r/bing.

To get up and running you will need to perform three steps.

1.) Gather data for your objects.
2.) "Train" a BoW-kNN classifier.
3.) Publish detections.


To gather data for your objects, point your kinect at your tabletop and run a command similar to:
```
rosrun oberlin_detection capture_object _data_directory:="/home/oberlin/catkin_ws_baxter/src/oberlin_detection/data2" _class_name:="class3" _run_prefix:="june2round1"
```
Make sure that data_directory/class_name is the FULL PATH to an existing directory.

This should open two windows. The first is the view through your kinect, augmented with information, called "Object Viewer". 
The second is an eerie green interpretation of the view called "Density Viewer". Look at the Object Viewer.

You should see one or more blue boxes. Mess with the scene until the only blue box is around your object. Click the Object Viewer
window somewhere in the main part. Check the contents of data_directory/class_name. You should see a saved crop for every
blue box that was on the screen. Notice that run_prefix is included in the file names.

Move the object around the table and capture crops of many different views. When you have enough crops, stop. If you want to
gather more data for the same object, run the command again but i
MAKE SURE to change the run_prefix so you don't overwrite your data.

Run capture_object for every object you want to train, using a different subdirectory of data_directory for each class
you want to train.


Next, train the classifier. Run a command similar to:
```
rosrun oberlin_detection train_classifier _data_directory:="/home/oberlin/catkin_ws_baxter/src/oberlin_detection/data2" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _class_labels:="class2 class3" _class_pose_models:="B S"
```
This will create three files (for vocab, knn, and labels) in the locations specified, paths relative to data_directory. The string
class_labels should be a " "-delimited list of the subdirectories you gathered data in, relative to data_directory. The string
class_pose_models is a " "-delimited list of the pose models for your classes. If your object is most like a bowl (i.e. you don't
care about its rotation in the plane of the table) its entry should be a "B", but if it is most like a spoon and you want
rotational estimates, its entry should be an "S".

Make sure your files were written and they look sensible.


Finally, run something like:
```
rosrun oberlin_detection publish_detections _data_directory:="/home/oberlin/catkin_ws_baxter/src/oberlin_detection/data2" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml"
```
This should bring up two familiar windows. This time, blue boxes should be labeled and table detections should be shown as brown
boxes.

You can subscribe to /oberlin_detection/labeled_objects and /oberlin_detection/object_markers. Open RViz and try viewing the markers.
The table estimate should appear and should align well with the point cloud. Markers should appear for the blue boxes.

