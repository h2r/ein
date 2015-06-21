# Ein

Please read this entire document before attempting to use the
software. We start by describing the underlying functionality and ROS
nodes and proceed to describe a convenient operating strategy and some
utility scripts which you may find helpful.

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
and finally 
```
sudo apt-get install qt5-default
```

**Install** the package h2r/ein by going to catkin_ws/src and then cloning.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone http://github.com/h2r/ein
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
cp baxter/baxter.sh ..
```
edit baxter.sh following the instructions in that file.
```
./baxter.sh
```

Then build:
```
cd .. 
source /opt/ros/indigo/setup.bash
catkin_make
source devel/setup.bash
catkin_make
```
Some other stuff you might need to do when installing fresh:
```
sudo apt-get install ros-indigo-object-recognition-msgs
sudo apt-get install ros-indigo-pcl-ros
sudo apt-get install libgsl0-dev
```

To run the program, from the root of your catkin workspace, run the
following command:
```
catkin_make && gdb --args devel/lib/ein/ein _data_directory:="$(rospack find ein)/ein_dataDefault" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _run_prefix:="ISRR" _left_or_right_arm:="left" left
```

And enter 'r' to start the program from within gdb. If the robot is enabled it should move to a ready position.
    


Ein

2: Shift to ready position on the front table.
capslock + @: assume wholeFoodsCounter1
c r c: clear call stack
capslock + shift d: queues scan program
y: execute what is in the queue
capslock + page up/page down: select target to pick
u:  display where the end effector is located
capslock + q: vision cycle
capslock + b: toggle publish objects
capslock + f: reinitialize and retrain everything
capslock + n: listen for pick requests from fetch command
capslock + v: thompson sampling test
capslock + I: count grasp

capslock + w: queues pick program

capslock + -: samples from Thompson
capslock + =: empty-a

numlock + X: Organize windows
capslock + z: load target class range map into register 1
numlock + 6: select best available grasp (not used)
numlock + ,: select best available grasp (used by capslock + w)
capslock + t: synchronic servo
capslock + Y: quick fetch taragetClass
numlock + a: drawMapRegisters
numlock + +: shift grasp gear

Grasp gear 2 on handle is a good grasp. 

Saving candidates:
capslock + A:
capslock + numlock + u:
capslock + numlock + A:  SAVE GRASP ME
capslock + Z
capslock + numlock + i: set grasp memories from classGraspMemories

numlock + s: select max target NOT cumulative
numlock + S: select max target cumulative
S
capslock + numlock + s, d, f: change current pick mode

capslock + numlock + ;: height learning


Command:                         
catkin_make && gdb --args ./devel/lib/node/ein  _data_directory:="$(rospack find node)/ein_data1" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _left_or_right_arm:="left" _gray_box_left:=90 _gray_box_right:=90 _gray_box_bot:=60 _add_blinders:=0 left


General notes:

crc:  clears the stack
y:  Runs what is on the stack

Training new model:
capslock + shift d: queues scan program


Prior (grasping):
2: move to position 2
capslock + pageup: select target class
capslock + backspace: load prior 
capslock + numlock + s: prior
capslock + w:  queue pick command


Training (grasping):
2: move to position 2
capslock + pageup: select target class
capslock + backspace: load prior (if desired)
capslock + numlock + d: select learning sampling 
capslock + w:  queue pick command
capslock + numlock + A: save the learned numbers

Marginal (grasping):
restart the program
2: move to position 2
capslock + pageup: select target class
capslock + =: load marginals.  Verify they are in Grasp Memory Sample window.
capslock + numlock + d: select learning sampling 
capslock + w:  queue pick command


Training height:
capslock + pageup: select target class
capslock + numlock + backspace: load prior (if desired)
capslock + numlock + :: queue height learning




**********************************************************************
ein_data_height_done (height, no grasp, possibly bad contrast)
ein_data_contrast (no height, no grasp, good contrast)
ein_data (baseline scans, possibly bad contrast)
ein_data_combine1 (height, no grasp, fixes)
 * really good models








Ein MIT Visit

Pull.
Copy the folder node/default/objects to node/ein_dataDefault.
Copy the contents of node/default/config to node/ein_dataDefault.


From the root of your catkin workspace, run the following command:

catkin_make && gdb --args devel/lib/node/ein _data_directory:="$(rospack find node)/ein_dataDefault" _vocab_file:="vocab.yml" _knn_file:="knn.yml" _label_file:="labels.yml" _run_prefix:="ISRR" _left_or_right_arm:="left" left

And enter 'r' to start the program from within gdb. If the robot is enabled it should move to a ready position.

In a different window run:

rosrun node ein_client.py left

to start the repl. Enter:

fillClearanceMap loadIkMap ;

and the Object Map View should change to reflect a mapping of a cross section of the ik space. There should be
red, light red, yellow, and green dots. Note, a space between the last word and the semi-colon is necessary.

Make sure numlock and capslock are off. With the Object Map View focused, use 'a','d','q', and 'e' to move the arm
deep within green territory. Place an object directly under the arm. Run the following command in the repl:

scanObject ;

and follow the instructions in the ein terminal window to scan the object.

At any time, run

guiShowAll ;

from the repl to show all of the windows.

When scanning finishes, models will be trained with an object class corresponding to each subfolder
of ein_dataDefault. Since there are no RGB images for
background, blueBowl, brownCup, and brush, models will be trained for those classes but with no positive examples
for them, nothing will ever be classified as one of those objects. So those folders can be removed at this point
(they were necessary earlier for picking to work with those classes and the pretrained knn and vocab).

loop example:
```
 0 10 start waitUntilAtCurrentPosition xDown xDown xDown xDown waitUntilAtCurrentPosition xUp xUp xUp xUp next ;
```
