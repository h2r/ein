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
sudo apt-get install qt5-default python-wstool ros-indigo-object-recognition-msgs libgsl0-dev exuberant-ctags  ros-indigo-serial

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

And enter 'r' to start the program from within gdb. For safety purposes, the arm should not move at startup.


