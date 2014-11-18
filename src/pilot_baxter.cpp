typedef struct {
  double px;
  double py;
  double pz;

  double ox;
  double oy;
  double oz;

  double qx;
  double qy;
  double qz;
  double qw;
} eePose;

// TODO patch targetting
// TODO redo cartesian autopilot to account for ee coordinate frame
// TODO make sure chirality is accounted for properly in autopilot

#define PROGRAM_NAME "pilot_baxter"
#define MY_FONT FONT_HERSHEY_SIMPLEX

// if autopilot takes too long, it gets angry and takes a shot on a yellow frame
int take_yellow_thresh = 3600;
int autoPilotFrameCounter = 0;

int ik_reset_thresh = 3600;
int ik_reset_counter = 0;
int ikInitialized = 0.0;
double ikShare = 1.0;
double oscillating_ikShare = .1;
double default_ikShare = 1.0;
int oscillatingPeriod = 3200;
//double oscillatingInvAmplitude = 3.0;
double oscillatingInvAmplitude = 4.0;
double oscillatingBias = 0.4;

int oscillating = 0;
int oscillatingSign = 1;
int oscillatorTimerThresh = 600;

int current_instruction = 'C';
double tap_factor = 0.1;
#include <vector>
int execute_stack = 0;
std::vector<int> pilot_call_stack;

int go_on_lock = 0;
int slf_thresh = 5;
int successive_lock_frames = 0;

int lock_reset_thresh = 1800;
int lock_status = 0; // TODO enum

//double slow_aim_factor = 0.5;
double slow_aim_factor = 0.75;
int aim_thresh = 20;
int lock_thresh = 5;

int timerCounter = 0;
int timerThresh = 16;

int timesTimerCounted = 0;

double prevPx = 0;
double prevPy = 0;
//double Kd = 0.00001; // for P only
double Kd = 0.002;
double Kp = 0.0004;
double cCutoff = 20.0;


double a_thresh_close = .11;
double a_thresh_far = .3;
double eeRange = 0.0;

double bDelta = .01;
double approachStep = .0005;
double hoverMultiplier = 0.5;


int zero_g_toggle = 0;
int holding_pattern = 0; // TODO enum
int auto_pilot = 0;

#include <tf/transform_listener.h>
tf::TransformListener* tfListener;
double tfPast = 10.0;

#include <ctime>
#include <sstream>
#include <iostream>
#include <math.h>
#include <string>

std::string wristViewName = "Wrist View";
std::string coreViewName = "Core View";


int reticleHalfWidth = 30;
int pilotTargetHalfWidth = 15;

eePose defaultRightReticle = {.px = 321, .py = 154, .pz = 0.0,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};
eePose defaultLeftReticle = {.px = 328, .py = 113, .pz = 0.0,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};

eePose defaultReticle = defaultRightReticle;
eePose reticle = defaultReticle;

eePose beeLHome = {.px = 0.657579481614, .py = 0.851981417433, .pz = 0.0388352386502,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = -0.366894936773, .qy = 0.885980397775, .qz = 0.108155782462, .qw = 0.262162481772};
eePose beeRHome = {.px = 0.657579481614, .py = -0.168019, .pz = 0.0388352386502,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = -0.366894936773, .qy = 0.885980397775, .qz = 0.108155782462, .qw = 0.262162481772};

eePose workCenter = {.px = 0.686428, .py = -0.509836, .pz = 0.0883011,
		     .ox = 0.0, .oy = 0.0, .oz = 0.0,
		     .qx = -0.435468, .qy = 0.900181, .qz = 0.00453569, .qw = 0.00463141};

eePose beeHome = beeRHome;

eePose pilotTarget = beeHome;

eePose lastGoodEEPose = beeHome;
eePose currentEEPose = beeHome;

eePose eepReg1 = workCenter;
eePose eepReg2 = beeHome;
eePose eepReg3 = beeHome;
eePose eepReg4 = beeHome;

eePose crane1right = {.px = 0.0448714, .py = -1.04476, .pz = 0.698522,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.631511, .qy = 0.68929, .qz = -0.25435, .qw = 0.247748};
eePose crane2right = {.px = 0.617214, .py = -0.301658, .pz = 0.0533165,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.0328281, .qy = 0.999139, .qz = 0.00170545, .qw = 0.0253245};
eePose crane3right = {.px = 0.668384, .py = 0.166692, .pz = -0.120018,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.0328281, .qy = 0.999139, .qz = 0.00170545, .qw = 0.0253245};
eePose crane1left = {.px = -0.0155901, .py = 0.981296, .pz = 0.71078,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.709046, .qy = -0.631526, .qz = -0.226613, .qw = -0.216967};
eePose crane2left = {.px = 0.646069, .py = 0.253621, .pz = 0.0570906,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.999605, .qy = -0.0120443, .qz = 0.0253545, .qw = -0.00117847};
eePose crane3left = {.px = 0.652866, .py = -0.206966, .pz = -0.130561,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.999605, .qy = -0.0120443, .qz = 0.0253545, .qw = -0.00117847};

std::string left_or_right_arm = "right";

eePose ik_reset_eePose = beeHome;

#include <baxter_core_msgs/EndpointState.h>
#include <sensor_msgs/Range.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>

#include <dirent.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"

#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <cv.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Eigen::Vector3d eeForward;
geometry_msgs::Pose trueEEPose;

int bfc = 0;
int bfc_period = 3;
int resend_times = 1;

baxter_core_msgs::SolvePositionIK ikRequest;

ros::ServiceClient ikClient;
ros::Publisher joint_mover;
ros::Publisher gripperPub;


void endpointCallback(const baxter_core_msgs::EndpointState& eps) {
  trueEEPose = eps.pose;
}

void rangeCallback(const sensor_msgs::Range& range) {
  eeRange = range.range;
  //cout << eeRange << endl;
}


void update_baxter(ros::NodeHandle &n) {
//cout << "block5" << endl;
  bfc = bfc % bfc_period;


  baxter_core_msgs::SolvePositionIK thisIkRequest;
  thisIkRequest.request.pose_stamp.resize(1);

  thisIkRequest.request.pose_stamp[0].header.seq = 0;
  thisIkRequest.request.pose_stamp[0].header.stamp = ros::Time::now();
  thisIkRequest.request.pose_stamp[0].header.frame_id = "/base";

  
  thisIkRequest.request.pose_stamp[0].pose.position.x = currentEEPose.px;
  thisIkRequest.request.pose_stamp[0].pose.position.y = currentEEPose.py;
  thisIkRequest.request.pose_stamp[0].pose.position.z = currentEEPose.pz;

  /* global cartesian update 
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(currentEEPose.ox*M_PI, Eigen::Vector3f::UnitX())
  * Eigen::AngleAxisf(currentEEPose.oy*M_PI, Eigen::Vector3f::UnitY())
  * Eigen::AngleAxisf(currentEEPose.oz*M_PI, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf eeRotator(m);
  Eigen::Quaternionf eeBaseQuat(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);

  eeBaseQuat = eeRotator * eeBaseQuat;
  */ 

  /* end effector local angular update */
  {
    Eigen::Vector3f localUnitX;
    {
      Eigen::Quaternionf qin(0, 1, 0, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitX.x() = qout.x();
      localUnitX.y() = qout.y();
      localUnitX.z() = qout.z();
    }

    Eigen::Vector3f localUnitY;
    {
      Eigen::Quaternionf qin(0, 0, 1, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitY.x() = qout.x();
      localUnitY.y() = qout.y();
      localUnitY.z() = qout.z();
    }

    Eigen::Vector3f localUnitZ;
    {
      Eigen::Quaternionf qin(0, 0, 0, 1);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitZ.x() = qout.x();
      localUnitZ.y() = qout.y();
      localUnitZ.z() = qout.z();
    }

    double sinBuff = 0.0;
    double angleRate = 1.0;
    Eigen::Quaternionf eeBaseQuat(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
    sinBuff = sin(angleRate*currentEEPose.ox/2.0);
    Eigen::Quaternionf eeRotatorX(cos(angleRate*currentEEPose.ox/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
    sinBuff = sin(angleRate*currentEEPose.oy/2.0);
    Eigen::Quaternionf eeRotatorY(cos(angleRate*currentEEPose.oy/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
    sinBuff = sin(angleRate*currentEEPose.oz/2.0);
    Eigen::Quaternionf eeRotatorZ(cos(angleRate*currentEEPose.oz/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
    currentEEPose.ox = 0;
    currentEEPose.oy = 0;
    currentEEPose.oz = 0;
    eeRotatorX.normalize();
    eeRotatorY.normalize();
    eeRotatorZ.normalize();
    //eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * 
		  //eeBaseQuat * 
		//eeRotatorZ.conjugate() * eeRotatorY.conjugate() * eeRotatorX.conjugate();
    eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
    eeBaseQuat.normalize();

    thisIkRequest.request.pose_stamp[0].pose.orientation.x = eeBaseQuat.x();
    thisIkRequest.request.pose_stamp[0].pose.orientation.y = eeBaseQuat.y();
    thisIkRequest.request.pose_stamp[0].pose.orientation.z = eeBaseQuat.z();
    thisIkRequest.request.pose_stamp[0].pose.orientation.w = eeBaseQuat.w();

    currentEEPose.qx = eeBaseQuat.x();
    currentEEPose.qy = eeBaseQuat.y();
    currentEEPose.qz = eeBaseQuat.z();
    currentEEPose.qw = eeBaseQuat.w();
  }

//cout << "block6" << endl;

  int ikResult = 0;


  // do not start in a state with ikShare 
  if ((drand48() <= ikShare) || !ikInitialized) {
    ikResult = (!ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);

  /*
    if ( ikClient.waitForExistence(ros::Duration(1, 0)) ) {
  //cout << "block6.1" << endl;
      ikResult = (!ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);
    } else {
      cout << "waitForExistence timed out" << endl;
      ikResult = 1;
    }
  */

    if (ikResult) 
    {
      ROS_ERROR_STREAM("ikClient says pose request is invalid.");
      ik_reset_counter++;

      if (ik_reset_counter > ik_reset_thresh)
	currentEEPose = ik_reset_eePose;
      else
	currentEEPose = lastGoodEEPose;

      return;
    }
    ik_reset_counter = 0;

    lastGoodEEPose = currentEEPose;
    ikRequest = thisIkRequest;
    ikInitialized = 1;
  }
  
//cout << "block7" << endl;

  int numJoints = 7;

  /*
  // using the joint controllers
  // rosmsg show control_msgs/FollowJointTrajectoryAction
  control_msgs::FollowJointTrajectoryActionGoal goal;

  goal.header.seq = 0;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "base";
  goal.goal.trajectory.header = goal.header;

  goal.goal.trajectory.points.resize(1);
  goal.goal.trajectory.points[0].positions.resize(7);
  goal.goal.trajectory.points[0].velocities.resize(7);
  goal.goal.trajectory.points[0].time_from_start = ros::Duration(2.0);


  for (int j = 0; j < numJoints; j++) {
    goal.goal.trajectory.joint_names.push_back(ikRequest.response.joints[0].name[j]);
    goal.goal.trajectory.points[0].positions[j] = ikRequest.response.joints[0].position[j];
    goal.goal.trajectory.points[0].velocities[j] = 0;
  }


  cout << "1" << endl;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> myClient(n, "/robot/left_velocity_trajectory_controller/follow_joint_trajectory", true);
  cout << "2" << endl;
  myClient.waitForServer();
  cout << "3" << endl;
  myClient.sendGoal(goal.goal);
  cout << "4" << endl;
  myClient.waitForResult(ros::Duration(5.0));
  cout << "5" << endl;
  if (myClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  */ 

  // but in theory we can bypass the joint controllers by publishing to this topic
  // /robot/limb/left/joint_command

  baxter_core_msgs::JointCommand myCommand;

  myCommand.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  myCommand.command.resize(numJoints);
  myCommand.names.resize(numJoints);

  for (int j = 0; j < numJoints; j++) {
    myCommand.names[j] = ikRequest.response.joints[0].name[j];
    myCommand.command[j] = ikRequest.response.joints[0].position[j];
  }
//cout << "block8" << endl;

  for (int r = 0; r < resend_times; r++) {
    joint_mover.publish(myCommand);
  }

  bfc++;
//cout << "block9" << endl;
}


void timercallback1(const ros::TimerEvent&) {


//cout << "block1" << endl;
  //eePose redTargetEEPose;

  eePose redTargetEEPose = beeHome;

  // target a chosen red box
  /*
  if ((numRedBoxes > 0) && (redTrackbarVariable > 0))
    if (redBoxes[redTrackbarVariable - 1].persistence > persistenceThresh) {
      tf::StampedTransform transform;
      try {
	geometry_msgs::PointStamped pStampedIn;
	geometry_msgs::PointStamped pStampedOut;
	pStampedIn.header.seq = 0;
	pStampedIn.header.stamp = ros::Time::now() - ros::Duration(tfPast);
	pStampedIn.header.frame_id = "/camera_rgb_optical_frame";
	pStampedIn.point.x = redBoxes[redTrackbarVariable - 1].com.px;
	pStampedIn.point.y = redBoxes[redTrackbarVariable - 1].com.py;
	pStampedIn.point.z = redBoxes[redTrackbarVariable - 1].com.pz;

//	try {
//	    //listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(10.0) );
//	    //listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
//	    tfListener->waitForTransform("/base", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0) );
//	    tfListener->transformPoint("/base", pStampedIn, pStampedOut); 
//	} catch (tf::TransformException ex) {
//	    ROS_ERROR("%s",ex.what());
//	}


	tfListener->transformPoint("/base", pStampedIn, pStampedOut); 



	redTargetEEPose.px = pStampedOut.point.x;
	redTargetEEPose.py = pStampedOut.point.y;
	redTargetEEPose.pz = pStampedOut.point.z;
	redTargetEEPose.ox = currentEEPose.ox; 
	redTargetEEPose.oy = currentEEPose.oy;
	redTargetEEPose.oz = currentEEPose.oz;
      } catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	//ROS_ERROR("unable to lookup transform...");
      }
    }
  */

//cout << "block2" << endl;
  //ROS_INFO("Callback 1 triggered");
  ros::NodeHandle n("~");

  int c = cvWaitKey(1);
  int takeSymbol = 1;
  if (c != -1) {
    cout << "You pressed " << c << " which when shifted is " << c + 65504 << " and the state is " << currentEEPose.ox << " " << currentEEPose.oy << " " <<currentEEPose.oz << endl;
    takeSymbol = 0;
  }
//cout << "block3" << endl;


  
  if (!auto_pilot)
    autoPilotFrameCounter = 0;
  // both of these could be moved into the symbolic logic
  // if we have a lock, proceed to grab 
  if ((auto_pilot && go_on_lock && (successive_lock_frames >= slf_thresh)) || ((autoPilotFrameCounter > take_yellow_thresh) && (lock_status == 1))) {
    lock_status = 0;
    successive_lock_frames = 0;
    auto_pilot = 0;
    go_on_lock = 0;
    c = -1;
    zero_g_toggle = 0;
    holding_pattern = 1;
    autoPilotFrameCounter = 0;
  }
  
  // deal with the stack
  if (execute_stack && takeSymbol) {
    if (pilot_call_stack.size() > 0) {
      c = pilot_call_stack.back();
      pilot_call_stack.pop_back();
      current_instruction = c;
    } else {
      execute_stack = 0;
    }
  }

  if (oscillating && (lock_status == 0) && (holding_pattern == 0) && (timerCounter >= oscillatorTimerThresh)) {
    currentEEPose.py = eepReg2.py + oscillatingSign*(oscillatingBias + sin( 2.0 * 3.1415926 * double(timesTimerCounted % oscillatingPeriod) / oscillatingPeriod)) / oscillatingInvAmplitude;
    currentEEPose.px = eepReg2.px;
    currentEEPose.pz = eepReg2.pz;
    ikShare = oscillating_ikShare;
  } else {
    ikShare = default_ikShare;
  }

  if (timerCounter >= lock_reset_thresh) {
    lock_status = 0;
  }
  

  switch (c) {
    case 30: // up arrow
      break;
    case 'k':
      {
	baxter_core_msgs::EndEffectorCommand command;
	command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	command.args = "{\"position\": 100.0}";
	command.id = 65538;
	gripperPub.publish(command);
      }
      break;
    case 'l':
      oscillating = !oscillating;
      break;
    case 'r':
      pilot_call_stack.resize(0);
      break;
    case 't':
      currentEEPose.pz += bDelta * tap_factor;
      break;
    case '7':
      {
	//double deltaX = workCenter.px - currentEEPose.px;
	//double deltaY = workCenter.py - currentEEPose.py;
	double deltaX = eepReg2.px - currentEEPose.px;
	double deltaY = eepReg2.py - currentEEPose.py;
	double xTimes = fabs(floor(deltaX / bDelta)); 
	double yTimes = fabs(floor(deltaY / bDelta)); 

	int tapTimes = 30;

	int numNoOps = 6*(yTimes + xTimes + floor(2 * tapTimes * tap_factor));
	for (int cc = 0; cc < numNoOps; cc++) {
	  pilot_call_stack.push_back('C');
	}

	if (deltaX > 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('e');
	if (deltaX < 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('q');
	if (deltaY > 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('d');
	if (deltaY < 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('a');
      

	for (int tc = 0; tc < tapTimes; tc++) {
	  pilot_call_stack.push_back('t');
	}
      }
      break;
    case '8': // load program 8
      {
	//double deltaX = workCenter.px - currentEEPose.px;
	//double deltaY = workCenter.py - currentEEPose.py;
	double deltaX = eepReg3.px - currentEEPose.px;
	double deltaY = eepReg3.py - currentEEPose.py;
	double xTimes = fabs(floor(deltaX / bDelta)); 
	double yTimes = fabs(floor(deltaY / bDelta)); 

	int tapTimes = 30;

	int numNoOps = 6*(yTimes + xTimes + floor(2 * tapTimes * tap_factor));
	for (int cc = 0; cc < numNoOps; cc++) {
	  pilot_call_stack.push_back('C');
	}

	if (deltaX > 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('e');
	if (deltaX < 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('q');
	if (deltaY > 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('d');
	if (deltaY < 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('a');
      

	for (int tc = 0; tc < tapTimes; tc++) {
	  pilot_call_stack.push_back('t');
	}
      }
      break;
    case '9': // load program 9
      {
	pilot_call_stack.push_back('9');
	pilot_call_stack.push_back('x');

	pilot_call_stack.push_back('7');
	pilot_call_stack.push_back('k');
	pilot_call_stack.push_back('8');

	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back(65616);
	pilot_call_stack.push_back('C');
      }
      break;
    case '0': // load program 0
      {
	pilot_call_stack.push_back('0');
	pilot_call_stack.push_back('x');
	int tapTimes = 30;
	for (int tc = 0; tc < tapTimes; tc++) {
	  pilot_call_stack.push_back('t');
	}
	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back(65616);
	pilot_call_stack.push_back('C');
      }
      break;
    case 'y': // execute stack
      execute_stack = 1;
      break;
    case 'u':
      cout << "Current EE Position (x,y,z): " << currentEEPose.px << " " << currentEEPose.py << " " << currentEEPose.pz << endl;
      cout << "Current EE Orientation (x,y,z,w): " << currentEEPose.qx << " " << currentEEPose.qy << " " << currentEEPose.qz << " " << currentEEPose.qw << endl;
      break;
    case 'i':
      {
	baxter_core_msgs::EndEffectorCommand command;
	command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
	command.id = 65538;
	gripperPub.publish(command);
      }
      break;
    case 'o':
      reticle = defaultReticle;
      break;
    case 'p':
      auto_pilot = !auto_pilot;
      break;
    case 65616: // 'P'
      auto_pilot = 1;
      go_on_lock = 1;
      break;
    case 'x':
      holding_pattern = -1;
      break;
    case 'c':
      {
	holding_pattern = 0;
	auto_pilot = 0;
	go_on_lock = 0;
	execute_stack = 0;
      }
      break;
    case 'C': // continue OR no-op
      {
	if (auto_pilot || (holding_pattern != 0)) {
	  pilot_call_stack.push_back('C');
	} else {
	  holding_pattern = 0;
	  auto_pilot = 0;
	  go_on_lock = 0;
	}
      }
      break;
    case 'v':
      holding_pattern = 1;
      break;
    case 'b':
      holding_pattern = 2;
      break;
    case 'z':
      zero_g_toggle = !zero_g_toggle;
      break;
    // begin cartesian controls
    case 'a':
      currentEEPose.py -= bDelta;
      break;
    case 'd':
      currentEEPose.py += bDelta;
      break;
    case 'w':
      currentEEPose.pz += bDelta;
      break;
    case 's':
      currentEEPose.pz -= bDelta;
      break;
    case 'q':
      currentEEPose.px -= bDelta;
      break;
    case 'e':
      currentEEPose.px += bDelta;
      break;
    // begin angular controls
    /* Global angular controls
    case 'a'+65504:
      currentEEPose.oy -= bDelta;
      break;
    case 'd'+65504:
      currentEEPose.oy += bDelta;
      break;
    case 'w'+65504:
      currentEEPose.oz += bDelta;
      break;
    case 's'+65504:
      currentEEPose.oz -= bDelta;
      break;
    case 'q'+65504:
      currentEEPose.ox -= bDelta;
      break;
    case 'e'+65504:
      currentEEPose.ox += bDelta;
      break;
    */
    /* Local angular controls */
    case 'w'+65504:
      currentEEPose.oy -= bDelta;
      break;
    case 's'+65504:
      currentEEPose.oy += bDelta;
      break;
    case 'e'+65504:
      currentEEPose.oz += bDelta;
      break;
    case 'q'+65504:
      currentEEPose.oz -= bDelta;
      break;
    case 'a'+65504:
      currentEEPose.ox -= bDelta;
      break;
    case 'd'+65504:
      currentEEPose.ox += bDelta;
      break;
    // begin register assignment controls
    case 65568+1: // !
      eepReg1 = currentEEPose;
      break;
    case 65600: // @
      eepReg2 = currentEEPose;
      break;
    case 65568+3: // #
      eepReg3 = currentEEPose;
      break;
    case 65568+4: // $
      eepReg4 = currentEEPose;
      break;
    // begin register recall controls
    case '1':
      currentEEPose = eepReg1;
      break;
    case '2':
      currentEEPose = eepReg2;
      break;
    case '3':
      currentEEPose = eepReg3;
      break;
    case '4':
      currentEEPose = eepReg4;
      break;
    // begin register target acquisition calls
    case 1+262192:
      eepReg1 = redTargetEEPose;
      break;
    case 2+262192:
      eepReg2 = redTargetEEPose;
      break;
    case 3+262192:
      eepReg3 = redTargetEEPose;
      break;
    case 4+262192:
      eepReg4 = redTargetEEPose;
      break;
    default:
      break;
  }

//cout << "block4" << endl;


  // stateful updates
  // this could be moved into the 'no-op' / 'continue' logic
  if (!zero_g_toggle) {

    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);

    qout = eeqform * qin * eeqform.conjugate();

    eeForward.x() = qout.x();
    eeForward.y() = qout.y();
    eeForward.z() = qout.z();

    // autopilot update
    // XXX TODO this really needs to be in the coordinate frame of the
    //  end effector to be general enough to use. now we are assuming that
    //  we are aligned with the table.
    if (auto_pilot && (timerCounter < timerThresh)) {
      currentEEPose.ox = 0.0;
      currentEEPose.oy = 0.0;
      currentEEPose.oz = 0.0;

      double PxPre = reticle.px - pilotTarget.px; 
      double PyPre = reticle.py - pilotTarget.py;

      double Px = PxPre;
      double Py = PyPre;

      double thisDelta = bDelta;
      lock_status = 0;
      if ((fabs(Px) < aim_thresh) && (fabs(Py) < aim_thresh)) {
	thisDelta = bDelta * slow_aim_factor;
	lock_status = 1;
      } 
      if ((fabs(Px) < lock_thresh) && (fabs(Py) < lock_thresh)) {
	thisDelta = bDelta * slow_aim_factor;
	lock_status = 2;
      }
      // angular control
      //currentEEPose.oy -= thisDelta*Kp*Py;
      //currentEEPose.ox -= thisDelta*Kp*Px;
  
      // cartesian control
      // square is flatter near 0 and we need the si
      /*
      if (Px < -cCutoff)
	Px = -cCutoff;
      if (Px > cCutoff)
	Px = cCutoff;
      if (Py < -cCutoff)
	Py = -cCutoff;
      if (Py > cCutoff)
	Py = cCutoff;
      */

      double Dx = pilotTarget.px - prevPx;
      double Dy = pilotTarget.py - prevPy;

      // parallel update
      //double pTermX = thisDelta*Kp*Px*fabs(Px);
      //double pTermY = thisDelta*Kp*Py*fabs(Py);
      double pTermX = thisDelta*Kp*Px;
      double pTermY = thisDelta*Kp*Py;
      double dTermX = -thisDelta*Kd*Dx;
      double dTermY = -thisDelta*Kd*Dy;
      //currentEEPose.px -= (thisDelta*Kp*Px*fabs(Px);
      //currentEEPose.py += (thisDelta*Kp*Py*fabs(Py);
      //currentEEPose.px -= pTermX;
      //currentEEPose.py += pTermY;
      currentEEPose.px -= pTermX + dTermX;
      currentEEPose.py += pTermY + dTermY;

      autoPilotFrameCounter++;


      cout << "ikShare: " << ikShare << " Px: " << Px << " Py: " << Py << "      Dx: " << Dx << " Dy: " << Dy << "      lock_status: " << lock_status << " slf: " << successive_lock_frames << " slf_t: " << slf_thresh << endl;
    }

    // holding pattern update
    switch (holding_pattern) {
      case -1:
	{
	  if (eeRange < a_thresh_far) {
	    currentEEPose.px -= eeForward.x()*approachStep;
	    currentEEPose.py -= eeForward.y()*approachStep;
	    currentEEPose.pz -= eeForward.z()*approachStep;
	  } else {
	    holding_pattern = 0;
	  }
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 100.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	}
	break;
      case 0:
	break;
      case 1:
	if (eeRange > a_thresh_close) {
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 100.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	  currentEEPose.px += eeForward.x()*approachStep;
	  currentEEPose.py += eeForward.y()*approachStep;
	  currentEEPose.pz += eeForward.z()*approachStep;
	} else {
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 0.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	  holding_pattern = 0;
	}
	break;
      case 2:
	{
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 100.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	  currentEEPose.qx = -0.283134;
	  currentEEPose.qy = 0.958744;
	  currentEEPose.qz = -0.00634737;
	  currentEEPose.qw = 0.0245754;

	  if (eeRange > a_thresh_far) {
	    currentEEPose.px += eeForward.x()*approachStep*hoverMultiplier;
	    currentEEPose.py += eeForward.y()*approachStep*hoverMultiplier;
	    currentEEPose.pz += eeForward.z()*approachStep*hoverMultiplier;
	  } else {
	    currentEEPose.px -= eeForward.x()*approachStep*hoverMultiplier;
	    currentEEPose.py -= eeForward.y()*approachStep*hoverMultiplier;
	    currentEEPose.pz -= eeForward.z()*approachStep*hoverMultiplier;
	  }
	}
	break;
    }
  }

  if (!zero_g_toggle) {
    update_baxter(n);
  }
  else {
    currentEEPose.px = trueEEPose.position.x;
    currentEEPose.py = trueEEPose.position.y;
    currentEEPose.pz = trueEEPose.position.z;
    currentEEPose.qx = trueEEPose.orientation.x;
    currentEEPose.qy = trueEEPose.orientation.y;
    currentEEPose.qz = trueEEPose.orientation.z;
    currentEEPose.qw = trueEEPose.orientation.w;
    currentEEPose.ox = 0.0;
    currentEEPose.oy = 0.0;
    currentEEPose.oz = 0.0;
  }


  //cv::waitKey(1);
  timerCounter++;
  timesTimerCounted++;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //cam_img = cv_ptr->image;
    //real_img = true;
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  {
    cv::Point outTop = cv::Point(reticle.px-reticleHalfWidth, reticle.py-reticleHalfWidth);
    cv::Point outBot = cv::Point(reticle.px+reticleHalfWidth, reticle.py+reticleHalfWidth);
    cv::Point inTop = cv::Point(reticle.px+1-reticleHalfWidth,reticle.py+1-reticleHalfWidth);
    cv::Point inBot = cv::Point(reticle.px-1+reticleHalfWidth,reticle.py-1+reticleHalfWidth);

    if (auto_pilot) {
      int boxBrightness = 128;
      if (go_on_lock)
	boxBrightness = 255;
      Mat vCrop = cv_ptr->image(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      cv::Scalar fillColor(0,boxBrightness,0);
      switch (lock_status) {
	case 0:
	  break;
	case 1:
	  fillColor = cv::Scalar(0,boxBrightness,boxBrightness);
	  break;
	case 2:
	  fillColor = cv::Scalar(0,0,boxBrightness);
	  break;
      }
      vCrop = vCrop + fillColor;
    }

    rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(82,70,22)); // RGB: 22 70 82
    rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(239,205,68)); // RGB: 68 205 239
  }

  {
    cv::Point outTop = cv::Point(pilotTarget.px-pilotTargetHalfWidth, pilotTarget.py-pilotTargetHalfWidth);
    cv::Point outBot = cv::Point(pilotTarget.px+pilotTargetHalfWidth, pilotTarget.py+pilotTargetHalfWidth);
    cv::Point inTop = cv::Point(pilotTarget.px+1-pilotTargetHalfWidth,pilotTarget.py+1-pilotTargetHalfWidth);
    cv::Point inBot = cv::Point(pilotTarget.px-1+pilotTargetHalfWidth,pilotTarget.py-1+pilotTargetHalfWidth);
    rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(53,10,97)); // RGB: 97 10 53
    rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(142,31,255)); // RGB: 255 31 142
  }

  cv::imshow(wristViewName, cv_ptr->image);

  //Mat coreImage = cv_ptr->image.clone();
  Mat coreImage(2*cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
  coreImage = 0.0*coreImage;

  cv::Scalar dataColor(192,192,192);
  cv::Scalar labelColor(160,160,160);

  cv::Point ciAnchor(10,50);
  putText(coreImage, "Current Registers: ", ciAnchor, MY_FONT, 0.5, labelColor, 1.0);

  char buf[256];
  cv::Point lAnchor(170,50);
  string lText;
  lText += "CI: ";
  lText += current_instruction;
  lText += "  ZG: ";
  sprintf(buf, "%d", zero_g_toggle);
  lText += buf;
  lText += "  HP: ";
  sprintf(buf, "%d", holding_pattern);
  lText += buf;
  lText += "  AP: ";
  sprintf(buf, "%d", auto_pilot);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "LS: ";
  sprintf(buf, "%d", lock_status);
  lText += buf;
  lText += "  GOL: ";
  sprintf(buf, "%d", go_on_lock);
  lText += buf;
  lText += "  SLF: ";
  sprintf(buf, "%d", successive_lock_frames);
  lText += buf;
  lText += "  O: ";
  sprintf(buf, "%d", oscillating);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "TYT: ";
  sprintf(buf, "%d", take_yellow_thresh);
  lText += buf;
  lText += " APFC: ";
  sprintf(buf, "%d", autoPilotFrameCounter);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  int stackRowY = 120;
  cv::Point csAnchor(10,stackRowY);
  putText(coreImage, "Call Stack: ", csAnchor, MY_FONT, 0.5, labelColor, 1.0);
  
  int instructionsPerRow = 25;
  int rowAnchorStep = 25;
  cv::Point rowAnchor(120,stackRowY);
  int insCount = 0; 
  while (insCount < pilot_call_stack.size()) {
    string outRowText;

    for (int rowCount = 0; (insCount < pilot_call_stack.size()) && (rowCount < instructionsPerRow); insCount++, rowCount++) {
      outRowText += pilot_call_stack[insCount];
      outRowText += " ";
    }

    putText(coreImage, outRowText, rowAnchor, MY_FONT, 0.5, dataColor, 2.0);
    rowAnchor.y += rowAnchorStep;
  }

  cv::imshow(coreViewName, coreImage);
}

void targetCallback(const geometry_msgs::Point& point) {
  prevPx = pilotTarget.px;
  prevPy = pilotTarget.py;

  pilotTarget.px = point.x;
  pilotTarget.py = point.y;
  pilotTarget.pz = point.z;
      
  //cout << ">>received target<<" << endl;

  timerCounter = 0;

  if (lock_status == 2)
    successive_lock_frames++;
  else
    successive_lock_frames = 0;
}

void CallbackFunc(int event, int x, int y, int flags, void* userdata) {
  if ( event == EVENT_LBUTTONDOWN ) {
    //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    reticle.px = x;
    reticle.py = y;
    cout << "x: " << x << " y: " << y << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void loadROSParamsFromArgs()
{
  ros::NodeHandle nh("~");
  nh.getParam("left_or_right_arm", left_or_right_arm);
  nh.getParam("default_ikShare", default_ikShare);
}


int main(int argc, char **argv) {

  // initialize non-ROS
  srand(time(NULL));
  eeForward = Eigen::Vector3d(1,0,0);

  cout << "argc: " << argc << endl;
  for (int ccc = 0; ccc < argc; ccc++) {
    cout << argv[ccc] << endl;
  }
  cout << "argc: " << argc << endl;

  string programName;
  if (argc > 1) {
    programName = string(PROGRAM_NAME) + "_" + argv[argc-1];
    cout << programName << endl;
  }
  else
    programName = string(PROGRAM_NAME);


  // initialize ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle n("~");

  loadROSParamsFromArgs();

  if (0 == left_or_right_arm.compare("left")) {
    cout << "Possessing left arm..." << endl;
    beeHome = crane1left;
    eepReg1 = crane1left;
    eepReg2 = crane2left;
    eepReg3 = crane3left;
    eepReg4 = beeLHome;
    oscillatingSign = 1;
    defaultReticle = defaultLeftReticle;
    reticle = defaultReticle;
  } else if (0 == left_or_right_arm.compare("right")) {
    cout << "Possessing right arm..." << endl;
    beeHome = crane1right;
    eepReg1 = crane1right;
    eepReg2 = crane2right;
    eepReg3 = crane3right;
    eepReg4 = beeRHome;
    oscillatingSign = -1;
    defaultReticle = defaultRightReticle;
    reticle = defaultReticle;
  } else {
    cout << "Invalid chirality. Exiting." << endl;
    exit(0);
  }
  pilotTarget = beeHome;
  lastGoodEEPose = beeHome;
  currentEEPose = beeHome;

  ik_reset_eePose = eepReg2;

  ros::Subscriber epState = n.subscribe("/robot/limb/" + left_or_right_arm + "/endpoint_state", 1, endpointCallback);
  ros::Subscriber eeRanger = n.subscribe("/robot/range/" + left_or_right_arm + "_hand_range/state", 1, rangeCallback);
  ros::Subscriber eeTarget = n.subscribe("/publish_detections_" + left_or_right_arm + "/pilot_target_" + left_or_right_arm, 1, targetCallback);

  wristViewName = "Wrist View " + left_or_right_arm;
  coreViewName = "Core View " + left_or_right_arm;

  cv::namedWindow(wristViewName);
  cv::setMouseCallback(wristViewName, CallbackFunc, NULL);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  std::string image_topic = "/cameras/" + left_or_right_arm + "_hand_camera/image";
  image_sub = it.subscribe(image_topic, 1, imageCallback);

  cv::namedWindow(coreViewName);

  ros::Timer timer1 = n.createTimer(ros::Duration(0.01), timercallback1);

  tfListener = new tf::TransformListener();

  ikClient = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/" + left_or_right_arm + "/PositionKinematicsNode/IKService");
  joint_mover = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + left_or_right_arm + "/joint_command", 10);
  gripperPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/" + left_or_right_arm + "_gripper/command",10);

  {
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    command.id = 65538;
    gripperPub.publish(command);
  }

  ros::spin();

  // multithreaded spinning causes what is probably a race condition...
  //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  //spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}
