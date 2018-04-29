#ifndef _EIN_MOVO_CONFIG_H_
#define _EIN_MOVO_CONFIG_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <actionlib_msgs/GoalStatusArray.h>


#include <movo_msgs/LinearActuatorCmd.h>
#include <movo_msgs/PanTiltActuatorFdbk.h>
#include <movo_msgs/PVA.h>
#include <movo_msgs/ConfigCmd.h>
#include <movo_msgs/PanTiltFdbk.h>
#include <movo_msgs/PanTiltCmd.h>
#include <movo_msgs/Battery.h>

#include <moveit/move_group_interface/move_group.h>
using namespace moveit;
using namespace planning_interface;


#include "ein_words.h"
#include "config.h"
#include "camera.h"



class EinMovoConfig {
 public:
  MachineState * ms;
  EinMovoConfig(MachineState * ms);
  ros::NodeHandle n;

  movo_msgs::ConfigCmd configMsg;
  ros::Publisher configCmdPub;  

  ros::Publisher cmdVelPub;
  geometry_msgs::Twist twistMsg;

  ros::Subscriber batterySubscriber;
  void batteryCallback(const movo_msgs::Battery& js);
  movo_msgs::Battery batteryMsg;
  bool batteryCharging;

  ros::Time lastMapLookupPrintTime;
  
  
  MoveGroup * upperBody;
  MoveGroup * leftArm;
  MoveGroup * rightArm;
  vector<MoveGroup *> endEffectors;
  int focused_ee = 0;

  ros::Subscriber moveitStatusSubscriber;
  actionlib_msgs::GoalStatusArray moveitStatusMsg;
  void moveitStatusCallback(const actionlib_msgs::GoalStatusArray&m);
  map<string, actionlib_msgs::GoalStatus> goals;
  ros::Time lastMoveitCallTime;

  ros::Subscriber gripperJointSubscriber;
  void gripperJointCallback(const sensor_msgs::JointState& js);
  sensor_msgs::JointState fingerJointState;

  ros::Subscriber torsoJointSubscriber;
  ros::Publisher torsoJointCmdPub;
  movo_msgs::LinearActuatorCmd torsoCmd;
  void torsoJointCallback(const sensor_msgs::JointState& js);
  double torsoGridSize=0.01;
  double targetTorsoJointPosition=0;
  double trueTorsoJointPosition=0;
  double trueTorsoJointVelocity=0;



  ros::Subscriber panTiltFdbkSubscriber;
  ros::Publisher panTiltCmdPub;
  movo_msgs::PanTiltFdbk ptaFdbkMsg;
  movo_msgs::PanTiltCmd ptaCmdMsg;
  void panTiltFdbkCallback(const movo_msgs::PanTiltFdbk& js);  
  double panTiltGridSize=0.1;
  double targetPanPos;
  double targetTiltPos;

  const static unsigned int num_joints=17;  
  const vector<double> homedJoints{-1.5,-0.2,-0.15,-2.0,2.0,-1.24,-1.1, 1.5,0.2,0.15,2.0,-2.0,1.24,1.1,0.35,0,0};
  const vector<double> tuckedJoints{-1.6,-1.4,0.4,-2.7,0.0,0.5,-1.7, 1.6,1.4,-0.4,2.7,0.0,-0.5, 1.7, 0.04, 0, 0};
  const vector<std::string> upperBodyJoints{"right_shoulder_pan_joint",
					    "right_shoulder_lift_joint",
					    "right_arm_half_joint",
					    "right_elbow_joint",
					    "right_wrist_spherical_1_joint",
					    "right_wrist_spherical_2_joint",
					    "right_wrist_3_joint",
					    "left_shoulder_pan_joint",
					    "left_shoulder_lift_joint",
					    "left_arm_half_joint",
					    "left_elbow_joint",
					    "left_wrist_spherical_1_joint",
					    "left_wrist_spherical_2_joint",
					    "left_wrist_3_joint",
					    "linear_joint",
					    "pan_joint",
					    "tilt_joint"};


  double gridSize=0.01;  
  eePose odomPose;
  eePose mapPose;

  eePose leftPose;
  eePose rightPose;

  eePose leftTargetPose;
  eePose rightTargetPose;

  

};

#endif /* _EIN_MOVO_CONFIG_H_ */
