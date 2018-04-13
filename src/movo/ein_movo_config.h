#ifndef _EIN_MOVO_CONFIG_H_
#define _EIN_MOVO_CONFIG_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>


#include "ein_words.h"
#include "config.h"
#include "camera.h"


class EinMovoConfig {
 public:
  MachineState * ms;
  EinMovoConfig(MachineState * ms);

  ros::Subscriber kinova_pose_sub;
  ros::Subscriber kinova_finger_pose_sub;
  //actionlib::SimpleActionClient<movo_msgs::ArmPoseAction> kinova_arm_pose_action;
  //actionlib::SimpleActionClient<movo_msgs::SetFingersPositionAction> kinova_fingers_position_action;

  ros::NodeHandle n;
  void endpointCallback(const geometry_msgs::PoseStamped& _p);
  void fingerCallback(const movo_msgs::FingerPosition p);

  eePose candleLikePose;
  eePose calibrationPose;

  double finger1;
  double finger2;
  double finger3;
 

};

#endif /* _EIN_MOVO_CONFIG_H_ */
