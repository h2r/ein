#ifndef _EIN_JACO_CONFIG_H_
#define _EIN_JACO_CONFIG_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include "ein_words.h"
#include "config.h"
#include "camera.h"


class EinJacoConfig {
 public:
  MachineState * ms;
  EinJacoConfig(MachineState * ms);

  ros::Subscriber kinova_pose_sub;
  ros::Subscriber kinova_finger_pose_sub;
  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> kinova_arm_pose_action;
  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> kinova_fingers_position_action;

  ros::NodeHandle n;
  void endpointCallback(const geometry_msgs::PoseStamped& _p);
  void fingerCallback(const kinova_msgs::FingerPosition p);

  double finger1;
  double finger2;
  double finger3;
 

};

#endif /* _EIN_JACO_CONFIG_H_ */
