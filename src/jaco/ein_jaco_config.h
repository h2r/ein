#ifndef _EIN_JACO_CONFIG_H_
#define _EIN_JACO_CONFIG_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <kinova_msgs/KinovaPose.h>
#include <kinova_msgs/ArmPoseAction.h>
#include "ein_words.h"
#include "config.h"
#include "camera.h"


class EinJacoConfig {
 public:
  MachineState * ms;
  EinJacoConfig(MachineState * ms);

  ros::Subscriber kinova_pose_sub;
  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> kinova_arm_pose_action;

  ros::NodeHandle n;
  void endpointCallback(const kinova_msgs::KinovaPose& _p);

};

#endif /* _EIN_JACO_CONFIG_H_ */
