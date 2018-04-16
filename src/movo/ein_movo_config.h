#ifndef _EIN_MOVO_CONFIG_H_
#define _EIN_MOVO_CONFIG_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

#include <movo_msgs/LinearActuatorCmd.h>

#include "ein_words.h"
#include "config.h"
#include "camera.h"



class EinMovoConfig {
 public:
  MachineState * ms;
  EinMovoConfig(MachineState * ms);
  ros::NodeHandle n;
  ros::Subscriber torsoJointSubscriber;
  ros::Publisher torsoJointCmdPub;
  movo_msgs::LinearActuatorCmd torsoCmd;

  double torsoGridSize=0.01;
  double targetTorsoJointPosition=0;
  double trueTorsoJointPosition=0;
  double trueTorsoJointVelocity=0;
  void torsoJointCallback(const sensor_msgs::JointState& js);
};

#endif /* _EIN_MOVO_CONFIG_H_ */
