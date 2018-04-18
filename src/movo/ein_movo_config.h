#ifndef _EIN_MOVO_CONFIG_H_
#define _EIN_MOVO_CONFIG_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

#include <movo_msgs/LinearActuatorCmd.h>
#include <movo_msgs/PanTiltActuatorFdbk.h>
#include <movo_msgs/PVA.h>
#include <movo_msgs/ConfigCmd.h>
#include <movo_msgs/PanTiltFdbk.h>
#include <movo_msgs/PanTiltCmd.h>
#include <movo_msgs/Battery.h>

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
  
  

  ros::Subscriber torsoJointSubscriber;
  ros::Publisher torsoJointCmdPub;
  movo_msgs::LinearActuatorCmd torsoCmd;
  void torsoJointCallback(const sensor_msgs::JointState& js);

  ros::Subscriber panTiltFdbkSubscriber;
  ros::Publisher panTiltCmdPub;
  movo_msgs::PanTiltFdbk ptaFdbkMsg;
  movo_msgs::PanTiltCmd ptaCmdMsg;
  void panTiltFdbkCallback(const movo_msgs::PanTiltFdbk& js);  
  double panTiltGridSize=0.01;

  double torsoGridSize=0.01;
  double targetTorsoJointPosition=0;
  double trueTorsoJointPosition=0;
  double trueTorsoJointVelocity=0;

  double targetPanPos;
  double targetTiltPos;

  eePose odomPose;
  eePose mapPose;

};

#endif /* _EIN_MOVO_CONFIG_H_ */
