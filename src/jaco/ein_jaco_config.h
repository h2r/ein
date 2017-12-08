#ifndef _EIN_JACO_CONFIG_H_
#define _EIN_JACO_CONFIG_H_

#include <ros/ros.h>

#include <kinova_msgs/KinovaPose.h>

#include "ein_words.h"
#include "config.h"
#include "camera.h"


class EinJacoConfig {
 public:
  MachineState * ms;
  EinJacoConfig(MachineState * ms);

  ros::Subscriber kinova_pose_sub;
  ros::NodeHandle n;
  void endpointCallback(const kinova_msgs::KinovaPose& _p);

};

#endif /* _EIN_JACO_CONFIG_H_ */
