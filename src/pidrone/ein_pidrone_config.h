#ifndef _EIN_PIDRONE_CONFIG_H_
#define _EIN_PIDRONE_CONFIG_H_

#include <ros/ros.h>

class EinPidroneConfig {

 public:
  MachineState * ms;

  EinPidroneConfig(MachineState * ms);

  ros::NodeHandle n;
};

#endif /* _EIN_PIDRONE_CONFIG_H_ */
