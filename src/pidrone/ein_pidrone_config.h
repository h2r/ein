#ifndef _EIN_PIDRONE_CONFIG_H_
#define _EIN_PIDRONE_CONFIG_H_

#include <ros/ros.h>

class EinPidroneConfig {

 public:
  MachineState * ms;

  ros::Subscriber eeRanger;

  EinPidroneConfig(MachineState * ms);

  void endPointCallback(const ros::TimerEvent&);
  ros::Timer eeTimer;

  ros::Publisher modePub;
  ros::Publisher resetTransformPub;
  ros::Publisher toggleTransformPub;


  ros::NodeHandle n;
};

#endif /* _EIN_PIDRONE_CONFIG_H_ */
