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
  ros::NodeHandle n;

};

#endif /* _EIN_MOVO_CONFIG_H_ */
