#ifndef _EIN_AIBO_CONFIG_H_
#define _EIN_AIBO_CONFIG_H_

class EinAiboConfig {
  int focusedMember = 0;
  std::vector<EinAiboConfig*> pack;

  ros::Time aiboStoppedTime;
  EinAiboJoints * stoppedJoints;
  ros::Time aiboComeToStopTime;
}

#endif /* _EIN_AIBO_CONFIG_H_ */
