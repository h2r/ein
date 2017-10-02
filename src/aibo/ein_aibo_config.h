#ifndef _EIN_AIBO_CONFIG_H_
#define _EIN_AIBO_CONFIG_H_

class EinAiboDog;
class EinAiboConfig {
 public:

  MachineState * ms;

  EinAiboConfig(MachineState * ms);

  int focusedMember = 0;
  std::vector<EinAiboDog*> pack;

  ros::Time aiboStoppedTime;
  EinAiboJoints * stoppedJoints;
  ros::Time aiboComeToStopTime;
};

#endif /* _EIN_AIBO_CONFIG_H_ */
