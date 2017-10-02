#ifndef _EIN_AIBO_H_
#define _EIN_AIBO_H_

class EinAiboSensors;
class EinAiboJoints;
class EinAiboIndicators;
class AiboPoseWord;
class EinAiboConfig;
class MachineState;

void robotInitializeConfig(MachineState * ms);
void robotEndPointCallback(MachineState * ms);
void robotSetCurrentJointPositions(MachineState * ms);

void robotActivateSensorStreaming(MachineState * ms);
void robotDeactivateSensorStreaming(MachineState * ms);
void robotUpdate(MachineState * ms);

void robotHappy(MachineState * ms);
void robotSad(MachineState * ms);
void robotNeutral(MachineState * ms);



#endif /* _EIN_AIBO_H_*/
