#ifndef _EIN_KUKA_H_
#define _EIN_KUKA_H_

class EinKukaConfig;
class MachineState;
void robotInitializeSerial(MachineState * ms);
void robotInitializeConfig(MachineState * ms);
void robotInitializeMachine(MachineState * ms);
void robotEndPointCallback(MachineState * ms);
void robotSetCurrentJointPositions(MachineState * ms);

void robotActivateSensorStreaming(MachineState * ms);
void robotDeactivateSensorStreaming(MachineState * ms);
void robotUpdate(MachineState * ms);

#include <vector>
#include <memory>
#endif /* _EIN_KUKA_H_ */
