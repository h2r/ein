#ifndef _EIN_JACO_H_
#define _EIN_JACO_H_

class EinJacoConfig;
class MachineState;
void robotInitializeConfig(MachineState * ms);
void robotInitializeMachine(MachineState * ms);
void robotEndPointCallback(MachineState * ms);
void robotSetCurrentJointPositions(MachineState * ms);

void robotActivateSensorStreaming(MachineState * ms);
void robotDeactivateSensorStreaming(MachineState * ms);
void robotUpdate(MachineState * ms);

#include <vector>
#include <memory>
#endif /* _EIN_JACO_H_ */
