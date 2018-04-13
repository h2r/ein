#ifndef _EIN_MOVO_H_
#define _EIN_MOVO_H_

class EinMovoConfig;
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
#endif /* _EIN_MOVO_H_ */
