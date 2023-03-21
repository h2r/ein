#ifndef _EIN_SPOT_H_
#define _EIN_SPOT_H_

class EinSpotConfig;
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
#endif /* _EIN_SPOT_H_ */
