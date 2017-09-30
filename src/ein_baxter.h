#ifndef _EIN_BAXTER_H_
#define _EIN_BAXTER_H_

class EinBaxterConfig;
class MachineState;
void baxterInitializeConfig(MachineState * ms);
void baxterEndPointCallback(MachineState * ms);
void baxterSetCurrentJointPositions(MachineState * ms);

void baxterActivateSensorStreaming(MachineState * ms);
void baxterDeactivateSensorStreaming(MachineState * ms);
void baxterUpdate(MachineState * ms);


#endif /* _EIN_BAXTER_H_ */
