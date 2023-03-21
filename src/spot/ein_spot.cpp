#include "ein_spot.h"
#include "config.h"
#include "camera.h"



void robotInitializeSerial(MachineState * ms) {
}
void robotInitializeConfig(MachineState * ms) {
  
  Camera * c = new Camera(ms, "hand_camera", "spottopic", "hand_camera", "hand_camera");
  ms->config.cameras.push_back(c);
  ms->config.focused_camera = 0;
  
}
void robotInitializeMachine(MachineState * ms) {
}

void robotSetCurrentJointPositions(MachineState * ms) {
}

void robotEndPointCallback(MachineState * ms) {
}

void robotActivateSensorStreaming(MachineState * ms){}
void robotDeactivateSensorStreaming(MachineState * ms){}
void robotUpdate(MachineState * ms){}
