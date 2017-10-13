#include "ein_pidrone.h"
#include "ein_pidrone_config.h"
#include "ein_words.h"
#include "config.h"
#include "camera.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <highgui.h>

void robotActivateSensorStreaming(MachineState * ms) {
}
void robotDeactivateSensorStreaming(MachineState * ms) {
}

void robotUpdate(MachineState * ms) {
}

void robotInitializeConfig(MachineState * ms) {
  ms->config.pidroneConfig = new EinPidroneConfig(ms);
}

void robotInitializeMachine(MachineState * ms) {

}

void robotSetCurrentJointPositions(MachineState * ms) {

}

void robotEndPointCallback(MachineState * ms) {


}




EinPidroneConfig::EinPidroneConfig(MachineState * myms): n("~") {
  ms = myms;
}

namespace ein_words {

WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "Ignore camera config for Pidrone!");

}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)

}
