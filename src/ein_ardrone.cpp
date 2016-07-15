#include "ein_words.h"
#include "ein.h"

#include "qtgui/einwindow.h"

void MachineState::ardroneTruePoseCallback(const geometry_msgs::PoseStamped p) {
  cout << "Received pose. " << endl;
}

namespace ein_words {

WORD(ArDroneFrontCamera)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneFrontCamera)

WORD(ArDroneBottomCamera)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneBottomCamera)


WORD(ArDroneTakeoff)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneTakeoff)

WORD(ArDroneLand)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneLand)

WORD(ArDroneReset)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneReset)

WORD(ArDroneHover)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneHover)


}
