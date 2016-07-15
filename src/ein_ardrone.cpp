#include "ein_words.h"
#include "ein.h"

#include "qtgui/einwindow.h"

void MachineState::ardroneTruePoseCallback(const geometry_msgs::PoseStamped p) {
  MachineState * ms = this;

  ms->config.trueEEPose = p.pose;
  {
    ms->config.trueEEPoseEEPose.px = p.pose.position.x;
    ms->config.trueEEPoseEEPose.py = p.pose.position.y;
    ms->config.trueEEPoseEEPose.pz = p.pose.position.z;
    ms->config.trueEEPoseEEPose.qx = p.pose.orientation.x;
    ms->config.trueEEPoseEEPose.qy = p.pose.orientation.y;
    ms->config.trueEEPoseEEPose.qz = p.pose.orientation.z;
    ms->config.trueEEPoseEEPose.qw = p.pose.orientation.w;
  }

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
  std_msgs::Empty myMsg;
  ms->arDroneState.takeoffPublisher.publish(myMsg);
}
END_WORD
REGISTER_WORD(ArDroneTakeoff)

WORD(ArDroneLand)
virtual void execute(MachineState * ms) {
  std_msgs::Empty myMsg;
  ms->arDroneState.landPublisher.publish(myMsg);
}
END_WORD
REGISTER_WORD(ArDroneLand)

WORD(ArDroneReset)
virtual void execute(MachineState * ms) {
  std_msgs::Empty myMsg;
  ms->arDroneState.resetPublisher.publish(myMsg);
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
