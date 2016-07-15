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


void MachineState::update_ardrone(ros::NodeHandle &n) {
  MachineState * ms = this;
  geometry_msgs::Pose pose;
  eePoseToRosPose(ms->config.currentEEPose, &pose);
  ms->arDroneState.posePublisher.publish(pose);
}

namespace ein_words {

WORD(ArDroneFrontCamera)
virtual string description() {
  return "Enable and subscribe to the front camera.";
}
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneFrontCamera)

WORD(ArDroneBottomCamera)
virtual string description() {
  return "Enable and subscribe to the bottom camera.";
}
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "implement me.");
}
END_WORD
REGISTER_WORD(ArDroneBottomCamera)


WORD(ArDroneTakeoff)
virtual string description() {
  return "Send the takeoff message to the AR Drone.";
}
virtual void execute(MachineState * ms) {
  std_msgs::Empty myMsg;
  ms->arDroneState.takeoffPublisher.publish(myMsg);
}
END_WORD
REGISTER_WORD(ArDroneTakeoff)

WORD(ArDroneLand)
virtual string description() {
  return "Send the land message to the AR Drone.";
}
virtual void execute(MachineState * ms) {
  std_msgs::Empty myMsg;
  ms->arDroneState.landPublisher.publish(myMsg);
}
END_WORD
REGISTER_WORD(ArDroneLand)

WORD(ArDroneReset)

virtual string description() {
  return "Send the reset message to the AR Drone.";
}
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


WORD(ArDroneUpdateSerial)
  virtual void execute(MachineState * ms) {
  string serial = exec("bash -c \"echo cat /factory/serial.txt | nc 192.168.1.1 23 -q 1 | tail -2 | head -1\"");
  ms->config.robot_serial = serial;
}
END_WORD
REGISTER_WORD(ArDroneUpdateSerial)
}
