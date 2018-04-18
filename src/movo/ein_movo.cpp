#include "ein_movo.h"
#include "ein_movo_config.h"

#include <actionlib/client/simple_action_client.h>

#include "config.h"
#include "ein.h"

#define MC ms->config.movoConfig


using namespace std;

EinMovoConfig::EinMovoConfig(MachineState * myms): n("~")
 {
   ms = myms;
   torsoJointSubscriber = n.subscribe("/movo/linear_actuator/joint_states", 1, &EinMovoConfig::torsoJointCallback, this);
   torsoJointCmdPub = n.advertise<movo_msgs::LinearActuatorCmd>("/movo/linear_actuator_cmd", 10);

   panTiltFdbkSubscriber = n.subscribe("/movo/head/data", 1, &EinMovoConfig::panTiltFdbkCallback, this);
   panTiltCmdPub = n.advertise<movo_msgs::PanTiltCmd>("/movo/head/cmd", 10);
}

void EinMovoConfig::panTiltFdbkCallback(const movo_msgs::PanTiltFdbk& m)
{
  MC->ptaFdbkMsg = m;
}

void EinMovoConfig::torsoJointCallback(const sensor_msgs::JointState& js)
{
  assert(js.position.size() == 1);
  MC->trueTorsoJointPosition = js.position[0];
  MC->trueTorsoJointVelocity = js.velocity[0];
}


void robotActivateSensorStreaming(MachineState * ms) {
}
void robotDeactivateSensorStreaming(MachineState * ms) {
}

void robotUpdate(MachineState * ms) {
  MC->torsoCmd.header.stamp = ros::Time::now();
  MC->torsoCmd.desired_position_m = MC->targetTorsoJointPosition;
  MC->torsoCmd.fdfwd_vel_mps = 0;
  MC->torsoJointCmdPub.publish(MC->torsoCmd);


  MC->ptaCmdMsg.header.stamp = ros::Time::now();
  MC->ptaCmdMsg.pan_cmd.pos_rad = MC->targetPanPos;
  MC->ptaCmdMsg.pan_cmd.vel_rps = 10000.0;
  MC->ptaCmdMsg.pan_cmd.acc_rps2 = 0.0;

  MC->ptaCmdMsg.tilt_cmd.pos_rad = MC->targetTiltPos;
  MC->ptaCmdMsg.tilt_cmd.vel_rps = 10000;
  MC->ptaCmdMsg.tilt_cmd.acc_rps2 = 0.0;
  MC->panTiltCmdPub.publish(MC->ptaCmdMsg);



  geometry_msgs::PoseStamped id;
  id.header.frame_id = "base_link";
  id.header.stamp = ros::Time(0);
  id.pose.position.x = 0;
  id.pose.position.y = 0;
  id.pose.position.z = 0;
  id.pose.orientation.x = 0;
  id.pose.orientation.y = 0;
  id.pose.orientation.z = 0;
  id.pose.orientation.w = 1;
  
  geometry_msgs::PoseStamped odom_pose;

  ms->config.tfListener->transformPose("odom", id, odom_pose);
  MC->odomPose = rosPoseToEEPose(odom_pose.pose);
}

WORD(OdomPose)
virtual string description() {
  return "The odom pose, transformed from base.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(MC->odomPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(OdomPose)


void robotInitializeConfig(MachineState * ms) {
 MC = new EinMovoConfig(ms);
 ms->config.cameras.clear();

 string image_topic = "/cameras/stub/image";
 Camera * c = new Camera(ms, "stub", image_topic, "stub", "stub");
 ms->config.cameras.push_back(c);
 ms->config.focused_camera = 0;

}


void robotInitializeMachine(MachineState * ms) {
  ms->evaluateProgram("zeroGOff"); 

}

void robotSetCurrentJointPositions(MachineState * ms) {
  MC->targetTorsoJointPosition = MC->trueTorsoJointPosition;
  MC->targetPanPos = MC->ptaFdbkMsg.pan.pos_rad;
  MC->targetTiltPos = MC->ptaFdbkMsg.tilt.pos_rad;
}

void robotEndPointCallback(MachineState * ms) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "";
  p.header.stamp = ros::Time::now();
  p.pose = eePoseToRosPose(ms->config.currentEEPose);
}


namespace ein_words {


WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  // stub
}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)

WORD(TorsoUp)
virtual string description() {
  return "Move the torso up.";
}
virtual void execute(MachineState * ms) {
  MC->targetTorsoJointPosition += MC->torsoGridSize;
}
END_WORD
REGISTER_WORD(TorsoUp)

WORD(TorsoDown)
virtual string description() {
  return "Move the torso down.";
}
virtual void execute(MachineState * ms) {
  MC->targetTorsoJointPosition -= MC->torsoGridSize;
}
END_WORD
REGISTER_WORD(TorsoDown)


WORD(PanDown)
virtual string description() {
  return "Move the pan angle down.";
}
virtual void execute(MachineState * ms) {
  MC->targetPanPos -= MC->panTiltGridSize;
}
END_WORD
REGISTER_WORD(PanDown)

WORD(PanUp)
virtual string description() {
  return "Move the pan angle up.";
}
virtual void execute(MachineState * ms) {
  MC->targetPanPos += MC->panTiltGridSize;
}
END_WORD
REGISTER_WORD(PanUp)


CONFIG_GETTER_DOUBLE(TrueTorsoJointPosition, MC->trueTorsoJointPosition, "The true torso position from the topic.")
CONFIG_GETTER_DOUBLE(TrueTorsoJointVelocity, MC->trueTorsoJointVelocity, "The true torso velocity from the topic.")

CONFIG_GETTER_DOUBLE(TargetTorsoJointPosition, MC->targetTorsoJointPosition, "The target torso position from the topic.")
CONFIG_SETTER_DOUBLE(SetTargetTorsoJointPosition, MC->targetTorsoJointPosition)


CONFIG_GETTER_DOUBLE(TorsoGridSize, MC->torsoGridSize, "The grid size when moving the torso up and down.  Default 1cm.")
CONFIG_SETTER_DOUBLE(SetTorsoGridSize, MC->torsoGridSize)


CONFIG_GETTER_DOUBLE(TargetPanPos, MC->targetPanPos, "The target pan.")
CONFIG_SETTER_DOUBLE(SetTargetPanPos, MC->targetPanPos)
CONFIG_GETTER_DOUBLE(PanCurrent, MC->ptaFdbkMsg.pan.current, "The current pan.")
CONFIG_GETTER_DOUBLE(PanPos, MC->ptaFdbkMsg.pan.pos_rad, "The current position in radians.")
CONFIG_GETTER_DOUBLE(PanVel, MC->ptaFdbkMsg.pan.vel_rps, "The current velocity in rps.")
CONFIG_GETTER_DOUBLE(PanTorque, MC->ptaFdbkMsg.pan.torque_nm, "The current torque in Newton-meters.")
CONFIG_GETTER_DOUBLE(PanPwm, MC->ptaFdbkMsg.pan.pwm, "The PWM value.")
CONFIG_GETTER_DOUBLE(PanEncoder, MC->ptaFdbkMsg.pan.encoder_rad, "The joint encoder value.")
CONFIG_GETTER_DOUBLE(PanAx, MC->ptaFdbkMsg.pan.accel.x, "The x accelleration.")
CONFIG_GETTER_DOUBLE(PanAy, MC->ptaFdbkMsg.pan.accel.x, "The y accelleration.")
CONFIG_GETTER_DOUBLE(PanAz, MC->ptaFdbkMsg.pan.accel.x, "The z accelleration.")
CONFIG_GETTER_DOUBLE(PanTemperature, MC->ptaFdbkMsg.pan.temperature_degC, "The temperature in Celcius.")


CONFIG_GETTER_DOUBLE(TargetTiltPos, MC->targetTiltPos, "The target tilt.")
CONFIG_SETTER_DOUBLE(SetTargetTilt, MC->targetTiltPos)
CONFIG_GETTER_DOUBLE(TiltCurrent, MC->ptaFdbkMsg.tilt.current, "The current tilt.")
CONFIG_GETTER_DOUBLE(TiltPos, MC->ptaFdbkMsg.tilt.pos_rad, "The current position in radians.")
CONFIG_GETTER_DOUBLE(TiltVel, MC->ptaFdbkMsg.tilt.vel_rps, "The current velocity in rps.")
CONFIG_GETTER_DOUBLE(TiltTorque, MC->ptaFdbkMsg.tilt.torque_nm, "The current torque in Newton-meters.")
CONFIG_GETTER_DOUBLE(TiltPwm, MC->ptaFdbkMsg.tilt.pwm, "The PWM value.")
CONFIG_GETTER_DOUBLE(TiltEncoder, MC->ptaFdbkMsg.tilt.encoder_rad, "The joint encoder value.")
CONFIG_GETTER_DOUBLE(TiltAx, MC->ptaFdbkMsg.tilt.accel.x, "The x accelleration.")
CONFIG_GETTER_DOUBLE(TiltAy, MC->ptaFdbkMsg.tilt.accel.x, "The y accelleration.")
CONFIG_GETTER_DOUBLE(TiltAz, MC->ptaFdbkMsg.tilt.accel.x, "The z accelleration.")
CONFIG_GETTER_DOUBLE(TiltTemperature, MC->ptaFdbkMsg.tilt.temperature_degC, "The temperature in Celcius.")



}
