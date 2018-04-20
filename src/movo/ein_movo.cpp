#include "ein_movo.h"
#include "ein_movo_config.h"

#include <moveit_msgs/MoveItErrorCodes.h>

#include "config.h"
#include "ein.h"

#define MC ms->config.movoConfig

#define TRACTOR_REQUEST 5
#define STANDBY_REQUEST 4
#define POWERDOWN_REQUEST 6
#define ESTOP_REQUEST 1

using namespace std;

EinMovoConfig::EinMovoConfig(MachineState * myms): n("~")
 {
   ms = myms;


   upperBody = new MoveGroup("upper_body");
   leftArm = new MoveGroup("left_arm");
   rightArm = new MoveGroup("right_arm");
   upperBody->setPlannerId("RRTConnectkConfigDefault");
   leftArm->setPlannerId("RRTConnectkConfigDefault");
   rightArm->setPlannerId("RRTConnectkConfigDefault");

   moveitStatusSubscriber = n.subscribe("/move_group/status", 1, &EinMovoConfig::moveitStatusCallback, this);

   batterySubscriber = n.subscribe("/movo/feedback/battery", 1, &EinMovoConfig::batteryCallback, this);
   
   batterySubscriber = n.subscribe("/movo/feedback/battery", 1, &EinMovoConfig::batteryCallback, this);
   

   torsoJointSubscriber = n.subscribe("/movo/linear_actuator/joint_states", 1, &EinMovoConfig::torsoJointCallback, this);
   torsoJointCmdPub = n.advertise<movo_msgs::LinearActuatorCmd>("/movo/linear_actuator_cmd", 10);

   configCmdPub = n.advertise<movo_msgs::ConfigCmd>("/movo/gp_command", 10);

   panTiltFdbkSubscriber = n.subscribe("/movo/head/data", 1, &EinMovoConfig::panTiltFdbkCallback, this);
   panTiltCmdPub = n.advertise<movo_msgs::PanTiltCmd>("/movo/head/cmd", 10);

   cmdVelPub = n.advertise<geometry_msgs::Twist>("/movo/cmd_vel", 10);
}

void EinMovoConfig::moveitStatusCallback(const actionlib_msgs::GoalStatusArray & m)
{
  for (int i = 0; i < m.status_list.size(); i++) {
    MC->goals[m.status_list[i].goal_id.id] = m.status_list[i];
  }
}


void EinMovoConfig::panTiltFdbkCallback(const movo_msgs::PanTiltFdbk& m)
{
  MC->ptaFdbkMsg = m;
}

void EinMovoConfig::batteryCallback(const movo_msgs::Battery& b) 
{
  MC->batteryMsg = b;
  if (0x1000 == (b.battery_status & 0x1000)) {
    MC->batteryCharging = true;
  } else {
    MC->batteryCharging = false;
  }
}

void EinMovoConfig::torsoJointCallback(const sensor_msgs::JointState& js)
{
  assert(js.position.size() == 1);
  MC->trueTorsoJointPosition = js.position[0];
  MC->trueTorsoJointVelocity = js.velocity[0];

  geometry_msgs::PoseStamped id;
  id.header.stamp = ros::Time(0);
  id.pose.position.x = 0;
  id.pose.position.y = 0;
  id.pose.position.z = 0;
  id.pose.orientation.x = 0;
  id.pose.orientation.y = 0;
  id.pose.orientation.z = 0;
  id.pose.orientation.w = 1;
  
  geometry_msgs::PoseStamped odom_pose;
  id.header.frame_id = "base_link";
  ms->config.tfListener->transformPose("odom", id, odom_pose);
  MC->odomPose = rosPoseToEEPose(odom_pose.pose);

  geometry_msgs::PoseStamped left_ee_pose;
  id.header.frame_id = "left_gripper_base_link";
  ms->config.tfListener->transformPose("base_link", id, left_ee_pose);
  MC->leftPose = rosPoseToEEPose(left_ee_pose.pose);

  geometry_msgs::PoseStamped right_ee_pose;
  id.header.frame_id = "right_gripper_base_link";
  ms->config.tfListener->transformPose("base_link", id, right_ee_pose);
  MC->rightPose = rosPoseToEEPose(right_ee_pose.pose);
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
  MC->ptaCmdMsg.pan_cmd.vel_rps = 0.87;
  MC->ptaCmdMsg.pan_cmd.acc_rps2 = 0.0;

  MC->ptaCmdMsg.tilt_cmd.pos_rad = MC->targetTiltPos;
  MC->ptaCmdMsg.tilt_cmd.vel_rps = 0.87; 
  MC->ptaCmdMsg.tilt_cmd.acc_rps2 = 0.0;
  MC->panTiltCmdPub.publish(MC->ptaCmdMsg);


  

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

WORD(LXup)
virtual string description() {
  return "Move the left arm up in x.";
}
virtual void execute(MachineState * ms) {
  MC->leftTargetPose.px += MC->gridSize;
}
END_WORD
REGISTER_WORD(LXup)


WORD(LeftPose)
virtual string description() {
  return "The left arm end effector pose.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(MC->leftPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(LeftPose)


WORD(RightPose)
virtual string description() {
  return "The right arm end effector pose.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(MC->rightPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(RightPose)


WORD(RightTargetPose)
virtual string description() {
  return "The right arm end effector target pose.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(MC->rightTargetPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(RightTargetPose)


WORD(LeftTargetPose)
virtual string description() {
  return "The left arm end effector target pose.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(MC->leftTargetPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(LeftTargetPose)




void robotInitializeConfig(MachineState * ms) {
 MC = new EinMovoConfig(ms);
 ms->config.cameras.clear();

 string image_topic = "/cameras/stub/image";
 Camera * c = new Camera(ms, "stub", image_topic, "stub", "stub");
 ms->config.cameras.push_back(c);
 ms->config.focused_camera = 0;

}


void robotInitializeMachine(MachineState * ms) {
  ms->evaluateProgram("\"movo\" import"); 

}

void robotSetCurrentJointPositions(MachineState * ms) {
  MC->targetTorsoJointPosition = MC->trueTorsoJointPosition;
  MC->targetPanPos = MC->ptaFdbkMsg.pan.pos_rad;
  MC->targetTiltPos = MC->ptaFdbkMsg.tilt.pos_rad;

  MC->leftTargetPose = MC->leftPose;
  MC->rightTargetPose = MC->rightPose;
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

WORD(TiltUp)
virtual string description() {
  return "Move the tilt angle up.";
}
virtual void execute(MachineState * ms) {
  MC->targetTiltPos += MC->panTiltGridSize;
}
END_WORD
REGISTER_WORD(TiltUp)

WORD(TiltDown)
virtual string description() {
  return "Move the tilt angle down.";
}
virtual void execute(MachineState * ms) {
  MC->targetTiltPos -= MC->panTiltGridSize;
}
END_WORD
REGISTER_WORD(TiltDown)



WORD(BaseGoCfg)
virtual string description() {
  return "Configure the base to start moving.";
}
virtual void execute(MachineState * ms) {
  MC->configMsg.gp_cmd = "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE";
  MC->configMsg.gp_param = TRACTOR_REQUEST;
  MC->configMsg.header.stamp = ros::Time::now();
  MC->configCmdPub.publish(MC->configMsg);
}
END_WORD
REGISTER_WORD(BaseGoCfg)


WORD(BaseStopCfg)
virtual string description() {
  return "Configure the base to stop moving.";
}
virtual void execute(MachineState * ms) {
  MC->configMsg.gp_cmd = "GENERAL_PURPOSE_CMD_NONE";
  MC->configMsg.gp_param = 0;
  MC->configMsg.header.stamp = ros::Time::now();
  MC->configCmdPub.publish(MC->configMsg);
}
END_WORD
REGISTER_WORD(BaseStopCfg)


WORD(BaseStopTwist)
virtual string description() {
  return "Send a zero twist command.";
}
virtual void execute(MachineState * ms) {
  MC->twistMsg.linear.x = 0;
  MC->twistMsg.linear.y = 0;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = 0;
  MC->cmdVelPub.publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseStopTwist)




WORD(BaseSendOZVel)
virtual string description() {
  return "Rotate the base at a velocity.";
}
virtual void execute(MachineState * ms) {
  double angularspeed;
  GET_NUMERIC_ARG(ms, angularspeed);

  MC->twistMsg.linear.x = 0;
  MC->twistMsg.linear.y = 0;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = angularspeed;
  MC->cmdVelPub.publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendOZVel)

WORD(BaseSendXVel)
virtual string description() {
  return "Move the base in X at a velocity.";
}
virtual void execute(MachineState * ms) {
  double speed;
  GET_NUMERIC_ARG(ms, speed);

  MC->twistMsg.linear.x = speed;
  MC->twistMsg.linear.y = 0;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = 0;
  MC->cmdVelPub.publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendXVel)


WORD(BaseSendYVel)
virtual string description() {
  return "Move the base in Y at a velocity.";
}
virtual void execute(MachineState * ms) {
  double speed;
  GET_NUMERIC_ARG(ms, speed);

  MC->twistMsg.linear.x = 0;
  MC->twistMsg.linear.y = speed;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = 0;
  MC->cmdVelPub.publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendYVel)


WORD(MovoKill)
virtual string description() {
  return "Send a kill command to estop the robot.";
}
virtual void execute(MachineState * ms) {
  MC->configMsg.gp_cmd = "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE";
  MC->configMsg.gp_param = ESTOP_REQUEST;
  MC->configMsg.header.stamp = ros::Time::now();
  MC->configCmdPub.publish(MC->configMsg);

}
END_WORD
REGISTER_WORD(MovoKill)




WORD(BaseSendTwist)
virtual string description() {
  return "Send a twist command.  linearX linearY linearZ angularX angularY angularZ baseSendTwist";
}
virtual void execute(MachineState * ms) {
  double lx, ly, lz;
  double ax, ay, az;
  GET_NUMERIC_ARG(ms, lx);
  GET_NUMERIC_ARG(ms, ly);
  GET_NUMERIC_ARG(ms, lz);
  GET_NUMERIC_ARG(ms, ax);
  GET_NUMERIC_ARG(ms, ay);
  GET_NUMERIC_ARG(ms, az);

  MC->twistMsg.linear.x = lx;
  MC->twistMsg.linear.y = ly;
  MC->twistMsg.linear.z = lz;
  MC->twistMsg.angular.x = ax;
  MC->twistMsg.angular.y = ay;
  MC->twistMsg.angular.z = az;
  MC->cmdVelPub.publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendTwist)


WORD(MoveToHome)
virtual string description() {
  return "Move to the home position.";
}
virtual void execute(MachineState * ms) {
  for (int i = 0; i < MC->homedJoints.size(); i++) {
    bool result = MC->upperBody->setJointValueTarget(MC->upperBodyJoints[i], MC->homedJoints[i]);
    if (!result) {
      CONSOLE_ERROR(ms, "Invalid joint target: " << MC->upperBodyJoints[i] << " value: " << MC->homedJoints[i]);
      return;
    }
  }
  MoveItErrorCode r = MC->upperBody->asyncMove();
  if (r.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    CONSOLE_ERROR(ms, "Couldn't execute.  Code:  " << r.val);
  }
}
END_WORD
REGISTER_WORD(MoveToHome)


WORD(MoveToTuck)
virtual string description() {
  return "Move to the tuck position.";
}
virtual void execute(MachineState * ms) {
  for (int i = 0; i < MC->tuckedJoints.size(); i++) {
    bool result = MC->upperBody->setJointValueTarget(MC->upperBodyJoints[i], MC->tuckedJoints[i]);
    if (!result) {
      CONSOLE_ERROR(ms, "Invalid joint target: " << MC->upperBodyJoints[i] << " value: " << MC->homedJoints[i]);
      return;
    }
  }
  MoveItErrorCode r = MC->upperBody->asyncMove();
  if (r.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    CONSOLE_ERROR(ms, "Couldn't execute.  Code:  " << r.val);
  }

}
END_WORD
REGISTER_WORD(MoveToTuck)



CONFIG_GETTER_DOUBLE(TrueTorsoJointPosition, MC->trueTorsoJointPosition, "The true torso position from the topic.")
CONFIG_GETTER_DOUBLE(TrueTorsoJointVelocity, MC->trueTorsoJointVelocity, "The true torso velocity from the topic.")

CONFIG_GETTER_DOUBLE(TargetTorsoJointPosition, MC->targetTorsoJointPosition, "The target torso position from the topic.")
CONFIG_SETTER_DOUBLE(SetTargetTorsoJointPosition, MC->targetTorsoJointPosition)


CONFIG_GETTER_DOUBLE(TorsoGridSize, MC->torsoGridSize, "The grid size when moving the torso up and down.  Default 1cm.")
CONFIG_SETTER_DOUBLE(SetTorsoGridSize, MC->torsoGridSize)

CONFIG_GETTER_DOUBLE(PanTiltGridSize, MC->panTiltGridSize, "The grid size when moving the panTilt up and down.  Default 0.1 radians.")
CONFIG_SETTER_DOUBLE(SetPanTiltGridSize, MC->panTiltGridSize)


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

CONFIG_GETTER_INT(BatteryCharging, MC->batteryCharging)
CONFIG_GETTER_DOUBLE(BatteryVoltage, MC->batteryMsg.battery_voltage_VDC, "Is the battery charging?")
CONFIG_GETTER_DOUBLE(BatterySoc, MC->batteryMsg.battery_soc, "Battery state of charge.")

CONFIG_GETTER_DOUBLE(MoveitUpperPlanningTime, MC->upperBody->getPlanningTime(), "Moveit planning time.")

CONFIG_GETTER_STRING(MoveitUpperPlanningFrame, MC->upperBody->getPlanningFrame(), "Moveit planning frame.")

CONFIG_GETTER_DOUBLE(MoveitUpperGoalJointTolerance, MC->upperBody->getGoalJointTolerance(), "Moveit goal joint tolerance.")
WORD(MoveitUpperSetGoalJointTolerance)
virtual string description() {
  return "Set moveit upper goal joint tolerance.";
}
virtual void execute(MachineState * ms) {
  double t;
  GET_NUMERIC_ARG(ms, t);

  MC->upperBody->setGoalJointTolerance(t);
}
END_WORD
REGISTER_WORD(MoveitUpperSetGoalJointTolerance)



}
