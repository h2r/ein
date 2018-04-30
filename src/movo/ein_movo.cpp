#include "ein_movo.h"
#include "ein_movo_config.h"

#include <moveit_msgs/MoveItErrorCodes.h>

#include "config.h"
#include "ein.h"

#define MC ms->config.movoConfig
#define CMG MC->endEffectors[MC->focused_ee]
#define EE_LEFT_ARM 0
#define EE_RIGHT_ARM 1

#define TRACTOR_REQUEST 5
#define STANDBY_REQUEST 4
#define POWERDOWN_REQUEST 6
#define ESTOP_REQUEST 1

using namespace std;

EinMovoConfig::EinMovoConfig(MachineState * myms): n("~"), 
						   leftGripperActionClient("/movo/left_gripper_controller/gripper_cmd", true),
						   rightGripperActionClient("/movo/right_gripper_controller/gripper_cmd", true)
 {
   ms = myms;


   upperBody = new MoveGroup("upper_body");
   leftArm = new MoveGroup("left_arm");
   rightArm = new MoveGroup("right_arm");
   upperBody->setPlannerId("RRTConnectkConfigDefault");
   leftArm->setPlannerId("RRTConnectkConfigDefault");
   rightArm->setPlannerId("RRTConnectkConfigDefault");
   endEffectors.push_back(leftArm);
   endEffectors.push_back(rightArm);
   //leftArm->setEndEffectorLink("left_gripper_base_link");
   //rightArm->setEndEffectorLink("right_gripper_base_link");

   
   gripperActions.push_back(&leftGripperActionClient);
   gripperActions.push_back(&rightGripperActionClient);

   moveitStatusSubscriber = n.subscribe("/move_group/status", 1, &EinMovoConfig::moveitStatusCallback, this);

   batterySubscriber = n.subscribe("/movo/feedback/battery", 1, &EinMovoConfig::batteryCallback, this);
   
   batterySubscriber = n.subscribe("/movo/feedback/battery", 1, &EinMovoConfig::batteryCallback, this);
   
   rightGripperJointSubscriber = n.subscribe("/movo/right_gripper/joint_states", 1, &EinMovoConfig::rightGripperJointCallback, this);
   leftGripperJointSubscriber = n.subscribe("/movo/left_gripper/joint_states", 1, &EinMovoConfig::leftGripperJointCallback, this);

   torsoJointSubscriber = n.subscribe("/movo/linear_actuator/joint_states", 1, &EinMovoConfig::torsoJointCallback, this);
   torsoJointCmdPub = n.advertise<movo_msgs::LinearActuatorCmd>("/movo/linear_actuator_cmd", 10);

   configCmdPub = n.advertise<movo_msgs::ConfigCmd>("/movo/gp_command", 10);

   panTiltFdbkSubscriber = n.subscribe("/movo/head/data", 1, &EinMovoConfig::panTiltFdbkCallback, this);
   panTiltCmdPub = n.advertise<movo_msgs::PanTiltCmd>("/movo/head/cmd", 10);

   cmdVelPub = n.advertise<geometry_msgs::Twist>("/movo/cmd_vel", 10);

   changeFocusedEndEffector(EE_RIGHT_ARM);
}

void EinMovoConfig::changeFocusedEndEffector(int idx)
{
  focused_ee = idx;
  focusedGripperActionClient = gripperActions[idx];
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

void EinMovoConfig::rightGripperJointCallback(const sensor_msgs::JointState& js)
{
  assert(js.position.size() == 3);
  MC->rightFingerJointState = js;
  if (MC->focused_ee == EE_RIGHT_ARM) {
    MC->fingerJointState = js;
  }
}

void EinMovoConfig::leftGripperJointCallback(const sensor_msgs::JointState& js)
{
  assert(js.position.size() == 3);
  MC->leftFingerJointState = js;
  if (MC->focused_ee == EE_LEFT_ARM) {
    MC->fingerJointState = js;
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

  try {
    geometry_msgs::PoseStamped map_pose;
    id.header.frame_id = "base_link";
    ms->config.tfListener->transformPose("map", id, map_pose);
    MC->mapPose = rosPoseToEEPose(map_pose.pose);
  } catch (tf2::LookupException e) {
    if (MC->lastMapLookupPrintTime == ros::Time() || ros::Time::now()  - MC->lastMapLookupPrintTime > ros::Duration(20)) {
      CONSOLE_ERROR(ms, "Warning, no map frame yet.");
      MC->lastMapLookupPrintTime  = ros::Time::now();
    }
  }


  geometry_msgs::PoseStamped left_ee_pose;
  id.header.frame_id = "left_ee_link";
  ms->config.tfListener->transformPose("base_link", id, left_ee_pose);
  MC->leftPose = rosPoseToEEPose(left_ee_pose.pose);

  geometry_msgs::PoseStamped right_ee_pose;
  id.header.frame_id = "right_ee_link";
  ms->config.tfListener->transformPose("base_link", id, right_ee_pose);
  MC->rightPose = rosPoseToEEPose(right_ee_pose.pose);

  if (MC->focused_ee == EE_LEFT_ARM) {
    ms->config.trueEEPoseEEPose = MC->leftPose;
  } else if (MC->focused_ee == EE_RIGHT_ARM) {
    ms->config.trueEEPoseEEPose = MC->rightPose;
  } else{
    CONSOLE_ERROR(ms, "Bad focused EE: " << MC->focused_ee);
    assert(0);
  }
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

  double distance, angleDistance;
  eePose::distanceXYZAndAngle(ms->config.currentEEPose, ms->config.trueEEPoseEEPose, &distance, &angleDistance);
		      
  if ((sqrt(distance) > 0.001 || angleDistance > 0.001) && 
      (ros::Time::now() - MC->lastMoveitCallTime  > ros::Duration(1))) {
    MC->lastMoveitCallTime = ros::Time::now();
    CMG->stop();
    geometry_msgs::PoseStamped p;
    p.pose = eePoseToRosPose(ms->config.currentEEPose);
    p.header.frame_id = "base_link";
    bool result = CMG->setPoseTarget(p);
    if (!result) {
      CONSOLE_ERROR(ms, "Invalid pose target: " << p);
      return;
    }
    MoveItErrorCode r = CMG->asyncMove();
    if (r.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      CONSOLE_ERROR(ms, "Couldn't execute.  Code:  " << r.val);
    }
  }

  if (ros::Time::now() - MC->lastGripperCallTime  > ros::Duration(1)) {
  }


  /*  if (eePose::distance(MC->leftTargetPose, MC->leftPose) > 0.001 && (ros::Time::now() - MC->lastMoveitCallTime  > ros::Duration(1) )) {
    MC->lastMoveitCallTime = ros::Time::now();
    MC->leftArm->stop();
    geometry_msgs::PoseStamped p;
    p.pose = eePoseToRosPose(MC->leftTargetPose);
    p.header.frame_id = "base_link";
    bool result = MC->leftArm->setPoseTarget(p);
    if (!result) {
      CONSOLE_ERROR(ms, "Invalid pose target: " << p);
      return;
    }
    
    MoveItErrorCode r = MC->leftArm->asyncMove();
    if (r.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      CONSOLE_ERROR(ms, "Couldn't execute.  Code:  " << r.val);
    }
    }*/

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


WORD(MapPose)
virtual string description() {
  return "The pose in the map frame.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(MC->mapPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(MapPose)


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
  ms->evaluateProgram("zeroGToggle");
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
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("goHome");
  return result;
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


WORD(IncrementEndEffector)
virtual string description() {
  return "Increment the focused end effector.  All movement commands like xUp will affect only the focused end effector.";
}
virtual void execute(MachineState * ms)
{
  MC->changeFocusedEndEffector((MC->focused_ee + 1) % MC->endEffectors.size());
}
END_WORD
REGISTER_WORD(IncrementEndEffector)

WORD(DecrementEndEffector)
virtual string description() {
  return "Decrement the focused end effector.  All movement commands like xUp will affect only the focused end effector.";
}
virtual void execute(MachineState * ms)
{
  MC->changeFocusedEndEffector((MC->focused_ee - 1) % MC->endEffectors.size());
}
END_WORD
REGISTER_WORD(DecrementEndEffector)

WORD(GripperSend)
virtual string description() {
  return "Send a gripper action to the focused end effector.  For example 0.165 -1 gripperSend opens the gripper.";
}
virtual void execute(MachineState * ms)
{
  double position, effort;
  GET_NUMERIC_ARG(ms, effort);
  GET_NUMERIC_ARG(ms, position);

  control_msgs::GripperCommandGoal goal;
  goal.command.position = position;
  goal.command.max_effort = effort;
  MC->lastGripperCallTime = ros::Time::now();
  MC->focusedGripperActionClient->sendGoal(goal);
}
END_WORD
REGISTER_WORD(GripperSend)


WORD(GripperOpen)
virtual string description() {
  return "Send a gripper action to the focused end effector.  For example 0.165 -1 gripperSend opens the gripper.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("openGripper");
  return result;
}
virtual void execute(MachineState * ms)
{
  stringstream ss;
  ss << MC->gripperOpenPosition << " -1 gripperSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(GripperOpen)


WORD(GripperClose)
virtual string description() {
  return "Send a gripper action to the focused end effector.  For example 0.165 -1 gripperSend closes the gripper.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("closeGripper");
  return result;
}
virtual void execute(MachineState * ms)
{
  stringstream ss;
  ss << MC->gripperClosedPosition << " -1 gripperSend";
  ms->evaluateProgram(ss.str());
}
END_WORD
REGISTER_WORD(GripperClose)


CONFIG_GETTER_DOUBLE(GripperOpenPosition, MC->gripperOpenPosition, "The position we command the gripper to go to to open it.");
CONFIG_SETTER_DOUBLE(GripperSetOpenPosition, MC->gripperOpenPosition);
CONFIG_GETTER_DOUBLE(GripperClosedPosition, MC->gripperClosedPosition, "The position we command the gripper to go to to closed it.");
CONFIG_SETTER_DOUBLE(GripperSetClosedPosition, MC->gripperClosedPosition);




WORD(GripperWait)
virtual string description() {
  return "Wait on the gripper action.";
}
virtual void execute(MachineState * ms)
{
  ros::Duration timeout(10);
  bool result = MC->focusedGripperActionClient->waitForResult(timeout);
  if (! result) {
    CONSOLE_ERROR(ms, "Gripper goal did not finish in " << timeout);
  }
}
END_WORD
REGISTER_WORD(GripperWait)

CONFIG_GETTER_STRING(GripperActionState, MC->focusedGripperActionClient->getState().getText(), "The action state for the gripper.");


CONFIG_GETTER_STRING(FocusedEndEffectorName, CMG->getName(), "The name of the focused end effector.");
CONFIG_GETTER_INT(FocusedEndEffector, MC->focused_ee);
CONFIG_GETTER_INT(NumEndEffectors, MC->endEffectors.size());

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

CONFIG_GETTER_DOUBLE(MoveitPlanningTime, CMG->getPlanningTime(), "Moveit planning time.")

CONFIG_GETTER_STRING(MoveitPlanningFrame, CMG->getPlanningFrame(), "Moveit planning frame.")

CONFIG_GETTER_STRING(MoveitPoseReferenceFrame, CMG->getPoseReferenceFrame(), "Moveit pose reference frame.")

CONFIG_GETTER_STRING(MoveitEndEffectorLink, CMG->getEndEffectorLink(), "Moveit end effector TF link.")

CONFIG_GETTER_DOUBLE(MoveitGoalPositionTolerance, CMG->getGoalPositionTolerance(), "Moveit goal position tolerance.")

CONFIG_GETTER_DOUBLE(MoveitGoalOrientationTolerance, CMG->getGoalOrientationTolerance(), "Moveit goal orientation tolerance.")

CONFIG_GETTER_DOUBLE(MoveitGoalJointTolerance, CMG->getGoalJointTolerance(), "Moveit goal joint tolerance.")


CONFIG_GETTER_DOUBLE(finger1JointPosition, MC->fingerJointState.position[0], "Position of finger 1 for focused end effector.");
CONFIG_GETTER_DOUBLE(finger1JointVelocity, MC->fingerJointState.velocity[0], "Velocity of finger 1 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger1JointEffort, MC->fingerJointState.effort[0], "Effort of finger 1 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger2JointPosition, MC->fingerJointState.position[1], "Position of finger 2 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger2JointVelocity, MC->fingerJointState.velocity[1], "Velocity of finger 2 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger2JointEffort, MC->fingerJointState.effort[1], "Effort of finger 2 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger3JointPosition, MC->fingerJointState.position[2], "Position of finger 3 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger3JointVelocity, MC->fingerJointState.velocity[2], "Velocity of finger 3 for focused end effector..");
CONFIG_GETTER_DOUBLE(finger3JointEffort, MC->fingerJointState.effort[2], "Effort of finger 3 for focused end effector..");



CONFIG_GETTER_DOUBLE(leftFinger1JointPosition, MC->leftFingerJointState.position[0], "Position of finger 1 for left end effector.");
CONFIG_GETTER_DOUBLE(leftFinger1JointVelocity, MC->leftFingerJointState.velocity[0], "Velocity of Finger 1 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger1JointEffort, MC->leftFingerJointState.effort[0], "Effort of finger 1 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger2JointPosition, MC->leftFingerJointState.position[1], "Position of finger 2 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger2JointVelocity, MC->leftFingerJointState.velocity[1], "Velocity of finger 2 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger2JointEffort, MC->leftFingerJointState.effort[1], "Effort of finger 2 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger3JointPosition, MC->leftFingerJointState.position[2], "Position of finger 3 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger3JointVelocity, MC->leftFingerJointState.velocity[2], "Velocity of finger 3 for left end effector..");
CONFIG_GETTER_DOUBLE(leftFinger3JointEffort, MC->leftFingerJointState.effort[2], "Effort of finger 3 for left end effector..");



CONFIG_GETTER_DOUBLE(rightFinger1JointPosition, MC->rightFingerJointState.position[0], "Position of finger 1 for right end effector.");
CONFIG_GETTER_DOUBLE(rightFinger1JointVelocity, MC->rightFingerJointState.velocity[0], "Velocity of Finger 1 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger1JointEffort, MC->rightFingerJointState.effort[0], "Effort of finger 1 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger2JointPosition, MC->rightFingerJointState.position[1], "Position of finger 2 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger2JointVelocity, MC->rightFingerJointState.velocity[1], "Velocity of finger 2 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger2JointEffort, MC->rightFingerJointState.effort[1], "Effort of finger 2 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger3JointPosition, MC->rightFingerJointState.position[2], "Position of finger 3 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger3JointVelocity, MC->rightFingerJointState.velocity[2], "Velocity of finger 3 for right end effector..");
CONFIG_GETTER_DOUBLE(rightFinger3JointEffort, MC->rightFingerJointState.effort[2], "Effort of finger 3 for right end effector..");


WORD(MoveitSetGoalJointTolerance)
virtual string description() {
  return "Set moveit goal joint tolerance.";
}
virtual void execute(MachineState * ms) {
  double t;
  GET_NUMERIC_ARG(ms, t);

  CMG->setGoalJointTolerance(t);
}
END_WORD
REGISTER_WORD(MoveitSetGoalJointTolerance)

}
