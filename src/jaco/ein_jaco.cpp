#include "ein_jaco.h"
#include "ein_jaco_config.h"

#include <actionlib/client/simple_action_client.h>

#include "config.h"
#include "ein.h"

#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/SetFingersPositionAction.h>

using namespace std;

EinJacoConfig::EinJacoConfig(MachineState * myms): n("~"),
						   kinova_arm_pose_action("/j2n6s300_driver/pose_action/tool_pose", true),
						   kinova_fingers_position_action("/j2n6s300_driver/fingers_action/finger_positions", true)
 {
  ms = myms;
  kinova_pose_sub = n.subscribe("/j2n6s300_driver/out/tool_pose", 1, &EinJacoConfig::endpointCallback, this);
  kinova_finger_pose_sub = n.subscribe("/j2n6s300_driver/out/finger_position", 1, &EinJacoConfig::fingerCallback, this);
}

void EinJacoConfig::fingerCallback(const kinova_msgs::FingerPosition p) {
  ms->config.jacoConfig->finger1 = p.finger1 / 6400.0;
  ms->config.jacoConfig->finger2 = p.finger2 / 6400.0;
  ms->config.jacoConfig->finger3 = p.finger3 / 6400.0;

  ms->config.lastGripperCallbackReceived = ros::Time::now();
  ms->config.gripperLastUpdated = ros::Time::now();
}

void EinJacoConfig::endpointCallback(const geometry_msgs::PoseStamped& p) {
  eePose ep = eePose::fromGeometryMsgPose(p.pose);

  ms->config.lastEndpointCallbackReceived = ros::Time::now();

  setRingPoseAtTime(ms, ms->config.lastEndpointCallbackReceived, p.pose);
  ms->config.trueEEPoseEEPose = ep;
}


void robotActivateSensorStreaming(MachineState * ms) {
}
void robotDeactivateSensorStreaming(MachineState * ms) {
}

void robotUpdate(MachineState * ms) {

  //CONSOLE(ms, "State: " << ms->config.jacoConfig->kinova_arm_pose_action.getState().toString());
  if (ms->config.jacoConfig->kinova_arm_pose_action.getState().isDone()) {
    kinova_msgs::ArmPoseGoal goal;
    goal.pose.header.frame_id = "j2n6s300_link_base";
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.pose = eePoseToRosPose(ms->config.currentEEPose);
    ms->config.jacoConfig->kinova_arm_pose_action.sendGoal(goal);
  }
  

}

void robotInitializeConfig(MachineState * ms) {
 ms->config.jacoConfig = new EinJacoConfig(ms);
 ms->config.cameras.clear();

 string image_topic = "/cameras/stub/image";
 Camera * c = new Camera(ms, "stub", image_topic, "stub", "stub");
 ms->config.cameras.push_back(c);
 ms->config.focused_camera = 0;

 ms->config.beeHome = eePose(0.287482, -0.271737, 0.176811, -1, 0, 0, 0);

 ms->config.crane1 = eePose(0.211652, -0.267056, 0.503267, 
			    0.648415, 0.314313, 0.422213, 0.550001);

 ms->config.jacoConfig->candleLikePose = eePose(-0.002073, -0.009800, 1.260297, 
						 -0.001499, -0.001493, 0.708971, 0.705234);

 // ms->config.jacoConfig->calibrationPose = eePose(0.211652, -0.267056, 0.503267, 
 // 						 0.648415, 0.314313, 0.422213, 0.550001);
 ms->config.jacoConfig->calibrationPose = ms->config.crane1;

}


void robotInitializeMachine(MachineState * ms) {
    ms->pushWord("zeroGOff"); 
}

void robotSetCurrentJointPositions(MachineState * ms) {

}

void robotEndPointCallback(MachineState * ms) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "";
  p.header.stamp = ros::Time::now();
  p.pose = eePoseToRosPose(ms->config.currentEEPose);
  ms->config.jacoConfig->endpointCallback(p);
}


namespace ein_words {


WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  // stub
}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)


WORD(SetFingers)
virtual string description() {
  return "Set finger position.  Takes 3 arguments, ranging from 0-1.  0 is open, 1 is closed, controlling each of the three fingers.";
}
virtual void execute(MachineState * ms) {
  double f3;
  GET_NUMERIC_ARG(ms, f3);
  double f2;
  GET_NUMERIC_ARG(ms, f2);
  double f1;
  GET_NUMERIC_ARG(ms, f1);

  
  kinova_msgs::SetFingersPositionGoal goal;
  goal.fingers.finger1 = f1 * 6400;
  goal.fingers.finger2 = f2 * 6400;
  goal.fingers.finger3 = f3 * 6400;

    ms->config.jacoConfig->kinova_fingers_position_action.sendGoal(goal);
}
END_WORD
REGISTER_WORD(SetFingers)


WORD(WaitUntilFingersAtCurrentPosition)
virtual string description() {
  return "Waits until the fingers stop moving.";
}
virtual void execute(MachineState * ms) {
  bool finished_before_timeout = ms->config.jacoConfig->kinova_fingers_position_action.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout) {
    CONSOLE_ERROR(ms, "Finger movement wait timed out.");
    ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(WaitUntilFingersAtCurrentPosition)


WORD(WaitUntilArmAtCurrentPosition)
virtual string description() {
  return "Waits until the fingers stop moving.";
}
virtual void execute(MachineState * ms) {
  bool finished_before_timeout = ms->config.jacoConfig->kinova_arm_pose_action.waitForResult(ros::Duration(60.0));
  if (!finished_before_timeout) {
    CONSOLE_ERROR(ms, "Arm movement wait timed out.");
    ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(WaitUntilArmAtCurrentPosition)



WORD(OpenGripper)
virtual string description() {
  return "Opens the gripper.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("0 0 0 setFingers");
}
END_WORD
REGISTER_WORD(OpenGripper)


WORD(CloseGripper)
virtual string description() {
  return "Closes the gripper.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("1 1 1 setFingers");
}
END_WORD
REGISTER_WORD(CloseGripper)

WORD(AssumeCandleLikePose)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.jacoConfig->candleLikePose;
}
END_WORD
REGISTER_WORD(AssumeCandleLikePose)

WORD(AssumeCalibrationPose)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.jacoConfig->calibrationPose;
}
END_WORD
REGISTER_WORD(AssumeCalibrationPose)

CONFIG_GETTER_DOUBLE(Finger1, ms->config.jacoConfig->finger1, "Finger 1 position.")
CONFIG_GETTER_DOUBLE(Finger2, ms->config.jacoConfig->finger2, "Finger 2 position.")
CONFIG_GETTER_DOUBLE(Finger3, ms->config.jacoConfig->finger3, "Finger 3 position.")

}
