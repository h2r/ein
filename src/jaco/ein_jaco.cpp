#include "ein_jaco.h"
#include "ein_jaco_config.h"

#include <actionlib/client/simple_action_client.h>

#include "config.h"
#include "ein.h"

#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseGoal.h>

using namespace std;

EinJacoConfig::EinJacoConfig(MachineState * myms): n("~"),
						   kinova_arm_pose_action("/j2n6s300_driver/pose_action/tool_pose", true)
 {
  ms = myms;
  kinova_pose_sub = n.subscribe("/j2n6s300_driver/out/cartesian_command", 1, &EinJacoConfig::endpointCallback, this);
  kinova_arm_pose_action.waitForServer();
}

void EinJacoConfig::endpointCallback(const kinova_msgs::KinovaPose& p) {
  eePose ep = eePose::identity();
  ep = ep.applyRPYTo(p.ThetaX, p.ThetaY, p.ThetaZ);
  ep.px = p.X;
  ep.py = p.Y;
  ep.pz = p.Z;


  ms->config.lastEndpointCallbackReceived = ros::Time::now();
  geometry_msgs::PoseStamped hand_pose;
  hand_pose.pose.position.x = ep.px;
  hand_pose.pose.position.y = ep.py;
  hand_pose.pose.position.z = ep.pz;
  hand_pose.pose.orientation.x = ep.qx;
  hand_pose.pose.orientation.y = ep.qy;
  hand_pose.pose.orientation.z = ep.qz;
  hand_pose.pose.orientation.w = ep.qw;

  setRingPoseAtTime(ms, ms->config.lastEndpointCallbackReceived, hand_pose.pose);
  ms->config.trueEEPose = hand_pose.pose;

  ms->config.trueEEPoseEEPose.px = hand_pose.pose.position.x;
  ms->config.trueEEPoseEEPose.py = hand_pose.pose.position.y;
  ms->config.trueEEPoseEEPose.pz = hand_pose.pose.position.z;
  ms->config.trueEEPoseEEPose.qx = hand_pose.pose.orientation.x;
  ms->config.trueEEPoseEEPose.qy = hand_pose.pose.orientation.y;
  ms->config.trueEEPoseEEPose.qz = hand_pose.pose.orientation.z;
  ms->config.trueEEPoseEEPose.qw = hand_pose.pose.orientation.w;


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
}


void robotInitializeMachine(MachineState * ms) {
}

void robotSetCurrentJointPositions(MachineState * ms) {

}

void robotEndPointCallback(MachineState * ms) {
  
}


namespace ein_words {


WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  // stub
}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)


}
