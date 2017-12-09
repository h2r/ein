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
  kinova_pose_sub = n.subscribe("/j2n6s300_driver/out/tool_pose", 1, &EinJacoConfig::endpointCallback, this);
  kinova_arm_pose_action.waitForServer();
}

void EinJacoConfig::endpointCallback(const geometry_msgs::PoseStamped& p) {
  eePose ep = eePose::fromGeometryMsgPose(p.pose);

  ms->config.lastEndpointCallbackReceived = ros::Time::now();

  setRingPoseAtTime(ms, ms->config.lastEndpointCallbackReceived, p.pose);
  ms->config.trueEEPose = p.pose;
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
