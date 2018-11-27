#include "ein_kuka.h"
#include "ein_kuka_config.h"

#include <moveit_msgs/MoveItErrorCodes.h>
#include <robotiq_3f_gripper_control/Robotiq3FGripper_robot_output.h>

#include "config.h"
#include "ein.h"


#define MC ms->config.kukaConfig
#define CMG MC->endEffectors[MC->focused_ee]
#define EE_ARM 0

#define TRACTOR_REQUEST 5
#define STANDBY_REQUEST 4
#define POWERDOWN_REQUEST 6
#define ESTOP_REQUEST 1

using namespace std;


EinKukaConfig::EinKukaConfig(MachineState * myms): n("~")
 {
   ms = myms;

   //arm = new MoveGroup("left_arm");
   //arm->setPlannerId("RRTConnectkConfigDefault");
   endEffectors.push_back(arm);

   jointStateSubscriber = n.subscribe("/iiwa/joint_states", 1, &EinKukaConfig::jointStateCallback, this);

   cartesianPosePub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 10);
   gripperPub = n.advertise<robotiq_3f_gripper_control::Robotiq3FGripper_robot_output>("/Robotiq3FGripperRobotOutput", 10);
   
   
   changeFocusedEndEffector(EE_ARM);
}

void EinKukaConfig::jointStateCallback(const sensor_msgs::JointState& js)
{

  geometry_msgs::PoseStamped id;
  id.header.stamp = ros::Time(0);
  id.pose.position.x = 0;
  id.pose.position.y = 0;
  id.pose.position.z = 0;
  id.pose.orientation.x = 0;
  id.pose.orientation.y = 0;
  id.pose.orientation.z = 0;
  id.pose.orientation.w = 1;
  try {
    geometry_msgs::PoseStamped armPose;
    id.header.frame_id = "iiwa_link_ee";
    ms->config.tfListener->transformPose("world", id, armPose);
    MC->armPose = rosPoseToEEPose(armPose.pose);
    ms->config.trueEEPoseEEPose = MC->armPose;
  } catch (tf2::LookupException e) {
    CONSOLE_ERROR(ms, "Extrapolation Exception: " << e.what());
  } catch (tf2::ExtrapolationException e) {
    CONSOLE_ERROR(ms, "Extrapolation Exception: " << e.what());
  }

}

void EinKukaConfig::changeFocusedEndEffector(int idx)
{
  focused_ee = idx;
}



void EinKukaConfig::moveitStatusCallback(const actionlib_msgs::GoalStatusArray & m)
{
  for (int i = 0; i < m.status_list.size(); i++) {
    //MC->goals[m.status_list[i].goal_id.id] = m.status_list[i];
  }
}


void robotActivateSensorStreaming(MachineState * ms) {
}
void robotDeactivateSensorStreaming(MachineState * ms) {
}

void robotUpdate(MachineState * ms) {
  double distance, angleDistance;
  eePose::distanceXYZAndAngle(ms->config.currentEEPose, ms->config.trueEEPoseEEPose, &distance, &angleDistance);

  if (MC->focused_ee == EE_ARM) {

    if (sqrt(distance) > 0.001 || angleDistance > 0.001) {
      geometry_msgs::PoseStamped target;
      target.pose = eePoseToRosPose(ms->config.currentEEPose);
      target.header.stamp = ros::Time::now();
      target.header.frame_id = "iiwa_link_ee";

      MC->cartesianPosePub.publish(target);
      
    }
    
    /*
    if ((sqrt(distance) > 0.001 || angleDistance > 0.001) && 
	(ros::Time::now() - MC->lastMoveitCallTime  > ros::Duration(1)) &&
	(MC->lastMoveitCallPose != ms->config.currentEEPose)) {
      MC->lastMoveitCallTime = ros::Time::now();
      MC->lastMoveitCallPose = ms->config.currentEEPose;
      CMG->stop();
      geometry_msgs::PoseStamped p;
      p.pose = eePoseToRosPose(ms->config.currentEEPose);
      p.header.frame_id = "base_link";
      bool result = CMG->setPoseTarget(p);
      if (!result) {
	CONSOLE_ERROR(ms, "Invalid pose target: " << p);
      } else {
	MoveItErrorCode r = CMG->asyncMove();
	if (r.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
	  CONSOLE_ERROR(ms, "Couldn't execute.  Code:  " << r.val);
	}
      }
    }
    */
  } else {
    CONSOLE_ERROR(ms, "Bad focused EE: " << MC->focused_ee);
    assert(0);
  }
}



void robotInitializeSerial(MachineState * ms) {
  ms->config.robot_serial = "brown_kuka";
  ms->config.robot_software_version = "";
}

void robotInitializeConfig(MachineState * ms) {
 MC = new EinKukaConfig(ms);
 ms->config.robot_serial = "brown_kuka";

 ms->config.cameras.clear();
 string image_topic = "/cameras/stub/image";
 //Camera * c = new Camera(ms, "stub", image_topic, "stub", "stub");
 //ms->config.cameras.push_back(c);
 ms->config.focused_camera = 0;
 Camera * c = new Camera(ms, "kinect2_color_qhd",  "/kinect2/qhd/image_color", "kinect2_link", "kinect2_rgb_link");
 ms->config.cameras.push_back(c);

}


void robotInitializeMachine(MachineState * ms) {
  ms->evaluateProgram("\"kuka\" import"); 
  ms->evaluateProgram("zeroGToggle");
}

void robotSetCurrentJointPositions(MachineState * ms) {

  MC->armTargetPose = MC->armPose;
}

void robotEndPointCallback(MachineState * ms) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "";
  p.header.stamp = ros::Time::now();
  p.pose = eePoseToRosPose(ms->config.currentEEPose);
}


namespace ein_words {

WORD(KukaKill)
virtual string description() {
  return "Send a kill command to estop the robot.";
}
virtual void execute(MachineState * ms) {

}
END_WORD
REGISTER_WORD(KukaKill)

WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  // stub
}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)


WORD(CloseGripper)
virtual string description() {
  return "Close the gripper.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("robotiqClose");
}
END_WORD
REGISTER_WORD(CloseGripper)


WORD(OpenGripper)
virtual string description() {
  return "Open gripper.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("robotiqOpen");
}
END_WORD
REGISTER_WORD(OpenGripper)

WORD(RobotiqOpen)
virtual string description() {
  return "Open the Robotiq gripper.";
}
virtual void execute(MachineState * ms) {
  robotiq_3f_gripper_control::Robotiq3FGripper_robot_output command;
  command.rACT = 1;
  command.rGTO = 1;
  command.rSPA = 255;
  command.rFRA = 150;
  
  command.rPRA = 0;
  MC->gripperPub.publish(command);

}
END_WORD
REGISTER_WORD(RobotiqOpen)


WORD(RobotiqClose)
virtual string description() {
  return "Close the Robotiq gripper.";
}
virtual void execute(MachineState * ms) {
  robotiq_3f_gripper_control::Robotiq3FGripper_robot_output command;
  command.rACT = 1;  
  command.rGTO = 1;
  command.rSPA = 255;
  command.rFRA = 150;
  command.rPRA = 255;
  MC->gripperPub.publish(command);

}
END_WORD
REGISTER_WORD(RobotiqClose)
  


WORD(RobotiqActivate)
virtual string description() {
  return "Activate the Robotiq gripper.";
}
virtual void execute(MachineState * ms) {
  robotiq_3f_gripper_control::Robotiq3FGripper_robot_output command;
  command.rACT = 1;
  command.rGTO = 1;
  command.rSPA = 255;
  command.rFRA = 150;

  MC->gripperPub.publish(command);

}
END_WORD
REGISTER_WORD(RobotiqActivate)
  


}
