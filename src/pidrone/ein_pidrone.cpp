#include "ein_pidrone.h"
#include "ein_pidrone_config.h"
#include "ein_words.h"
#include "ein.h"
#include "config.h"
#include "camera.h"

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <pidrone_pkg/Mode.h>

#include <highgui.h>

#define MC ms->config.pidroneConfig

void robotActivateSensorStreaming(MachineState * ms) {
}
void robotDeactivateSensorStreaming(MachineState * ms) {
}


void robotInitializeSerial(MachineState * ms) {
  ros::NodeHandle nh("~");  
  nh.getParam("/manifest/robot_serial", ms->config.robot_serial);
}


void robotUpdate(MachineState * ms) {
  

}

void robotInitializeConfig(MachineState * ms) {
  ms->config.pidroneConfig = new EinPidroneConfig(ms);
  ms->config.sceneInitWidth = 3;
  ms->config.sceneInitHeight = 3;

  ms->config.cameras.clear();
  string image_topic = "/pidrone/picamera/image_raw";
  Camera * c = new Camera(ms, "picamera", image_topic, "/base", "/base");
  c->imRingBufferSize = 10;
  ms->config.cameras.push_back(c);
  ms->config.focused_camera = 0;

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  
  camera->initializeConfig(240, 320);

  camera->reticle = camera->defaultReticle;

}

void robotInitializeMachine(MachineState * ms) {
  ms->evaluateProgram("waitUntilImageCallbackReceived cameraInitializeConfig zeroGOff");
}

void robotSetCurrentJointPositions(MachineState * ms) {

}

void robotEndPointCallback(MachineState * ms) {

  geometry_msgs::PoseStamped basePose;
  basePose.header.stamp = ros::Time(0);
  basePose.header.frame_id =  ms->config.eeTfFrame;

  basePose.pose.position.x = 0;
  basePose.pose.position.y = 0;
  basePose.pose.position.z = 0;
  basePose.pose.orientation.x = 0;
  basePose.pose.orientation.y = 0;
  basePose.pose.orientation.z = 0;
  basePose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped worldPose;

  try {
    ms->config.tfListener->transformPose(ms->config.baseTfFrame, basePose, worldPose);
    
    ms->config.trueEEPoseEEPose = rosPoseToEEPose(worldPose.pose);
    //CONSOLE(ms, "Stamp: " << worldPose.header.stamp);
    setRingPoseAtTime(ms, worldPose.header.stamp, worldPose.pose);
    int cfClass = ms->config.focusedClass;
    if ((cfClass > -1) && (cfClass < ms->config.classLabels.size()) && (ms->config.sensorStreamOn) && (ms->config.sisPose)) {
      //cout << "Pose now: " << worldPose.header.stamp << endl;
      double thisNow = worldPose.header.stamp.toSec();
      eePose tempPose;
      {
        tempPose.px = worldPose.pose.position.x;
        tempPose.py = worldPose.pose.position.y;
        tempPose.pz = worldPose.pose.position.z;
        tempPose.qx = worldPose.pose.orientation.x;
        tempPose.qy = worldPose.pose.orientation.y;
        tempPose.qz = worldPose.pose.orientation.z;
        tempPose.qw = worldPose.pose.orientation.w;
      }
      streamPoseAsClass(ms, tempPose, cfClass, thisNow); 
    }
  } catch (tf2::LookupException e) {
    if (MC->lastMapLookupPrintTime == ros::Time() || ros::Time::now()  - MC->lastMapLookupPrintTime > ros::Duration(20)) {
      CONSOLE_ERROR(ms, "Warning, no map frame yet.");
      MC->lastMapLookupPrintTime  = ros::Time::now();
    }
  } catch (tf::TransformException ex){
    CONSOLE_ERROR(ms, "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__);
    CONSOLE_ERROR(ms, ex.what());
  }

}


void EinPidroneConfig::endPointCallback(const ros::TimerEvent&) {
  robotEndPointCallback(ms);
}

EinPidroneConfig::EinPidroneConfig(MachineState * myms): n("~") {
  ms = myms;

  ms->config.baseTfFrame = "odom";
  ms->config.eeTfFrame = "base_link";
  eeRanger = n.subscribe("/pidrone/infrared", 1, &MachineState::rangeCallback, ms);
  eeTimer = n.createTimer(ros::Duration(0.001), &EinPidroneConfig::endPointCallback, this);

  modePub = n.advertise<pidrone_pkg::Mode>("/pidrone/set_mode", 10);
  resetTransformPub = n.advertise<std_msgs::Empty>("/pidrone/reset_transform", 10);
  toggleTransformPub = n.advertise<std_msgs::Empty>("/pidrone/toggle_transform", 10);

}

namespace ein_words {

WORD (PiDroneResetTransform)
virtual string description() {
  return "Reset the transform used for position hold (the saved image).";
}
virtual void execute(MachineState * ms) {
  std_msgs::Empty msg;
  ms->config.pidroneConfig->resetTransformPub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneResetTransform)

WORD (PiDroneToggleTransform)
virtual string description() {
  return "Toggle whether flow_pub_transform should use position or velocity mode.";
}
virtual void execute(MachineState * ms) {
  std_msgs::Empty msg;
  ms->config.pidroneConfig->toggleTransformPub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneToggleTransform)


WORD (PiDroneArm)
virtual string description() {
  return "Arm the drone.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::ARMED;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneArm)

WORD (PiDroneDisarm)
virtual string description() {
  return "Disarm the drone.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::DISARMED;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneDisarm)



WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "Ignore camera config for Pidrone!");

}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)

}
