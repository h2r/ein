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

void robotActivateSensorStreaming(MachineState * ms) {
}
void robotDeactivateSensorStreaming(MachineState * ms) {
}

void robotUpdate(MachineState * ms) {
  

}

void robotInitializeConfig(MachineState * ms) {
  ms->config.pidroneConfig = new EinPidroneConfig(ms);


  ms->config.cameras.clear();
  string image_topic = "/pidrone/picamera/image_raw";
  Camera * c = new Camera(ms, "picamera", image_topic, "/base", "/base");
  c->
  ms->config.cameras.push_back(c);
  ms->config.focused_camera = 0;

  ms->config.backScanningPose = eePose(-0.304942, 0.703968, 0.186738,
                                       0.0, 1, 0.0, 0.0);
  
  ms->config.beeHome = eePose(0, 0, 0,
                              0, 0, 0, 1);

  ms->config.currentEEPose = ms->config.beeHome;
    
  ms->config.eepReg4 = ms->config.beeHome;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  
  camera->initializeConfig(240, 320);

  camera->reticle = camera->defaultReticle;
  ms->config.leftTableZ = 0;
  ms->config.currentTableZ = ms->config.leftTableZ;
  ms->config.bagTableZ = ms->config.leftTableZ;
  ms->config.counterTableZ = ms->config.leftTableZ;
  ms->config.pantryTableZ  = ms->config.leftTableZ;

  ms->config.eepReg1 = ms->config.beeHome; 
  ms->config.eepReg2 = ms->config.beeHome; 
  
  ms->config.mapSearchFenceXMin = -0.75;
  //ms->config.mapSearchFenceXMin = 0.25;
  //ms->config.mapSearchFenceXMax = 0.25;
  ms->config.mapSearchFenceXMax = 0.9; //1.0;
  ms->config.mapSearchFenceYMin = -0.5; //0.1;//-1.25;
  ms->config.mapSearchFenceYMax = 1.25;
  
  //.px = 0.278252, .py = 0.731958, .pz = -0.0533381,
  
  ms->config.mapRejectFenceXMin = ms->config.mapSearchFenceXMin;
  ms->config.mapRejectFenceXMax = ms->config.mapSearchFenceXMax;
  ms->config.mapRejectFenceYMin = ms->config.mapSearchFenceYMin;
  ms->config.mapRejectFenceYMax = ms->config.mapSearchFenceYMax;
  
  ms->config.mapBackgroundXMin = ms->config.mapSearchFenceXMin - ms->config.mapBackgroundBufferMeters;
  ms->config.mapBackgroundXMax = ms->config.mapSearchFenceXMax + ms->config.mapBackgroundBufferMeters;
  ms->config.mapBackgroundYMin = ms->config.mapSearchFenceYMin - ms->config.mapBackgroundBufferMeters;
  ms->config.mapBackgroundYMax = ms->config.mapSearchFenceYMax + ms->config.mapBackgroundBufferMeters;
  
  camera->heightReticles[0] = camera->defaultReticle;
  camera->heightReticles[1] = camera->defaultReticle;
  camera->heightReticles[2] = camera->defaultReticle;
  camera->heightReticles[3] = camera->defaultReticle;
  
  camera->heightReticles[3].px = 323;
  camera->heightReticles[2].px = 326;
  camera->heightReticles[1].px = 329;
  camera->heightReticles[0].px = 336;
  
  camera->heightReticles[3].py = 135;
  camera->heightReticles[2].py = 128;
  camera->heightReticles[1].py = 117;
  camera->heightReticles[0].py = 94;
  
    /* color reticle init */
    /* XXX TODO needs recalibrating */
    //const int camera->xCR[camera->numCReticleIndexes] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
  camera->xCR[0] = 462;
  camera->xCR[1] = 450;
  camera->xCR[2] = 439;
  camera->xCR[3] = 428;
  camera->xCR[4] = 419;
  camera->xCR[5] = 410;
  camera->xCR[6] = 405;
  camera->xCR[7] = 399;
  camera->xCR[8] = 394;
  camera->xCR[9] = 389;
  camera->xCR[10] = 383;
  camera->xCR[11] = 381;
  camera->xCR[12] = 379;
  camera->xCR[13] = 378;
  
  /* left arm */
  //const int camera->yCR[camera->numCReticleIndexes] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};
  camera->yCR[0] = 153;
  camera->yCR[1] = 153;
  camera->yCR[2] = 153;
  camera->yCR[3] = 153;
  camera->yCR[4] = 153;
  camera->yCR[5] = 154;
  camera->yCR[6] = 154;
  camera->yCR[7] = 154;
  camera->yCR[8] = 154;
  camera->yCR[9] = 154;
  camera->yCR[10] = 155;
  camera->yCR[11] = 155;
  camera->yCR[12] = 155;
  camera->yCR[13] = 155;
  
  /* lens correction */
  camera->m_x_h[0] = 1.2;
  camera->m_x_h[1] = 1.06;
  camera->m_x_h[2] = 0.98;
  camera->m_x_h[3] = 0.94;
  
  camera->m_y_h[0] = 0.95;
  camera->m_y_h[1] = 0.93;
  camera->m_y_h[2] = 0.92;
  camera->m_y_h[3] = 0.92;
  
  // ir offset
  camera->gear0offset = Eigen::Quaternionf(0.0, 0.03, 0.023, 0.0167228); // z is from TF, good for depth alignment
}

void robotInitializeMachine(MachineState * ms) {
  ms->evaluateProgram("waitUntilImageCallbackReceived cameraInitializeConfig zeroGOff");
}

void robotSetCurrentJointPositions(MachineState * ms) {

}

void robotEndPointCallback(MachineState * ms) {

  geometry_msgs::PoseStamped basePose;
  basePose.header.stamp = ros::Time(0);
  basePose.header.frame_id =  "base";

  basePose.pose.position.x = 0;
  basePose.pose.position.y = 0;
  basePose.pose.position.z = 0;
  basePose.pose.orientation.x = 0;
  basePose.pose.orientation.y = 0;
  basePose.pose.orientation.z = 0;
  basePose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped worldPose;

  try {
    ms->config.tfListener->transformPose("world", basePose, worldPose);
    
    ms->config.trueEEPose = worldPose.pose;
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
  } catch (tf::TransformException ex){
    CONSOLE_ERROR(ms, "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__);
  }

}


void EinPidroneConfig::endPointCallback(const ros::TimerEvent&) {
  robotEndPointCallback(ms);
}

EinPidroneConfig::EinPidroneConfig(MachineState * myms): n("~") {
  ms = myms;

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
  msg.mode = pidrone_pkg::Mode::MODE_ARM;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
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
  msg.mode = pidrone_pkg::Mode::MODE_DISARM;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneDisarm)

WORD (PiDroneYawRight)
virtual string description() {
  return "Yaw right.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  msg.z_velocity = 0;
  msg.yaw_velocity = 50;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneYawRight)


WORD (PiDroneYawLeft)
virtual string description() {
  return "Yaw left.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  msg.z_velocity = 0;
  msg.yaw_velocity = -50;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneYawLeft)



WORD (PiDroneTranslateDown)
virtual string description() {
  return "Translate down.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  msg.z_velocity = -2.5;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTranslateDown)


WORD (PiDroneTranslateUp)
virtual string description() {
  return "Translate up.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  msg.z_velocity = 2.5;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTranslateUp)


WORD (PiDroneTranslateBackward)
virtual string description() {
  return "Translate backwards.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = -10;
  msg.z_velocity = 0;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTranslateBackward)



WORD (PiDroneTranslateForward)
virtual string description() {
  return "Translate forwards.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 10;
  msg.z_velocity = 0;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTranslateForward)


WORD (PiDroneTranslateRight)
virtual string description() {
  return "Translate right.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 10;
  msg.y_velocity = 0;
  msg.z_velocity = 0;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTranslateRight)


WORD (PiDroneTranslateLeft)
virtual string description() {
  return "Translate left.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = -10;
  msg.y_velocity = 0;
  msg.z_velocity = 0;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTranslateLeft)

WORD (PiDroneTakeoff)
virtual string description() {
  return "Takeoff.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  msg.z_velocity = 0;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneTakeoff)


WORD (PiDroneZeroVelocity)
virtual string description() {
  return "Zero velocity.";
}
virtual void execute(MachineState * ms) {
  pidrone_pkg::Mode msg;
  msg.mode = pidrone_pkg::Mode::MODE_FLY;
  msg.x_velocity = 0;
  msg.y_velocity = 0;
  msg.z_velocity = 0;
  msg.yaw_velocity = 0;
  ms->config.pidroneConfig->modePub.publish(msg);
}
END_WORD
REGISTER_WORD(PiDroneZeroVelocity)



WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "Ignore camera config for Pidrone!");

}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)

}
