#include "ein_baxter.h"
#include "ein_baxter_config.h"
#include "ein_words.h"
#include "config.h"
#include "camera.h"
#include "ein_ik.h"
#include "eigen_util.h"

#include <dirent.h>
#include <sys/stat.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "ikfast/ikfast_wrapper_left.h"
#undef IKFAST_NAMESPACE
#undef IKFAST_SOLVER_CPP
#undef MY_NAMESPACE
#include "ikfast/ikfast_wrapper_right.h"

#include <highgui.h>

void baxterCameraInit(MachineState * ms);

void happy(MachineState * ms) {
  std_msgs::Int32 msg;
  msg.data = 0;
  ms->config.baxterConfig->facePub.publish(msg);
}

void sad(MachineState * ms) {
  std_msgs::Int32 msg;
  msg.data = 99;
  ms->config.baxterConfig->facePub.publish(msg);
}

void neutral(MachineState * ms) {
  std_msgs::Int32 msg;
  msg.data = 50;
  ms->config.baxterConfig->facePub.publish(msg);
}

void robotInitializeSerial(MachineState * ms) {
  ros::NodeHandle nh("~");  
  nh.getParam("/manifest/robot_serial", ms->config.robot_serial);
  nh.getParam("/rethink/software_version", ms->config.robot_software_version);
}

void robotInitializeConfig(MachineState * ms) {
 ms->config.baxterConfig = new EinBaxterConfig(ms);

 ms->config.cameras.clear();
 if ( (ms->config.left_or_right_arm.compare("right") == 0) || (ms->config.left_or_right_arm.compare("left") == 0) ) {
   string image_topic = "/cameras/" + ms->config.left_or_right_arm + "_hand_camera/image";
   Camera * c = new Camera(ms, ms->config.left_or_right_arm + "_hand_camera", image_topic, ms->config.left_or_right_arm + "_hand", ms->config.left_or_right_arm + "_hand_camera");
   ms->config.cameras.push_back(c);
   ms->config.focused_camera = 0;
 } else {
   CONSOLE_ERROR(ms, "Could not find camear for arm: " << ms->config.left_or_right_arm);
 }
 baxterCameraInit(ms);
}

void robotInitializeMachine(MachineState * ms) {
    ms->evaluateProgram("cameraFitHyperbolic 2 cameraSetCalibrationMode");
  if (ms->config.currentRobotMode == PHYSICAL) {

    ms->pushWord("zeroGOff"); 
    ms->pushWord("waitUntilEndpointCallbackReceived"); 
    
    ms->pushCopies("zUp", 15);
    ms->pushWord("incrementTargetClass"); 

    ms->pushWord("silenceSonar");
    ms->pushWord("exportWords");
    ms->pushWord("openGripper");
    ms->pushWord("calibrateGripper");
    
    ms->pushWord("fillClearanceMap"); 
  }

  ms->pushWord("loadIkMap"); 
  ms->pushWord("loadGripperMask"); 

}

void robotSetCurrentJointPositions(MachineState * ms) {
  if ( (ms->config.baxterConfig->currentJointPositions.response.joints.size() > 0) && (ms->config.baxterConfig->currentJointPositions.response.joints[0].position.size() == NUM_JOINTS) ) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      ms->config.baxterConfig->currentJointPositions.response.joints[0].position[j] = ms->config.trueJointPositions[j];
    }
  } else {
    ms->config.baxterConfig->currentJointPositions.response.joints.resize(1);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].name.resize(NUM_JOINTS);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position.resize(NUM_JOINTS);
  }
}

void robotEndPointCallback(MachineState * ms) {
    baxter_core_msgs::EndpointState myEPS;

    myEPS.header.stamp = ros::Time::now();
    myEPS.pose.position.x = ms->config.currentEEPose.px;
    myEPS.pose.position.y = ms->config.currentEEPose.py;
    myEPS.pose.position.z = ms->config.currentEEPose.pz;
    myEPS.pose.orientation.x = ms->config.currentEEPose.qx;
    myEPS.pose.orientation.y = ms->config.currentEEPose.qy;
    myEPS.pose.orientation.z = ms->config.currentEEPose.qz;
    myEPS.pose.orientation.w = ms->config.currentEEPose.qw;

    ms->config.baxterConfig->endpointCallback(myEPS);

}




EinBaxterConfig::EinBaxterConfig(MachineState * myms): n("~") {
  ms = myms;
  currentHeadPanCommand.target = 0;

#ifdef RETHINK_SDK_1_2_0
  currentHeadPanCommand.speed_ratio = 0.5;
#else
  currentHeadPanCommand.speed = 50;
#endif
  currentHeadNodCommand.data = 0;
  currentSonarCommand.data = 0;


  sonarPub = n.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/set_sonars_enabled",10);
  headPub = n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",10);
  nodPub = n.advertise<std_msgs::Bool>("/robot/head/command_head_nod",10);
  joint_mover = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + ms->config.left_or_right_arm + "/joint_command", 10);
  digital_io_pub = n.advertise<baxter_core_msgs::DigitalOutputCommand>("/robot/digital_io/command",10);
  analog_io_pub = n.advertise<baxter_core_msgs::AnalogOutputCommand>("/robot/analog_io/command",10);

  sonar_pub = n.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/lights/set_lights",10);
  red_halo_pub = n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_red_level",10);
  green_halo_pub = n.advertise<std_msgs::Float32>("/robot/sonar/head_sonar/lights/set_green_level",10);
  face_screen_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay",10);
  facePub = n.advertise<std_msgs::Int32>("/confusion/target/command", 10);


  gripperPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/" + ms->config.left_or_right_arm + "_gripper/command",10);
  moveSpeedPub = n.advertise<std_msgs::Float64>("/robot/limb/" + ms->config.left_or_right_arm + "/set_speed_ratio",10);

  stiffPub = n.advertise<std_msgs::UInt32>("/robot/limb/" + ms->config.left_or_right_arm + "/command_stiffness",10);
  ikClient = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/" + ms->config.left_or_right_arm + "/PositionKinematicsNode/IKService");

  cameraClient = n.serviceClient<baxter_core_msgs::OpenCamera>("/cameras/open");

  ms = myms;
  if (ms->config.currentRobotMode == PHYSICAL || ms->config.currentRobotMode == SNOOP) {
    epState =   n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/endpoint_state", 1, &EinBaxterConfig::endpointCallback, this);
    eeRanger =  n.subscribe("/robot/range/" + ms->config.left_or_right_arm + "_hand_range/state", 1, &MachineState::rangeCallback, ms);

    gravity_comp_sub = n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/gravity_compensation_torques", 1, &EinBaxterConfig::gravityCompCallback, this);

    cuff_grasp_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_upper_button/state", 1, &EinBaxterConfig::cuffGraspCallback, this);
    cuff_ok_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_lower_button/state", 1, &EinBaxterConfig::cuffOkCallback, this);
    shoulder_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_shoulder_button/state", 1, &EinBaxterConfig::shoulderCallback, this);

    torso_fan_sub = n.subscribe("/robot/analog_io/torso_fan/state", 1, &EinBaxterConfig::torsoFanCallback, this);

#ifdef RETHINK_SDK_1_2_0
    arm_button_back_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_button_back/state", 1, &EinBaxterConfig::armBackButtonCallback, this);
    arm_button_ok_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_button_ok/state", 1, &EinBaxterConfig::armOkButtonCallback, this);
    arm_button_show_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_button_show/state", 1, &EinBaxterConfig::armShowButtonCallback, this);
#else
    arm_button_back_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_itb_button1/state", 1, &EinBaxterConfig::armBackButtonCallback, this);
    arm_button_ok_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_itb_button0/state", 1, &EinBaxterConfig::armOkButtonCallback, this);
    arm_button_show_sub = n.subscribe("/robot/digital_io/" + ms->config.left_or_right_arm + "_itb_button2/state", 1, &EinBaxterConfig::armShowButtonCallback, this);
#endif


    collisionDetectionState = n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/collision_detection_state", 1, &EinBaxterConfig::collisionDetectionStateCallback, this);
    gripState = n.subscribe("/robot/end_effector/" + ms->config.left_or_right_arm + "_gripper/state", 1, &EinBaxterConfig::gripStateCallback, this);
    eeAccelerator =  n.subscribe("/robot/accelerometer/" + ms->config.left_or_right_arm + "_accelerometer/state", 1, &MachineState::accelerometerCallback, ms);
    eeTarget =  n.subscribe("/ein_" + ms->config.left_or_right_arm + "/pilot_target_" + ms->config.left_or_right_arm, 1, &MachineState::targetCallback, ms);
    jointSubscriber = n.subscribe("/robot/joint_states", 1, &EinBaxterConfig::jointCallback, this);

  } else if (ms->config.currentRobotMode == SIMULATED) {
    cout << "SIMULATION mode enabled." << endl;

    ms->config.simulatorCallbackTimer = n.createTimer(ros::Duration(1.0/ms->config.simulatorCallbackFrequency), &MachineState::simulatorCallback, ms);


    { // load sprites
      // snoop data/sprites folder
      //   loop through subfolders
      //     load image.ppm for now, default everything else
      vector<string> spriteLabels;
      spriteLabels.resize(0);
      ms->config.masterSprites.resize(0);
      ms->config.instanceSprites.resize(0);
      DIR *dpdf;
      struct dirent *epdf;
      string dot(".");
      string dotdot("..");

      char buf[1024];
      sprintf(buf, "%s/simulator/sprites", ms->config.data_directory.c_str());
      dpdf = opendir(buf);
      if (dpdf != NULL){
	while (epdf = readdir(dpdf)){
	  string thisFileName(epdf->d_name);

	  string thisFullFileName(buf);
	  thisFullFileName = thisFullFileName + "/" + thisFileName;
	  cout << "checking " << thisFullFileName << " during sprite snoop...";

	  struct stat buf2;
	  stat(thisFullFileName.c_str(), &buf2);

	  int itIsADir = S_ISDIR(buf2.st_mode);
	  if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	    spriteLabels.push_back(thisFileName);
	    cout << " is a directory." << endl;
	  } else {
	    cout << " is NOT a directory." << endl;
	  }
	}
      }

      ms->config.masterSprites.resize(spriteLabels.size());
      for (int s = 0; s < ms->config.masterSprites.size(); s++) {
	ms->config.masterSprites[s].name = spriteLabels[s];
	string filename = ms->config.data_directory + "/simulator/sprites/" + ms->config.masterSprites[s].name + "/image.ppm";
	cout << "loading sprite from " << filename << " ... ";

	Mat tmp = imread(filename);
	ms->config.masterSprites[s].image = tmp;
	ms->config.masterSprites[s].scale = 15/.01;

	ms->config.masterSprites[s].top = eePose::zero();
	ms->config.masterSprites[s].bot = eePose::zero();
	ms->config.masterSprites[s].pose = eePose::zero();
	cout << "loaded " << ms->config.masterSprites[s].name << " as masterSprites[" << s << "] scale " << ms->config.masterSprites[s].scale << " image size " << ms->config.masterSprites[s].image.size() << endl;
      }
    }

    // load background
    int tileBackground = 1;
    if (tileBackground) {
      string filename;
      filename = ms->config.data_directory + "/simulator/tableTile.png";
      cout << "loading mapBackgroundImage from " << filename << " "; cout.flush();
      Mat tmp = imread(filename);
      cout << "done. Tiling " << tmp.size() << " "; cout.flush();
      //cout << "downsampling... "; cout.flush();
      //cv::resize(tmp, tmp, cv::Size(tmp.cols/2,tmp.rows/2));
      cv::resize(tmp, ms->config.mapBackgroundImage, cv::Size(ms->config.mbiWidth,ms->config.mbiHeight));

      int tilesWidth = ms->config.mbiWidth / tmp.cols;
      int tilesHeight = ms->config.mbiHeight / tmp.rows;

      for (int tx = 0; tx < tilesWidth; tx++) {
	for (int ty = 0; ty < tilesHeight; ty++) {
	  Mat crop = ms->config.mapBackgroundImage(cv::Rect(tx*tmp.cols, ty*tmp.rows, tmp.cols, tmp.rows));
	  resize(tmp, crop, crop.size(), 0, 0, CV_INTER_LINEAR);
	  if (tx % 2) {
	    flip(crop, crop, 1);
	  }
	  if ((ty) % 2) {
	    flip(crop, crop, 0);
	  }
	}
      }

      cout << "done. " << ms->config.mapBackgroundImage.size() << endl; cout.flush();
    } else {
      string filename;
      //filename = ms->config.data_directory + "/mapBackground.ppm";
      filename = ms->config.data_directory + "/simulator/carpetBackground.jpg";
      cout << "loading mapBackgroundImage from " << filename << " "; cout.flush();
      Mat tmp = imread(filename);
      cout << "done. Resizing " << tmp.size() << " "; cout.flush();
      cv::resize(tmp, ms->config.mapBackgroundImage, cv::Size(ms->config.mbiWidth,ms->config.mbiHeight));
      cout << "done. " << ms->config.mapBackgroundImage.size() << endl; cout.flush();
    }
    ms->config.originalMapBackgroundImage = ms->config.mapBackgroundImage.clone();

  }  else {
    assert(0);
  }

}


void EinBaxterConfig::gravityCompCallback(const baxter_core_msgs::SEAJointState& seaJ) {

  for (int i = 0; i < NUM_JOINTS; i++) {
    ms->config.last_joint_actual_effort[i] = seaJ.actual_effort[i];
  }
}

void EinBaxterConfig::cuffGraspCallback(const baxter_core_msgs::DigitalIOState& cuffDIOS) {

  if (cuffDIOS.state == 1) {
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    command.args = "{\"position\": 0.0}";
    command.id = 65538;
    ms->config.baxterConfig->gripperPub.publish(command);
  } else {
  }

}

void EinBaxterConfig::cuffOkCallback(const baxter_core_msgs::DigitalIOState& cuffDIOS) {

  if (cuffDIOS.state == 1) {
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    command.args = "{\"position\": 100.0}";
    command.id = 65538;
    ms->config.baxterConfig->gripperPub.publish(command);
    ms->config.lastMeasuredClosed = ms->config.gripperPosition;
  } else {
  }

}


void EinBaxterConfig::armShowButtonCallback(const baxter_core_msgs::DigitalIOState& dios) {


  if (dios.state == 1) {
    // only if this is the first of recent presses
    if (ms->config.lastArmOkButtonState == 0) {
      ms->evaluateProgram("infiniteScan");
    }      
  }

  if (dios.state) {
    ms->config.lastArmShowButtonState = 1;
  } else {
    ms->config.lastArmShowButtonState = 0;
  }
}

void EinBaxterConfig::armBackButtonCallback(const baxter_core_msgs::DigitalIOState& dios) {


  if (dios.state == 1) {
    // only if this is the first of recent presses
    if (ms->config.lastArmOkButtonState == 0) {
      ms->clearStack();
      ms->clearData();

      ms->evaluateProgram("inputPileWorkspace moveEeToPoseWord");
    }      
  }

  if (dios.state) {
    ms->config.lastArmBackButtonState = 1;
  } else {
    ms->config.lastArmBackButtonState = 0;
  }
}

void EinBaxterConfig::armOkButtonCallback(const baxter_core_msgs::DigitalIOState& dios) {

  if (dios.state == 1) {
    // only if this is the first of recent presses
    if (ms->config.lastArmOkButtonState == 0) {
      ms->evaluateProgram("zeroGToggle");
    }      
  }

  if (dios.state == 1) {
    ms->config.lastArmOkButtonState = 1;
  } else if (dios.state == 0) {
    ms->config.lastArmOkButtonState = 0;
  } else {
    CONSOLE_ERROR(ms, "Unexpected state: " << dios);
    assert(0);
  }

}

void EinBaxterConfig::torsoFanCallback(const baxter_core_msgs::AnalogIOState& aios) {

  ms->config.torsoFanState = aios.value;
}

void EinBaxterConfig::shoulderCallback(const baxter_core_msgs::DigitalIOState& dios) {

  // this is backwards, probably a bug
  if (!dios.state && ms->config.lastShoulderState == 1) {
    ms->config.intendedEnableState = !ms->config.intendedEnableState;
    if (ms->config.intendedEnableState) {
      int sis = system("bash -c \"echo -e \'C\003\' | rosrun baxter_tools enable_robot.py -e\"");
    } else {
      int sis = system("bash -c \"echo -e \'C\003\' | rosrun baxter_tools enable_robot.py -d\"");
    }
  }

  if (dios.state) {
    ms->config.lastShoulderState = 1;
  } else {
    ms->config.lastShoulderState = 0;
  }

  void jointCallback(const sensor_msgs::JointState& js);

}



void EinBaxterConfig::jointCallback(const sensor_msgs::JointState& js) {

  if (ms->config.jointNamesInit) {
    int limit = js.position.size();
    for (int i = 0; i < limit; i++) {
      for (int j = 0; j < NUM_JOINTS; j++) {
	if (0 == js.name[i].compare(ms->config.jointNames[j]))
	  ms->config.trueJointPositions[j] = js.position[i];
	  ms->config.trueJointVelocities[j] = js.velocity[i];
	  ms->config.trueJointEfforts[j] = js.effort[i];
	//cout << "tJP[" << j << "]: " << trueJointPositions[j] << endl;
      }
    }
  }
}


void EinBaxterConfig::collisionDetectionStateCallback(const baxter_core_msgs::CollisionDetectionState& cds) {
  CollisionDetection detection;
  
  detection.inCollision = cds.collision_state;
  detection.time = cds.header.stamp;

  ms->config.collisionStateBuffer.push_front(detection);
  while (ms->config.collisionStateBuffer.size() > 100) {
    ms->config.collisionStateBuffer.pop_back();
  }

}


void EinBaxterConfig::endpointCallback(const baxter_core_msgs::EndpointState& _eps) {
  baxter_core_msgs::EndpointState eps = _eps;

  eePose endPointEEPose;
  {
    endPointEEPose.px = _eps.pose.position.x;
    endPointEEPose.py = _eps.pose.position.y;
    endPointEEPose.pz = _eps.pose.position.z;
    endPointEEPose.qx = _eps.pose.orientation.x;
    endPointEEPose.qy = _eps.pose.orientation.y;
    endPointEEPose.qz = _eps.pose.orientation.z;
    endPointEEPose.qw = _eps.pose.orientation.w;
  }

  ms->config.lastEndpointCallbackReceived = ros::Time::now();

  // note that the quaternion field holds a vector3!
  ms->config.trueEEWrench.px = eps.wrench.force.x;
  ms->config.trueEEWrench.py = eps.wrench.force.y;
  ms->config.trueEEWrench.pz = eps.wrench.force.z;
  ms->config.trueEEWrench.qx = eps.wrench.torque.x;
  ms->config.trueEEWrench.qy = eps.wrench.torque.y;
  ms->config.trueEEWrench.qz = eps.wrench.torque.z;

  double thisWrenchNorm = eePose::distance(eePose::zero(), ms->config.trueEEWrench);
  double td = ms->config.averagedWrechDecay;
  //cout << "JJJ theWrenchNorm " << thisWrenchNorm << " " << td << endl;
  ms->config.averagedWrechAcc = (1.0-td)*thisWrenchNorm + (td)*ms->config.averagedWrechAcc;
  ms->config.averagedWrechMass =  (1.0-td)*1 + (td)*ms->config.averagedWrechMass;
  //cout << "JJJ " << ms->config.averagedWrechMass << " " << ms->config.averagedWrechAcc << endl;

  //cout << "endpoint frame_id: " << eps.header.frame_id << endl;
  // XXX
  //ms->config.tfListener
  //tf::StampedTransform transform;
  //ms->config.tfListener->lookupTransform("base", ms->config.left_or_right_arm + "_gripper_base", ros::Time(0), transform);

  geometry_msgs::PoseStamped hand_pose;
  //tf::StampedTransform base_to_hand_transform;
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    //pose.header.stamp = ros::Time(0);
    pose.header.stamp = eps.header.stamp;
    pose.header.frame_id =  ms->config.left_or_right_arm + "_hand";

    if (ms->config.currentRobotMode != SIMULATED) {    
      try {
        ms->config.tfListener->waitForTransform("base", ms->config.left_or_right_arm + "_hand", pose.header.stamp, ros::Duration(1.0));
        ms->config.tfListener->transformPose("base", pose.header.stamp, pose, ms->config.left_or_right_arm + "_hand", hand_pose);
      } catch (tf::TransformException ex){
        CONSOLE_ERROR(ms, "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__);
        CONSOLE_ERROR(ms, ex.what())
      }
    } else {
      //pose = ms->config.currentEEPose;
    }
    //ms->config.tfListener->lookupTransform("base", ms->config.left_or_right_arm + "_hand", ros::Time(0), base_to_hand_transform);
  }


  // XXX
  // right now we record the position of the hand and then switch current and
  // true pose to refer to where the gripper actually is, which is a minor
  // inconsistency
  setRingPoseAtTime(ms, hand_pose.header.stamp, hand_pose.pose);
  geometry_msgs::Pose thisPose;
  int weHavePoseData = getRingPoseAtTime(ms, hand_pose.header.stamp, thisPose);

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size()) && (ms->config.sensorStreamOn) && (ms->config.sisPose)) {
    double thisNow = hand_pose.header.stamp.toSec();
    eePose tempPose;
    {
      tempPose.px = hand_pose.pose.position.x;
      tempPose.py = hand_pose.pose.position.y;
      tempPose.pz = hand_pose.pose.position.z;
      tempPose.qx = hand_pose.pose.orientation.x;
      tempPose.qy = hand_pose.pose.orientation.y;
      tempPose.qz = hand_pose.pose.orientation.z;
      tempPose.qw = hand_pose.pose.orientation.w;
    }
    streamPoseAsClass(ms, tempPose, cfClass, thisNow); 
  } 
  // XXX 

  eePose handEEPose;
  {
    handEEPose.px = hand_pose.pose.position.x;
    handEEPose.py = hand_pose.pose.position.y;
    handEEPose.pz = hand_pose.pose.position.z;
    handEEPose.qx = hand_pose.pose.orientation.x;
    handEEPose.qy = hand_pose.pose.orientation.y;
    handEEPose.qz = hand_pose.pose.orientation.z;
    handEEPose.qw = hand_pose.pose.orientation.w;
  }

  if (eePose::distance(handEEPose, ms->config.lastHandEEPose) == 0 && ms->config.currentRobotMode != SIMULATED) {
    //CONSOLE_ERROR(ms, "Ooops, duplicate pose: " << tArmP.px << " " << tArmP.py << " " << tArmP.pz << " " << endl);
  }
  ms->config.lastHandEEPose = handEEPose;
  
  ms->config.handToRethinkEndPointTransform = endPointEEPose.getPoseRelativeTo(handEEPose);
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = ms->config.handEndEffectorOffset.px;
    pose.pose.position.y = ms->config.handEndEffectorOffset.py;
    pose.pose.position.z = ms->config.handEndEffectorOffset.pz;
    pose.pose.orientation.x = ms->config.handEndEffectorOffset.qx;
    pose.pose.orientation.y = ms->config.handEndEffectorOffset.qy;
    pose.pose.orientation.z = ms->config.handEndEffectorOffset.qz;
    pose.pose.orientation.w = ms->config.handEndEffectorOffset.qw;

    //pose.header.stamp = ros::Time(0);
    pose.header.stamp = eps.header.stamp;
    pose.header.frame_id =  ms->config.left_or_right_arm + "_hand";
    
    geometry_msgs::PoseStamped transformed_pose;
    if (ms->config.currentRobotMode != SIMULATED) {    
      try {
        ms->config.tfListener->waitForTransform("base", ms->config.left_or_right_arm + "_hand", pose.header.stamp, ros::Duration(1.0));
        ms->config.tfListener->transformPose("base", pose.header.stamp, pose, ms->config.left_or_right_arm + "_hand", transformed_pose);
      } catch (tf::TransformException ex){
        cout << "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__ << endl;
        cout << ex.what();
        //ROS_ERROR("%s", ex.what());
        //throw;
      }
    }

    eps.pose.position.x = transformed_pose.pose.position.x;
    eps.pose.position.y = transformed_pose.pose.position.y;
    eps.pose.position.z = transformed_pose.pose.position.z;
    eps.pose.orientation = transformed_pose.pose.orientation;

    //cout << pose << transformed_pose << hand_pose;
    //cout << base_to_hand_transform.getOrigin().x() << base_to_hand_transform.getOrigin().y() << base_to_hand_transform.getOrigin().z() << endl;
    //tf::Vector3 test(0,0,0.03);
    //tf::Vector3 test2 = base_to_hand_transform * test;
    //cout <<  test2.x() << test2.y() << test2.z() << endl;
  }
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->updateTrueCameraPoseWithHandCameraOffset(eps.header.stamp);
  }


  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = ms->config.handRangeOffset.px;
    pose.pose.position.y = ms->config.handRangeOffset.py;
    pose.pose.position.z = ms->config.handRangeOffset.pz;
    pose.pose.orientation.x = ms->config.handRangeOffset.qx;
    pose.pose.orientation.y = ms->config.handRangeOffset.qy;
    pose.pose.orientation.z = ms->config.handRangeOffset.qz;
    pose.pose.orientation.w = ms->config.handRangeOffset.qw;

    //pose.header.stamp = ros::Time(0);
    pose.header.stamp = eps.header.stamp;
    pose.header.frame_id =  ms->config.left_or_right_arm + "_hand";
    
    geometry_msgs::PoseStamped transformed_pose;
    if (ms->config.currentRobotMode != SIMULATED) {    
      try {
        ms->config.tfListener->waitForTransform("base", ms->config.left_or_right_arm + "_hand", pose.header.stamp, ros::Duration(1.0));
        ms->config.tfListener->transformPose("base", pose.header.stamp, pose, ms->config.left_or_right_arm + "_hand", transformed_pose);
      } catch (tf::TransformException ex){
        cout << "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__ << endl;
        cout << ex.what();
        //ROS_ERROR("%s", ex.what());
        //throw;
      }
    }

    ms->config.trueRangePose.px = transformed_pose.pose.position.x;
    ms->config.trueRangePose.py = transformed_pose.pose.position.y;
    ms->config.trueRangePose.pz = transformed_pose.pose.position.z;
    ms->config.trueRangePose.qx = transformed_pose.pose.orientation.x;
    ms->config.trueRangePose.qy = transformed_pose.pose.orientation.y;
    ms->config.trueRangePose.qz = transformed_pose.pose.orientation.z;
    ms->config.trueRangePose.qw = transformed_pose.pose.orientation.w;
  }

// XXX TODO get a better convention for controlling gripper position, maybe
// this should be done by grasp calculators
  {
    ms->config.trueEEPoseEEPose.px = eps.pose.position.x;
    ms->config.trueEEPoseEEPose.py = eps.pose.position.y;
    ms->config.trueEEPoseEEPose.pz = eps.pose.position.z;
    ms->config.trueEEPoseEEPose.qx = eps.pose.orientation.x;
    ms->config.trueEEPoseEEPose.qy = eps.pose.orientation.y;
    ms->config.trueEEPoseEEPose.qz = eps.pose.orientation.z;
    ms->config.trueEEPoseEEPose.qw = eps.pose.orientation.w;
  }
  ms->config.handFromEndEffectorTransform = handEEPose.getPoseRelativeTo(ms->config.trueEEPoseEEPose);
// XXX




  {
    double distance = eePose::squareDistance(ms->config.trueEEPoseEEPose, ms->config.lastTrueEEPoseEEPose);
    double distance2 = eePose::squareDistance(ms->config.trueEEPoseEEPose, ms->config.currentEEPose);

    if (ms->config.currentMovementState == ARMED ) {
      if (distance2 > ms->config.armedThreshold*ms->config.armedThreshold) {
	cout << "armedThreshold crossed so leaving armed state into MOVING." << endl;
	ms->config.currentMovementState = MOVING;
	ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	ms->config.lastMovementStateSet = ros::Time::now();
      } else {
	//cout << "ms->config.currentMovementState is ARMED." << endl;
      }
    } else if (distance > ms->config.movingThreshold*ms->config.movingThreshold) {
      ms->config.currentMovementState = MOVING;
      ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
      ms->config.lastMovementStateSet = ros::Time::now();
    } else if (distance > ms->config.hoverThreshold*ms->config.hoverThreshold) {
      if (distance2 > ms->config.hoverThreshold) {
	ms->config.currentMovementState = MOVING;
	ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	ms->config.lastMovementStateSet = ros::Time::now();
      } else {
	ms->config.currentMovementState = HOVERING;
	ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	ms->config.lastMovementStateSet = ros::Time::now();
      }

    } else {
      ros::Duration deltaT = ros::Time::now() - ms->config.lastMovementStateSet;
      if ( (deltaT.sec) > ms->config.stoppedTimeout ) {
	if (distance2 > ms->config.hoverThreshold*ms->config.hoverThreshold) {
	  ms->config.currentMovementState = BLOCKED;
	  ms->config.lastMovementStateSet = ros::Time::now();
	  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	} else {
	  ms->config.currentMovementState = STOPPED;
	  ms->config.lastMovementStateSet = ros::Time::now();
	  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	}
      }
    }
  }
}

void robotActivateSensorStreaming(MachineState * ms) {
  ms->config.baxterConfig->activateSensorStreaming();
}
void robotDeactivateSensorStreaming(MachineState * ms) {
  ms->config.baxterConfig->deactivateSensorStreaming();
}

void EinBaxterConfig::deactivateSensorStreaming() {
  cout << "deactivateSensorStreaming: Subscribe to endpoint_state." << endl;
  epState =   n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/endpoint_state", 1, &EinBaxterConfig::endpointCallback, this);
  cout << "deactivateSensorStreaming: Subscribe to hand_range." << endl;
  eeRanger =  n.subscribe("/robot/range/" + ms->config.left_or_right_arm + "_hand_range/state", 1, &MachineState::rangeCallback, ms);
}
void EinBaxterConfig::activateSensorStreaming() {
    // turn that queue size up!
    epState =   n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/endpoint_state", 100, &EinBaxterConfig::endpointCallback, this);
    eeRanger =  n.subscribe("/robot/range/" + ms->config.left_or_right_arm + "_hand_range/state", 100, &MachineState::rangeCallback, ms);

}

void EinBaxterConfig::gripStateCallback(const baxter_core_msgs::EndEffectorState& ees) {


  ms->config.lastGripperCallbackReceived = ros::Time::now();
  ms->config.gripperLastUpdated = ros::Time::now();
  ms->config.gripperPosition  = ees.position;
  ms->config.gripperMoving = ees.moving;
  ms->config.gripperGripping = ees.gripping;
}

void robotUpdate(MachineState * ms) {
  ms->config.baxterConfig->update_baxter();
}

void EinBaxterConfig::update_baxter() {
  ms->config.bfc = ms->config.bfc % ms->config.bfc_period;
  if (!ms->config.shouldIDoIK) {
    return;
  }

  if (ms->config.currentRobotMode == SIMULATED) {
    return;
  }
  if (ms->config.currentRobotMode == SNOOP) {
    return;
  }

  baxter_core_msgs::SolvePositionIK thisIkRequest;
  fillIkRequest(ms->config.currentEEPose, &thisIkRequest);

  int ikResultFailed = 0;
  eePose originalCurrentEEPose = ms->config.currentEEPose;

  // do not start in a state with ikShare 
  if ((drand48() <= ms->config.ikShare) || !ms->config.ikInitialized) {

    int numIkRetries = 2;//100; //5000;//100;
    double ikNoiseAmplitude = 0.01;//0.1;//0.03;
    double useZOnly = 1;
    double ikNoiseAmplitudeQuat = 0;
    for (int ikRetry = 0; ikRetry < numIkRetries; ikRetry++) {
      // ATTN 24
      //int ikCallResult = ms->config.ikClient.call(thisIkRequest);
      int ikCallResult = 0;
      queryIK(ms, &ikCallResult, &thisIkRequest);

      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	// set this here in case noise was added
	ms->config.currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
	ms->config.currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
	ms->config.currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
	ikResultFailed = 0;
	if (ikRetry > 0) {
	  ROS_WARN_STREAM("___________________");
	  CONSOLE_ERROR(ms, "Accepting perturbed IK result.");
	  cout << "ikRetry: " << ikRetry << endl;
	  eePose::print(originalCurrentEEPose);
	  eePose::print(ms->config.currentEEPose);
	  ROS_WARN_STREAM("___________________");
	}
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;

	  ikResultFailed = 0;
	  ms->config.currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
	  ms->config.currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
	  ms->config.currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      if (!ikResultFailed) {
	break;
      }

      ROS_WARN_STREAM("Initial IK result invalid... adding noise and retrying.");
      cout << thisIkRequest.request.pose_stamp[0].pose << endl;

      reseedIkRequest(ms, &ms->config.currentEEPose, &thisIkRequest, ikRetry, numIkRetries);
      fillIkRequest(ms->config.currentEEPose, &thisIkRequest);
    }
  }
  
  /*if ( ms->config.ikClient.waitForExistence(ros::Duration(1, 0)) ) {
    ikResultFailed = (!ms->config.ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);
  } else {
    cout << "waitForExistence timed out" << endl;
    ikResultFailed = 1;
  }*/

  if (ikResultFailed) 
  {
    CONSOLE_ERROR(ms, "ikClient says pose request is invalid.");
    ms->config.ik_reset_counter++;
    ms->config.lastIkWasSuccessful = false;

    cout << "ik_reset_counter, ik_reset_thresh: " << ms->config.ik_reset_counter << " " << ms->config.ik_reset_thresh << endl;
    if (ms->config.ik_reset_counter > ms->config.ik_reset_thresh) {
      ms->config.ik_reset_counter = 0;
      //ms->config.currentEEPose = ms->config.ik_reset_eePose;
      //cout << "  reset pose disabled! setting current position to true position." << endl;
      //ms->config.currentEEPose = ms->config.trueEEPoseEEPose;
      cout << "  reset pose disabled! setting current position to last good position." << endl;
      ms->config.currentEEPose = ms->config.lastGoodEEPose;
      //ms->pushWord("pauseStackExecution"); // pause stack execution
      cout << "  pausing disabled!" << endl;
      ms->pushCopies("beep", 15); // beep
      cout << "target position denied by ik, please reset the object.";
    }
    else {
      cout << "This pose was rejected by ikClient:" << endl;
      cout << "Current EE Position (x,y,z): " << ms->config.currentEEPose.px << " " << ms->config.currentEEPose.py << " " << ms->config.currentEEPose.pz << endl;
      cout << "Current EE Orientation (x,y,z,w): " << ms->config.currentEEPose.qx << " " << ms->config.currentEEPose.qy << " " << ms->config.currentEEPose.qz << " " << ms->config.currentEEPose.qw << endl;

      if (ms->config.currentIKBoundaryMode == IK_BOUNDARY_STOP) {
	ms->config.currentEEPose = ms->config.lastGoodEEPose;
      } else if (ms->config.currentIKBoundaryMode == IK_BOUNDARY_PASS) {
      } else {
	assert(0);
      }
    }

    return;
  }
  ms->config.lastIkWasSuccessful = true;
  ms->config.ik_reset_counter = max(ms->config.ik_reset_counter-1, 0);

  ms->config.lastGoodEEPose = ms->config.currentEEPose;
  this->ikRequest = thisIkRequest;
  ms->config.ikInitialized = 1;
  

  // but in theory we can bypass the joint controllers by publishing to this topic
  // /robot/limb/left/joint_command

  baxter_core_msgs::JointCommand myCommand;

  if (!ms->config.jointNamesInit) {
    ms->config.jointNames.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      ms->config.jointNames[j] = this->ikRequest.response.joints[0].name[j];
    }
    ms->config.jointNamesInit = 1;
  }

  if (ms->config.currentControlMode == VELOCITY) {

    double l2Gravity = 0.0;

    myCommand.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    myCommand.command.resize(NUM_JOINTS);
    myCommand.names.resize(NUM_JOINTS);

    ros::Time theNow = ros::Time::now();
    ros::Duration howLong = theNow - ms->config.oscilStart;
    double spiralEta = 1.25;
    double rapidJointGlobalOmega[NUM_JOINTS] = {4, 0, 0, 4, 4, 4, 4};
    double rapidJointLocalOmega[NUM_JOINTS] = {.2, 0, 0, 2, 2, .2, 2};
    double rapidJointLocalBias[NUM_JOINTS] = {0, 0, 0, 0.7, 0, 0, 0};
    int rapidJointMask[NUM_JOINTS] = {1, 0, 0, 1, 1, 1, 1};
    double rapidJointScales[NUM_JOINTS] = {.10, 0, 0, 1.0, 2.0, .20, 3.1415926};


    for (int j = 0; j < NUM_JOINTS; j++) {
      myCommand.names[j] = this->ikRequest.response.joints[0].name[j];
      myCommand.command[j] = spiralEta*rapidJointScales[j]*(this->ikRequest.response.joints[0].position[j] - ms->config.trueJointPositions[j]);
    }
    {
      double tim = howLong.toSec();
      double rapidAmp1 = 0.00; //0.3 is great
      myCommand.command[4] += -rapidAmp1*rapidJointScales[4]*sin(rapidJointLocalBias[4] + (rapidJointLocalOmega[4]*rapidJointGlobalOmega[4]*tim));
      myCommand.command[3] +=  rapidAmp1*rapidJointScales[3]*cos(rapidJointLocalBias[3] + (rapidJointLocalOmega[3]*rapidJointGlobalOmega[3]*tim));

      double rapidAmp2 = 0.00;
      myCommand.command[5] += -rapidAmp2*rapidJointScales[5]*sin(rapidJointLocalBias[5] + (rapidJointLocalOmega[5]*rapidJointGlobalOmega[5]*tim));
      myCommand.command[0] +=  rapidAmp2*rapidJointScales[0]*cos(rapidJointLocalBias[0] + (rapidJointLocalOmega[0]*rapidJointGlobalOmega[0]*tim));
    }
  } else if (ms->config.currentControlMode == EEPOSITION) {
    myCommand.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    myCommand.command.resize(NUM_JOINTS);
    myCommand.names.resize(NUM_JOINTS);

    this->lastGoodIkRequest.response.joints.resize(1);
    this->lastGoodIkRequest.response.joints[0].name.resize(NUM_JOINTS);
    this->lastGoodIkRequest.response.joints[0].position.resize(NUM_JOINTS);

    ms->config.baxterConfig->currentJointPositions.response.joints.resize(1);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].name.resize(NUM_JOINTS);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position.resize(NUM_JOINTS);

    for (int j = 0; j < NUM_JOINTS; j++) {
      myCommand.names[j] = this->ikRequest.response.joints[0].name[j];
      myCommand.command[j] = this->ikRequest.response.joints[0].position[j];
      this->lastGoodIkRequest.response.joints[0].name[j] = this->ikRequest.response.joints[0].name[j];
      this->lastGoodIkRequest.response.joints[0].position[j] = this->ikRequest.response.joints[0].position[j];

      ms->config.baxterConfig->currentJointPositions.response.joints[0].name[j] = myCommand.names[j];
      ms->config.baxterConfig->currentJointPositions.response.joints[0].position[j] = myCommand.command[j];
    }
    ms->config.goodIkInitialized = 1;
  } else if (ms->config.currentControlMode == ANGLES) {
    //ms->config.currentEEPose.px = ms->config.trueEEPose.position.x;
    //ms->config.currentEEPose.py = ms->config.trueEEPose.position.y;
    //ms->config.currentEEPose.pz = ms->config.trueEEPose.position.z;
    //ms->config.currentEEPose.qx = ms->config.trueEEPose.orientation.x;
    //ms->config.currentEEPose.qy = ms->config.trueEEPose.orientation.y;
    //ms->config.currentEEPose.qz = ms->config.trueEEPose.orientation.z;
    //ms->config.currentEEPose.qw = ms->config.trueEEPose.orientation.w;

    myCommand.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    myCommand.command.resize(NUM_JOINTS);
    myCommand.names.resize(NUM_JOINTS);

    this->lastGoodIkRequest.response.joints.resize(1);
    this->lastGoodIkRequest.response.joints[0].name.resize(NUM_JOINTS);
    this->lastGoodIkRequest.response.joints[0].position.resize(NUM_JOINTS);

    ms->config.baxterConfig->currentJointPositions.response.joints.resize(1);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].name.resize(NUM_JOINTS);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position.resize(NUM_JOINTS);

    for (int j = 0; j < NUM_JOINTS; j++) {
      myCommand.names[j] = ms->config.baxterConfig->currentJointPositions.response.joints[0].name[j];
      myCommand.command[j] = ms->config.baxterConfig->currentJointPositions.response.joints[0].position[j];
      //this->lastGoodIkRequest.response.joints[0].name[j] = this->ikRequest.response.joints[0].name[j];
      //this->lastGoodIkRequest.response.joints[0].position[j] = this->ikRequest.response.joints[0].position[j];
    }
  } else {
    assert(0);
  }
  if (ms->config.publish_commands_mode) {
    std_msgs::Float64 speedCommand;
    speedCommand.data = ms->config.currentEESpeedRatio;
    int param_resend_times = 1;
    for (int r = 0; r < param_resend_times; r++) {
      ms->config.baxterConfig->joint_mover.publish(myCommand);
      ms->config.baxterConfig->moveSpeedPub.publish(speedCommand);
      
      {
        std_msgs::UInt16 thisCommand;
        thisCommand.data = ms->config.baxterConfig->sonar_led_state;
        ms->config.baxterConfig->sonar_pub.publish(thisCommand);
      }
      if (ms->config.baxterConfig->repeat_halo) {
        {
          std_msgs::Float32 thisCommand;
          thisCommand.data = ms->config.baxterConfig->red_halo_state;
          ms->config.baxterConfig->red_halo_pub.publish(thisCommand);
        }
        {
          std_msgs::Float32 thisCommand;
          thisCommand.data = ms->config.baxterConfig->green_halo_state;
          ms->config.baxterConfig->green_halo_pub.publish(thisCommand);
        }
      }
    }
  }

  ms->config.bfc++;
}


void baxterCameraInit(MachineState * ms) {

  if (0 == ms->config.left_or_right_arm.compare("left")) {
    ms->config.joint_min[0] = -1.70168;
    ms->config.joint_min[1] = -2.147;
    ms->config.joint_min[2] = -3.05418;
    ms->config.joint_min[3] = -0.05;
    ms->config.joint_min[4] = -3.059;
    ms->config.joint_min[5] = -1.5708;
    ms->config.joint_min[6] = -3.059;


    ms->config.joint_max[0] = 1.70168;
    ms->config.joint_max[1] = 1.047;
    ms->config.joint_max[2] = 3.05418;
    ms->config.joint_max[3] = 2.618;
    ms->config.joint_max[4] = 3.059;
    ms->config.joint_max[5] = 2.094;
    ms->config.joint_max[6] = 3.059;

    ms->config.backScanningPose = eePose(-0.304942, 0.703968, 0.186738,
                                         0.0, 1, 0.0, 0.0);

    ms->config.beeHome = eePose(0.334217, 0.75386, 0.0362593,
                                -0.00125253, 0.999999, -0.000146851, 0.000236656);
    
    ms->config.eepReg4 = ms->config.beeHome;
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    camera->defaultReticle = eePose(334, 100, 0.0,
                                       0.0, 0.0, 0.0, 0.0);
    camera->reticle = camera->defaultReticle;

    ms->config.crane1 = eePose(-0.0155901, 0.981296, 0.71078,
                               0.709046, -0.631526, -0.226613, -0.216967);

    double ystart = 0.1;
    double yend = 0.7;
    int numposes = 4;
    double ystep = (yend - ystart) / numposes;
    eePose pose1 = eePose(0.65, 0.0544691, -0.0582791,
                          0, 1, 0, 0);
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses.push_back(pose1);
    }
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses[i].py = ystart + i * ystep;
    }

    ms->config.ik_reset_eePose = eePose(0.334217, 0.75386, 0.0362593,
                                        -0.00125253, 0.999999, -0.000146851, 0.000236656);

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

    // left arm
    // (313, 163)

    camera->vanishingPointReticle.px = 313;
    camera->vanishingPointReticle.py = 163;
    camera->probeReticle = camera->vanishingPointReticle;

    // ATTN 16
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

    //ms->config.handingPose = {.px = 0.955119, .py = 0.0466243, .pz = 0.20442,
    //               .qx = 0.538769, .qy = -0.531224, .qz = 0.448211, .qw = -0.476063};
    ms->config.handingPose = eePose(1.0858369, 0.0495844, 0.2052459,
                                    0.5398360, -0.5294786, 0.4481372, -0.4768674);

    ms->config.eepReg3 = ms->config.handingPose;

    // ir offset
    camera->gear0offset = Eigen::Quaternionf(0.0, 0.03, 0.023, 0.0167228); // z is from TF, good for depth alignment

    ms->config.calibrationPose = eePose(0.434176, 0.633423, 0.48341,
                                        0.000177018, 1, -0.000352912, -0.000489087);
    ms->config.shrugPose = eePose(0.0354772, 1.20633, 0.150562,
                                  -0.370521, 0.381345, 0.578528, 0.618544);
  } else if (0 == ms->config.left_or_right_arm.compare("right")) {
    ms->config.joint_min[0] = -1.70168;
    ms->config.joint_min[1] = -2.147;
    ms->config.joint_min[2] = -3.05418;
    ms->config.joint_min[3] = -0.05;
    ms->config.joint_min[4] = -3.059;
    ms->config.joint_min[5] = -1.5708;
    ms->config.joint_min[6] = -3.059;


    ms->config.joint_max[0] = 1.70168;
    ms->config.joint_max[1] = 1.047;
    ms->config.joint_max[2] = 3.05418;
    ms->config.joint_max[3] = 2.618;
    ms->config.joint_max[4] = 3.059;
    ms->config.joint_max[5] = 2.094;
    ms->config.joint_max[6] = 3.059;



    ms->config.backScanningPose = eePose(-0.304942, -0.703968, 0.186738,
                                         0.0, 1, 0.0, 0.0);

    ms->config.beeHome = eePose(0.525866, -0.710611, 0.0695764,
                                -0.00122177, 0.999998, 0.00116169, -0.001101);

    ms->config.eepReg4 = ms->config.beeHome;
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    camera->defaultReticle = eePose(325, 127, 0.0,
                                       0.0, 0.0, 0.0, 0.0);
    camera->reticle = camera->defaultReticle;

    ms->config.crane1 = eePose(0.0448714, -1.04476, 0.698522,
                               0.631511, 0.68929, -0.25435, 0.247748);

    double ystart = -0.7;
    double yend = -0.1;
    int numposes = 4;
    double ystep = (yend - ystart) / numposes;
    eePose pose1 = eePose(0.65, 0.0544691, -0.0582791,
                          0, 1, 0, 0);
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses.push_back(pose1);
    }
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses[i].py = ystart + i * ystep;
    }


    ms->config.ik_reset_eePose = ms->config.beeHome;

    ms->config.currentTableZ = ms->config.rightTableZ;
    ms->config.bagTableZ = ms->config.rightTableZ;
    ms->config.counterTableZ = ms->config.rightTableZ;
    ms->config.pantryTableZ  = ms->config.rightTableZ;


    ms->config.eepReg1 = ms->config.beeHome;
    ms->config.eepReg2 = ms->config.beeHome;

    // raw fence values (from John estimating arm limits)
    // True EE Position (x,y,z): -0.329642 -0.77571 0.419954
    // True EE Position (x,y,z): 0.525236 -0.841226 0.217111

    // full workspace
    ms->config.mapSearchFenceXMin = -0.75;
    ms->config.mapSearchFenceXMax = 0.9;
    ms->config.mapSearchFenceYMin = -1.25;
    ms->config.mapSearchFenceYMax = 0.5; //-0.1;//1.25;
    ms->config.mapRejectFenceXMin = ms->config.mapSearchFenceXMin;
    ms->config.mapRejectFenceXMax = ms->config.mapSearchFenceXMax;
    ms->config.mapRejectFenceYMin = ms->config.mapSearchFenceYMin;
    ms->config.mapRejectFenceYMax = ms->config.mapSearchFenceYMax;

    ms->config.mapBackgroundXMin = ms->config.mapSearchFenceXMin - ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundXMax = ms->config.mapSearchFenceXMax + ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundYMin = ms->config.mapSearchFenceYMin - ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundYMax = ms->config.mapSearchFenceYMax + ms->config.mapBackgroundBufferMeters;

    // right arm
    camera->vanishingPointReticle.px = 313;
    camera->vanishingPointReticle.py = 185;
    camera->probeReticle = camera->vanishingPointReticle;

    // ATTN 16
    camera->heightReticles[0] = camera->defaultReticle;
    camera->heightReticles[1] = camera->defaultReticle;
    camera->heightReticles[2] = camera->defaultReticle;
    camera->heightReticles[3] = camera->defaultReticle;
    
    camera->heightReticles[3].px = 314;
    camera->heightReticles[2].px = 317;
    camera->heightReticles[1].px = 320;
    camera->heightReticles[0].px = 328;

    camera->heightReticles[3].py = 154;
    camera->heightReticles[2].py = 149;
    camera->heightReticles[1].py = 139;
    camera->heightReticles[0].py = 120;

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

    /* right arm */
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
    camera->m_x_h[0] = 1.18;
    camera->m_x_h[1] = 1.12;
    camera->m_x_h[2] = 1.09;
    camera->m_x_h[3] = 1.08;

    camera->m_y_h[0] = 1.16;
    camera->m_y_h[1] = 1.17;
    camera->m_y_h[2] = 1.16;
    camera->m_y_h[3] = 1.2;

    ms->config.handingPose = eePose(0.879307, -0.0239328, 0.223839,
                                    0.459157, 0.527586, 0.48922, 0.521049);
    ms->config.eepReg3 = ms->config.handingPose;

    // ir offset
    camera->gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment

    ms->config.calibrationPose = eePose(0.562169, -0.348055, 0.493231,
                                        0.00391311, 0.999992, -0.00128095, 8.18951e-05);
    ms->config.shrugPose = eePose(0.0558937, -1.12849, 0.132171,
                                  0.392321, 0.324823, -0.555039, 0.657652);


  } else {
    cout << "Invalid chirality: " << ms->config.left_or_right_arm << ".  Exiting." << endl;
    exit(0);
  }
  ms->config.pilotTarget = ms->config.beeHome;
  ms->config.lastGoodEEPose = ms->config.beeHome;
  ms->config.currentEEPose = ms->config.beeHome;

}


namespace ein_words {




WORD(ShakeHeadPositive)
virtual void execute(MachineState * ms)
{
  ms->config.baxterConfig->currentHeadPanCommand.target = 3.1415926/2.0;
#ifdef RETHINK_SDK_1_2_0
  ms->config.baxterConfig->currentHeadPanCommand.speed_ratio = 0.5;
#else
  ms->config.baxterConfig->currentHeadPanCommand.speed = 50;
#endif
  ms->config.baxterConfig->headPub.publish(ms->config.baxterConfig->currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadPositive)


WORD(ShakeHeadNegative)
virtual void execute(MachineState * ms)
{
  ms->config.baxterConfig->currentHeadPanCommand.target = -3.1415926/2.0;
#ifdef RETHINK_SDK_1_2_0
  ms->config.baxterConfig->currentHeadPanCommand.speed_ratio = 0.5;
#else
  ms->config.baxterConfig->currentHeadPanCommand.speed = 50;
#endif
  ms->config.baxterConfig->headPub.publish(ms->config.baxterConfig->currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadNegative)

WORD(CenterHead)
virtual void execute(MachineState * ms)
{
  ms->config.baxterConfig->currentHeadPanCommand.target = 0;
#ifdef RETHINK_SDK_1_2_0
  ms->config.baxterConfig->currentHeadPanCommand.speed_ratio = 0.5;
#else
  ms->config.baxterConfig->currentHeadPanCommand.speed = 50;
#endif
  ms->config.baxterConfig->headPub.publish(ms->config.baxterConfig->currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(CenterHead)

WORD(SetHeadPanTargetSpeed)
virtual void execute(MachineState * ms)
{
  double t_target;
  double t_speed;
  GET_NUMERIC_ARG(ms, t_speed);
  GET_NUMERIC_ARG(ms, t_target);

  cout << "setHeadPanTargetSpeed: " << t_target << " " << t_speed << endl;

  ms->config.baxterConfig->currentHeadPanCommand.target = t_target;
#ifdef RETHINK_SDK_1_2_0
  ms->config.baxterConfig->currentHeadPanCommand.speed_ratio = floor(t_speed);
#else
  ms->config.baxterConfig->currentHeadPanCommand.speed = floor(100 * t_speed);
#endif
  ms->config.baxterConfig->headPub.publish(ms->config.baxterConfig->currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(SetHeadPanTargetSpeed)

WORD(SilenceSonar)
virtual void execute(MachineState * ms)
{
  ms->config.baxterConfig->currentSonarCommand.data = 0;
  ms->config.baxterConfig->sonarPub.publish(ms->config.baxterConfig->currentSonarCommand);
}
END_WORD
REGISTER_WORD(SilenceSonar)

WORD(UnSilenceSonar)
virtual void execute(MachineState * ms)
{
  ms->config.baxterConfig->currentSonarCommand.data = 1;
  ms->config.baxterConfig->sonarPub.publish(ms->config.baxterConfig->currentSonarCommand);
}
END_WORD
REGISTER_WORD(UnSilenceSonar)

WORD(Nod)
virtual void execute(MachineState * ms)
{
  ms->config.baxterConfig->currentHeadNodCommand.data = 1;
  ms->config.baxterConfig->nodPub.publish(ms->config.baxterConfig->currentHeadNodCommand);
}
END_WORD
REGISTER_WORD(Nod)

WORD(ArmPublishJointPositionCommand)
virtual void execute(MachineState * ms) {
  // XXX needs to be an ArmPose reactive variables currentArmPose, trueArmPose
  baxter_core_msgs::JointCommand myCommand;

  if (!ms->config.jointNamesInit) {
    ms->config.jointNames.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      ms->config.jointNames[j] = ms->config.baxterConfig->ikRequest.response.joints[0].name[j];
    }
    ms->config.jointNamesInit = 1;
  }

  myCommand.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  myCommand.command.resize(NUM_JOINTS);
  myCommand.names.resize(NUM_JOINTS);

  ms->config.baxterConfig->lastGoodIkRequest.response.joints.resize(1);
  ms->config.baxterConfig->lastGoodIkRequest.response.joints[0].name.resize(NUM_JOINTS);
  ms->config.baxterConfig->lastGoodIkRequest.response.joints[0].position.resize(NUM_JOINTS);

  ms->config.baxterConfig->currentJointPositions.response.joints.resize(1);
  ms->config.baxterConfig->currentJointPositions.response.joints[0].name.resize(NUM_JOINTS);
  ms->config.baxterConfig->currentJointPositions.response.joints[0].position.resize(NUM_JOINTS);

  for (int j = 0; j < NUM_JOINTS; j++) {
    myCommand.names[j] = ms->config.baxterConfig->currentJointPositions.response.joints[0].name[j];
    myCommand.command[j] = ms->config.baxterConfig->currentJointPositions.response.joints[0].position[j];
  }

  ms->config.baxterConfig->joint_mover.publish(myCommand);
}
END_WORD
REGISTER_WORD(ArmPublishJointPositionCommand)


WORD(MoveJointsToAngles)
virtual void execute(MachineState * ms) {
  double an[NUM_JOINTS];
  for (int i = NUM_JOINTS-1; i >= 0; i--) {
    GET_NUMERIC_ARG(ms, an[i]);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i] = an[i];
  }
}
END_WORD
REGISTER_WORD(MoveJointsToAngles)

WORD(MoveJointsByAngles)
virtual void execute(MachineState * ms) {
  double an[NUM_JOINTS];
  for (int i = NUM_JOINTS-1; i >= 0; i--) {
    GET_NUMERIC_ARG(ms, an[i]);
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i] += an[i];
  }
}
END_WORD
REGISTER_WORD(MoveJointsByAngles)

WORD(PrintJointAngles)
virtual void execute(MachineState * ms) {
  cout << "currentJointPositions: " << endl;
  for (int i = 0; i < NUM_JOINTS; i++) {
    cout << ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i] << " ";
  }
  cout << "moveJointsToAngles" << endl;
}
END_WORD
REGISTER_WORD(PrintJointAngles)

WORD(PushCurrentJointAngle)
virtual void execute(MachineState * ms) {
  int jointToPush = 0; 
  GET_INT_ARG(ms, jointToPush);
  ms->pushWord(make_shared<DoubleWord>(ms->config.baxterConfig->currentJointPositions.response.joints[0].position[jointToPush]));
}
END_WORD
REGISTER_WORD(PushCurrentJointAngle)


WORD(PushCurrentJointAngles)
virtual void execute(MachineState * ms) {
  shared_ptr<CompoundWord> angles = make_shared<CompoundWord>();
  for (int i =  0; i < ms->config.baxterConfig->currentJointPositions.response.joints[0].position.size(); i++) {
    int j = ms->config.baxterConfig->currentJointPositions.response.joints[0].position.size() - 1 - i;
    angles->pushWord(make_shared<DoubleWord>(ms->config.baxterConfig->currentJointPositions.response.joints[0].position[j]));
  }
  ms->pushData(angles);
}
END_WORD
REGISTER_WORD(PushCurrentJointAngles)

WORD(CurrentJointWord)
virtual void execute(MachineState * ms) {
  armPose pose;
  for (int i = NUM_JOINTS-1; i >= 0; i--) {
    pose.joints[i] = ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i];
  }

  ms->pushWord(make_shared<ArmPoseWord>(pose));
}
END_WORD
REGISTER_WORD(CurrentJointWord)

WORD(MoveArmToPoseWord)
virtual void execute(MachineState * ms) {
  armPose pose;
  GET_ARG(ms, ArmPoseWord, pose);

  for (int i = NUM_JOINTS-1; i >= 0; i--) {
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i] = pose.joints[i];
  }
}
END_WORD
REGISTER_WORD(MoveArmToPoseWord)

WORD(SetCurrentPoseFromJoints)
virtual void execute(MachineState * ms) {

  vector<double> joint_angles(NUM_JOINTS);
  for (int i =  0; i < ms->config.baxterConfig->currentJointPositions.response.joints[0].position.size(); i++) {
    joint_angles[i] = ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i];
  }

  eePose ikfastEndPointPose;
  if (ms->config.left_or_right_arm == "left") {
    ikfastEndPointPose = ikfast_left_ein::ikfast_computeFK(ms, joint_angles);
  } else if (ms->config.left_or_right_arm == "right") {
    ikfastEndPointPose = ikfast_right_ein::ikfast_computeFK(ms, joint_angles);
  } else {
    assert(0);
  }

  eePose handFromIkFastEndPoint = {0,0,-0.0661, 0,0,0,1};

  eePose desiredHandPose = handFromIkFastEndPoint.applyAsRelativePoseTo(ikfastEndPointPose);
  eePose desiredEndPointPose = ms->config.handToRethinkEndPointTransform.applyAsRelativePoseTo(desiredHandPose);
  // XXX factor1

  ms->config.currentEEPose = desiredEndPointPose;

  cout << "setCurrentPoseFromJoints: doing it " << ms->config.currentEEPose << endl;
}
END_WORD
REGISTER_WORD(SetCurrentPoseFromJoints)

#define ARM_POSE_DELTAS(J, I, C, T, D) \
WORD(Arm ## J ## Up) \
virtual void execute(MachineState * ms) { \
  C += D; \
} \
END_WORD \
REGISTER_WORD(Arm ## J ## Up) \
\
WORD(Arm ## J ## Down) \
virtual void execute(MachineState * ms) { \
  C -= D; \
} \
END_WORD \
REGISTER_WORD(Arm ## J ## Down) \
\
WORD(Arm ## J ## By) \
virtual void execute(MachineState * ms) { \
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
  C += amount; \
} \
END_WORD \
REGISTER_WORD(Arm ## J ## By) \
WORD(Arm ## J ## To) \
virtual void execute(MachineState * ms) { \
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
  C = amount; \
} \
END_WORD \
REGISTER_WORD(Arm ## J ## To) \
WORD(Arm ## J ## Current) \
virtual void execute(MachineState * ms) { \
  ms->pushData(make_shared<DoubleWord>(C)); \
} \
END_WORD \
REGISTER_WORD(Arm ## J ## Current) \
WORD(Arm ## J ## True) \
virtual void execute(MachineState * ms) { \
  ms->pushData(make_shared<DoubleWord>(T)); \
} \
END_WORD \
REGISTER_WORD(Arm ## J ## True) \
WORD(ArmPose ## J ## Get) \
virtual void execute(MachineState * ms) { \
  armPose pose; \
  GET_ARG(ms, ArmPoseWord, pose); \
  ms->pushData(make_shared<DoubleWord>(pose.joints[I])); \
} \
END_WORD \
REGISTER_WORD(ArmPose ## J ## Get) \
WORD(ArmPose ## J ## Set) \
virtual void execute(MachineState * ms) { \
  double amount = 0.0; \
  GET_NUMERIC_ARG(ms, amount); \
\
  armPose pose; \
  GET_ARG(ms, ArmPoseWord, pose); \
\
  pose.joints[I] = amount;\
\
  ms->pushWord(make_shared<ArmPoseWord>(pose));\
} \
END_WORD \
REGISTER_WORD(ArmPose ## J ## Set) \

// 1 indexed to match aibo
ARM_POSE_DELTAS(1, 0, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[0], ms->config.trueJointPositions[0], ms->config.bDelta)
ARM_POSE_DELTAS(2, 1, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[1], ms->config.trueJointPositions[1], ms->config.bDelta)
ARM_POSE_DELTAS(3, 2, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[2], ms->config.trueJointPositions[2], ms->config.bDelta)
ARM_POSE_DELTAS(4, 3, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[3], ms->config.trueJointPositions[3], ms->config.bDelta)
ARM_POSE_DELTAS(5, 4, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[4], ms->config.trueJointPositions[4], ms->config.bDelta)
ARM_POSE_DELTAS(6, 5, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[5], ms->config.trueJointPositions[5], ms->config.bDelta)
ARM_POSE_DELTAS(7, 6, ms->config.baxterConfig->currentJointPositions.response.joints[0].position[6], ms->config.trueJointPositions[6], ms->config.bDelta)


WORD(ArmPoseToEePose)
virtual void execute(MachineState * ms) {
// ComputeFk
// XXX 
  armPose armPoseIn;
  GET_ARG(ms, ArmPoseWord, armPoseIn);

  vector<double> joint_angles(NUM_JOINTS);
  for (int i =  0; i < ms->config.baxterConfig->currentJointPositions.response.joints[0].position.size(); i++) {
    joint_angles[i] = armPoseIn.joints[i];
  }

  eePose ikfastEndPointPose;
  if (ms->config.left_or_right_arm == "left") {
    ikfastEndPointPose = ikfast_left_ein::ikfast_computeFK(ms, joint_angles);
  } else if (ms->config.left_or_right_arm == "right") {
    ikfastEndPointPose = ikfast_right_ein::ikfast_computeFK(ms, joint_angles);
  } else {
    assert(0);
  }

  eePose handFromIkFastEndPoint = {0,0,-0.0661, 0,0,0,1};

  eePose desiredHandPose = handFromIkFastEndPoint.applyAsRelativePoseTo(ikfastEndPointPose);
  eePose desiredEndPointPose = ms->config.handToRethinkEndPointTransform.applyAsRelativePoseTo(desiredHandPose);
  // XXX factor1

  ms->pushWord(std::make_shared<EePoseWord>(desiredEndPointPose));
}
END_WORD
REGISTER_WORD(ArmPoseToEePose)


WORD(EePoseToArmPose)
virtual void execute(MachineState * ms) {
// ComputeIk
// XXX 
  eePose eePoseIn;
  GET_ARG(ms, EePoseWord, eePoseIn);

  baxter_core_msgs::SolvePositionIK thisIkRequest;
  fillIkRequest(eePoseIn, &thisIkRequest);

  int ikCallResult = 0;
  queryIK(ms, &ikCallResult, &thisIkRequest);

  // XXX this maintains history in the solver
  //  not good form
  for (int i = NUM_JOINTS-1; i >= 0; i--) {
    ms->config.baxterConfig->currentJointPositions.response.joints[0].position[i] =
	  thisIkRequest.response.joints[0].position[i];
  }

  if (ikCallResult && thisIkRequest.response.isValid[0]) {
    cout << "eePoseToArmPose: got " << thisIkRequest.response.joints.size() << " solutions, using first." << endl;
    ms->pushWord( 
      std::make_shared<ArmPoseWord>(  
	armPose(
	  thisIkRequest.response.joints[0].position[0],
	  thisIkRequest.response.joints[0].position[1],
	  thisIkRequest.response.joints[0].position[2],
	  thisIkRequest.response.joints[0].position[3],
	  thisIkRequest.response.joints[0].position[4],
	  thisIkRequest.response.joints[0].position[5],
	  thisIkRequest.response.joints[0].position[6]
	)
      )
    );
  } else {
    cout << "eePoseToArmPose: ik failed..." << endl;
  }
}
END_WORD
REGISTER_WORD(EePoseToArmPose)

WORD(AnalogIOCommand)
virtual void execute(MachineState * ms)
{
  string component;
  double value;
  GET_ARG(ms, StringWord, component);
  GET_NUMERIC_ARG(ms, value);

  baxter_core_msgs::AnalogOutputCommand thisCommand;

  thisCommand.name = component;
  thisCommand.value = value;

  ms->config.baxterConfig->analog_io_pub.publish(thisCommand);
}
END_WORD
REGISTER_WORD(AnalogIOCommand)


WORD(DigitalIOCommand)
virtual void execute(MachineState * ms)
{
  string component;
  int value;
  GET_ARG(ms, StringWord, component);
  GET_ARG(ms, IntegerWord, value);

  baxter_core_msgs::DigitalOutputCommand thisCommand;

  thisCommand.name = component;
  thisCommand.value = value;

  ms->config.baxterConfig->digital_io_pub.publish(thisCommand);
}
END_WORD
REGISTER_WORD(DigitalIOCommand)

WORD(SetRedHalo)
virtual void execute(MachineState * ms)
{
  double value;
  GET_NUMERIC_ARG(ms, value);
  ms->config.baxterConfig->red_halo_state = value;

  {
    std_msgs::Float32 thisCommand;
    thisCommand.data = ms->config.baxterConfig->red_halo_state;
    ms->config.baxterConfig->red_halo_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SetRedHalo)

WORD(SetGreenHalo)
virtual void execute(MachineState * ms)
{
  double value;
  GET_NUMERIC_ARG(ms, value);
  ms->config.baxterConfig->green_halo_state = value;

  {
    std_msgs::Float32 thisCommand;
    thisCommand.data = ms->config.baxterConfig->green_halo_state;
    ms->config.baxterConfig->green_halo_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SetGreenHalo)

WORD(SetSonarLed)
virtual void execute(MachineState * ms)
{
  int value;
  GET_ARG(ms, IntegerWord, value);
  ms->config.baxterConfig->sonar_led_state = value;

  {
    std_msgs::UInt16 thisCommand;
    thisCommand.data = ms->config.baxterConfig->sonar_led_state;
    ms->config.baxterConfig->sonar_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SetSonarLed)

WORD(LightsOn)
virtual void execute(MachineState * ms)
{
  std::stringstream program;
  program << "100 setGreenHalo 100 setRedHalo 4095 setSonarLed ";
  program << "1 \"left_itb_light_inner\" digitalIOCommand 1 \"right_itb_light_inner\" digitalIOCommand 1 \"torso_left_itb_light_inner\" digitalIOCommand 1 \"torso_right_itb_light_inner\" digitalIOCommand 1 \"left_itb_light_outer\" digitalIOCommand 1 \"right_itb_light_outer\" digitalIOCommand 1 \"torso_left_itb_light_outer\" digitalIOCommand 1 \"torso_right_itb_light_outer\" digitalIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(LightsOn)

WORD(LightsOff)
virtual void execute(MachineState * ms)
{
  std::stringstream program;
  program << "0 setGreenHalo 0 setRedHalo 32768 setSonarLed ";
  program << "0 \"left_itb_light_inner\" digitalIOCommand 0 \"right_itb_light_inner\" digitalIOCommand 0 \"torso_left_itb_light_inner\" digitalIOCommand 0 \"torso_right_itb_light_inner\" digitalIOCommand 0 \"left_itb_light_outer\" digitalIOCommand 0 \"right_itb_light_outer\" digitalIOCommand 0 \"torso_left_itb_light_outer\" digitalIOCommand 0 \"torso_right_itb_light_outer\" digitalIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(LightsOff)

WORD(SwitchSonarLed)
virtual void execute(MachineState * ms)
{
  int value;
  int led;
  GET_ARG(ms, IntegerWord, led);
  GET_ARG(ms, IntegerWord, value);

  int blanked_sls = (~(1<<led)) & ms->config.baxterConfig->sonar_led_state;

  ms->config.baxterConfig->sonar_led_state = blanked_sls | (1<<15) | (value * (1<<led));

  {
    std_msgs::UInt16 thisCommand;
    thisCommand.data = ms->config.baxterConfig->sonar_led_state;
    ms->config.baxterConfig->sonar_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SwitchSonarLed)

WORD(PublishWristViewToFace)
virtual void execute(MachineState * ms) {
  Size toBecome(1024,600);
  sensor_msgs::Image msg;

  Mat toresize = ms->config.wristViewImage.clone();
  Mat topub;
  cv::resize(toresize, topub, toBecome);

  msg.header.stamp = ros::Time::now();
  msg.width = topub.cols;
  msg.height = topub.rows;
  msg.step = topub.cols * topub.elemSize();
  msg.is_bigendian = false;
  msg.encoding = sensor_msgs::image_encodings::BGR8;
  msg.data.assign(topub.data, topub.data + size_t(topub.rows * msg.step));
  ms->config.baxterConfig->face_screen_pub.publish(msg);
}
END_WORD
REGISTER_WORD(PublishWristViewToFace)

WORD(PublishImageFileToFace)
virtual void execute(MachineState * ms) {
  string imfilename_post;
  GET_STRING_ARG(ms, imfilename_post);
  
  string imfilename = "src/ein/images/" + imfilename_post;
  Mat topub = imread(imfilename);

  if (isSketchyMat(topub)) {
    CONSOLE_ERROR(ms, "publishImageFileToFace: cannot load file " << imfilename);
    ms->pushWord("pauseStackExecution");
    return;
  } else {
    sensor_msgs::Image msg;
    msg.header.stamp = ros::Time::now();
    msg.width = topub.cols;
    msg.height = topub.rows;
    msg.step = topub.cols * topub.elemSize();
    msg.is_bigendian = false;
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.data.assign(topub.data, topub.data + size_t(topub.rows * msg.step));
    ms->config.baxterConfig->face_screen_pub.publish(msg);
  }
}
END_WORD
REGISTER_WORD(PublishImageFileToFace)


WORD(BlankFace)
virtual void execute(MachineState * ms) {
  std::stringstream program;
  program << "\"black.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(BlankFace)


WORD(HappyFace)
virtual void execute(MachineState * ms) {
  std::stringstream program;
  program << "\"ursula_yes.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(HappyFace)


WORD(SadFace)
virtual void execute(MachineState * ms) {
  std::stringstream program;
  program << "\"ursula_no.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(SadFace)


WORD(NeutralFace)
virtual void execute(MachineState * ms) {
  std::stringstream program;
  program << "\"ursula_neutral.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(NeutralFace)


WORD(TorsoFanOn)
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("100 \"torso_fan\" analogIOCommand");
}
END_WORD
REGISTER_WORD(TorsoFanOn)

WORD(TorsoFanOff)
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("1 \"torso_fan\" analogIOCommand");
}
END_WORD
REGISTER_WORD(TorsoFanOff)

WORD(TorsoFanAuto)
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("0 \"torso_fan\" analogIOCommand");
}
END_WORD
REGISTER_WORD(TorsoFanAuto)


WORD(SetTorsoFanLevel)
virtual void execute(MachineState * ms)
{
  double value;
  GET_NUMERIC_ARG(ms, value);
  std::stringstream program;
  program << value << " \"torso_fan\" analogIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(SetTorsoFanLevel)



WORD(SetGripperMovingForce)
  virtual string description() {
  return "Set the moving force of the gripper.  Takes an integer between 0-100. ";
}
virtual void execute(MachineState * ms) {
// velocity - Velocity at which a position move will execute 
// moving_force - Force threshold at which a move will stop 
// holding_force - Force at which a grasp will continue holding 
// dead_zone - Position deadband within move considered successful 
// ALL PARAMETERS (0-100) 

  int amount = 0; 
  GET_ARG(ms,IntegerWord,amount);

  amount = min(max(0,amount),100);

  cout << "setGripperMovingForce, amount: " << amount << endl;

  char buf[1024]; sprintf(buf, "{\"moving_force\": %d.0}", amount);
  string argString(buf);

  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_CONFIGURE;
  command.args = argString.c_str();
  command.id = 65538;
  ms->config.baxterConfig->gripperPub.publish(command);
}
END_WORD
REGISTER_WORD(SetGripperMovingForce)

WORD(CloseGripper)
CODE('j')
virtual void execute(MachineState * ms) {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 0.0}";

  // command id depends on model of gripper
  // standard
  command.id = 65538;
  // legacy
  //command.id = 65664;

  ms->config.baxterConfig->gripperPub.publish(command);
}
END_WORD
REGISTER_WORD(CloseGripper)

WORD(OpenGripper)
CODE('k')
virtual void execute(MachineState * ms) {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 100.0}";

  // command id depends on model of gripper
  // standard
  command.id = 65538;
  // legacy
  //command.id = 65664;

  ms->config.baxterConfig->gripperPub.publish(command);
  ms->config.lastMeasuredClosed = ms->config.gripperPosition;
}
END_WORD
REGISTER_WORD(OpenGripper)

WORD(OpenGripperInt)
virtual void execute(MachineState * ms) {
  cout << "openGripperInt: ";

  int amount = 0; 
  GET_ARG(ms,IntegerWord,amount);

  amount = min(max(0,amount),100);

  char buf[1024]; sprintf(buf, "{\"position\": %d.0}", amount);
  string argString(buf);

  cout << "opening gripper to " << amount << endl;

  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = argString.c_str();

  // command id depends on model of gripper
  // standard
  command.id = 65538;
  // legacy
  //command.id = 65664;

  ms->config.baxterConfig->gripperPub.publish(command);
  ms->config.lastMeasuredClosed = ms->config.gripperPosition;
}
END_WORD
REGISTER_WORD(OpenGripperInt)



WORD(SetStiffness)
virtual void execute(MachineState * ms)
{
/*
Don't use this code, if stiff == 1 the robot flails dangerously...
*/
  int stiff = 0;
  // this is a safety value, do not go below 50. have e-stop ready.
  stiff = max(50, stiff);

  GET_ARG(ms, IntegerWord, stiff);
  ms->config.baxterConfig->currentStiffnessCommand.data = stiff;
  ms->config.baxterConfig->stiffPub.publish(ms->config.baxterConfig->currentStiffnessCommand);
}
END_WORD
REGISTER_WORD(SetStiffness)

WORD(MoveCropToProperValueNoUpdate)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  cout << "Setting exposure " << camera->cameraExposure << " gain: " << camera->cameraGain;
  cout << " wbr: " << camera->cameraWhiteBalanceRed << " wbg: " << camera->cameraWhiteBalanceGreen << " wbb: " << camera->cameraWhiteBalanceBlue << endl;
  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(7);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = camera->cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = camera->cropUpperLeftCorner.py;
  ocMessage.request.settings.controls[2].id = 100;
  ocMessage.request.settings.controls[2].value = camera->cameraExposure;
  ocMessage.request.settings.controls[3].id = 101;
  ocMessage.request.settings.controls[3].value = camera->cameraGain;
  ocMessage.request.settings.controls[4].id = 102;
  ocMessage.request.settings.controls[4].value = camera->cameraWhiteBalanceRed;
  ocMessage.request.settings.controls[5].id = 103;
  ocMessage.request.settings.controls[5].value = camera->cameraWhiteBalanceGreen;
  ocMessage.request.settings.controls[6].id = 104;
  ocMessage.request.settings.controls[6].value = camera->cameraWhiteBalanceBlue;

  int testResult = ms->config.baxterConfig->cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)


WORD(AssumeAny3dGrasp)
virtual void execute(MachineState * ms) {
  double p_backoffDistance = 0.10;

  for (int tc = 0; tc < ms->config.class3dGrasps[ms->config.targetClass].size(); tc++) {
    eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][tc].grasp_pose;  

    eePose thisBase = ms->config.lastLockedPose;
    thisBase.pz = -ms->config.currentTableZ;

    cout << "assumeAny3dGrasp, tc: " << tc << endl;

    // this order is important because quaternion multiplication is not commutative
    //ms->config.currentEEPose = ms->config.currentEEPose.plusP(ms->config.currentEEPose.applyQTo(toApply));
    //ms->config.currentEEPose = ms->config.currentEEPose.multQ(toApply);
    eePose graspPose = toApply.applyAsRelativePoseTo(thisBase);

    int increments = floor(p_backoffDistance / GRID_COARSE); 
    Vector3d localUnitX;
    Vector3d localUnitY;
    Vector3d localUnitZ;
    fillLocalUnitBasis(graspPose, &localUnitX, &localUnitY, &localUnitZ);
    eePose retractedGraspPose = eePoseMinus(graspPose, p_backoffDistance * localUnitZ);

    int ikResultPassedBoth = 1;
    {
      cout << "Checking IK for 3D grasp number " << tc << ", "; 
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = graspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }
    {
      cout << "Checking IK for 3D pre-grasp number " << tc << ", ";
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = retractedGraspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }

    if (ikResultPassedBoth) {
      ms->config.current3dGraspIndex = tc;

      cout << "Grasp and pre-grasp both passed, accepting." << endl;

      ms->config.currentEEPose = retractedGraspPose;
      ms->config.lastPickPose = graspPose;
      int tbb = ms->config.targetBlueBox;
      if (tbb < ms->config.blueBoxMemories.size()) {
	ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
      } else {
	assert(0);
      }

      if (ms->config.snapToFlushGrasp) {
	// using twist and effort
	ms->pushWord("closeGripper");
	ms->pushWord("pressUntilEffortCombo");

	ms->pushWord("setEffortThresh");
	ms->pushWord("7.0");

	ms->pushWord("setSpeed");
	ms->pushWord("0.03");

	ms->pushWord("pressUntilEffortInit");
	ms->pushWord("comeToStop");
	ms->pushWord("setMovementStateToMoving");
	ms->pushWord("comeToStop");
	ms->pushWord("waitUntilAtCurrentPosition");

	ms->pushWord("setSpeed");
	ms->pushWord("0.05");

	ms->pushWord("setGridSizeCoarse");
      } else {
	ms->pushWord("comeToStop"); 
	ms->pushWord("waitUntilAtCurrentPosition"); 

	ms->pushCopies("localZUp", increments);
	ms->pushWord("setGridSizeCoarse");
	ms->pushWord("approachSpeed");
      }

      ms->pushWord("waitUntilAtCurrentPosition"); 
      return;
    }
  }

  cout << "No 3D grasps were feasible. Continuing." << endl;
}
END_WORD
REGISTER_WORD(AssumeAny3dGrasp)


WORD(UnFixCameraLightingNoUpdate)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = camera->cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = camera->cropUpperLeftCorner.py;
  int testResult = ms->config.baxterConfig->cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(UnFixCameraLightingNoUpdate)

CONFIG_GETTER_INT(RepeatHalo, ms->config.baxterConfig->repeat_halo)
CONFIG_SETTER_INT(SetRepeatHalo, ms->config.baxterConfig->repeat_halo)

}
