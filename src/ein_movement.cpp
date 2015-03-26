WORD(AssumeDeliveryPose)
virtual void execute() {
  double oldz = currentEEPose.pz;
  currentEEPose = deliveryPoses[currentDeliveryPose];
  currentEEPose.pz = oldz;
  currentDeliveryPose = (currentDeliveryPose + 1) % deliveryPoses.size();
  pushWord("waitUntilAtCurrentPosition");
}
END_WORD

WORD(WaitUntilAtCurrentPosition)
CODE(131154)    // capslock + r
virtual void execute() {
  waitUntilAtCurrentPositionCounter = 0;
  double dx = (currentEEPose.px - trueEEPose.position.x);
  double dy = (currentEEPose.py - trueEEPose.position.y);
  double dz = (currentEEPose.pz - trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;
  
  double qx = (fabs(currentEEPose.qx) - fabs(trueEEPose.orientation.x));
  double qy = (fabs(currentEEPose.qy) - fabs(trueEEPose.orientation.y));
  double qz = (fabs(currentEEPose.qz) - fabs(trueEEPose.orientation.z));
  double qw = (fabs(currentEEPose.qw) - fabs(trueEEPose.orientation.w));
  double angleDistance = qx*qx + qy*qy + qz*qz + qw*qw;
  
  if ((distance > w1GoThresh*w1GoThresh) || (angleDistance > w1AngleThresh*w1AngleThresh)) {
    pushWord("waitUntilAtCurrentPositionB"); 
    endThisStackCollapse = 1;
    shouldIDoIK = 1;
  } else {
    endThisStackCollapse = endCollapse;
  }
}
END_WORD

WORD(WaitUntilAtCurrentPositionB)
virtual void execute() {
  double dx = (currentEEPose.px - trueEEPose.position.x);
  double dy = (currentEEPose.py - trueEEPose.position.y);
  double dz = (currentEEPose.pz - trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;
  
  double qx = (fabs(currentEEPose.qx) - fabs(trueEEPose.orientation.x));
  double qy = (fabs(currentEEPose.qy) - fabs(trueEEPose.orientation.y));
  double qz = (fabs(currentEEPose.qz) - fabs(trueEEPose.orientation.z));
  double qw = (fabs(currentEEPose.qw) - fabs(trueEEPose.orientation.w));
  double angleDistance = qx*qx + qy*qy + qz*qz + qw*qw;
  
  if (waitUntilAtCurrentPositionCounter < waitUntilAtCurrentPositionCounterTimeout) {
    waitUntilAtCurrentPositionCounter++;
    if ((distance > w1GoThresh*w1GoThresh) || (angleDistance > w1AngleThresh*w1AngleThresh)) {
      pushWord("waitUntilAtCurrentPositionB"); 
      endThisStackCollapse = 1;
      shouldIDoIK = 1;
    } else {
      endThisStackCollapse = endCollapse;
    }
  } else {
    cout << "Warning: waitUntilAtCurrentPosition timed out, moving on." << endl;
    endThisStackCollapse = endCollapse;
  }
}
END_WORD

WORD(WaitUntilGripperNotMoving)
virtual void execute() {
  waitUntilGripperNotMovingCounter = 0;
  lastGripperCallbackRequest = ros::Time::now();
  pushWord("waitUntilGripperNotMovingB"); 
  endThisStackCollapse = 1;
}
END_WORD

WORD(WaitUntilGripperNotMovingB)
virtual void execute() {
  if (lastGripperCallbackRequest >= lastGripperCallbackReceived) {
    pushWord("waitUntilGripperNotMovingB"); 
  } else {
    lastGripperCallbackRequest = ros::Time::now();
    if (waitUntilGripperNotMovingCounter < waitUntilGripperNotMovingTimeout) {
      if (gripperMoving) {
	waitUntilGripperNotMovingCounter++;
	pushWord("waitUntilGripperNotMovingB"); 
      } else {
	pushWord("waitUntilGripperNotMovingC"); 
	waitUntilGripperNotMovingStamp = ros::Time::now();
	waitUntilGripperNotMovingCounter = 0;
      }
    } else {
      cout << "Warning: waitUntilGripperNotMovingB timed out, moving on." << endl;
    }
  }
  endThisStackCollapse = 1;
}
END_WORD

WORD(WaitUntilGripperNotMovingC)
virtual void execute() {
// waits until gripper has not been moving for gripperNotMovingConfirmTime
  if (lastGripperCallbackRequest >= lastGripperCallbackReceived) {
    pushWord("waitUntilGripperNotMovingC"); 
  } else {
    lastGripperCallbackRequest = ros::Time::now();
    if (waitUntilGripperNotMovingCounter < waitUntilGripperNotMovingTimeout) {
      ros::Duration deltaSinceUpdate = gripperLastUpdated - waitUntilGripperNotMovingStamp;
      if (deltaSinceUpdate.toSec() <= gripperNotMovingConfirmTime) {
	waitUntilGripperNotMovingCounter++;
	pushWord("waitUntilGripperNotMovingC"); 
      }
    } else {
      cout << "Warning: waitUntilGripperNotMovingC timed out, moving on." << endl;
    }
  }
  endThisStackCollapse = 1;
}
END_WORD

WORD(PerturbPosition)
CODE(1048623)     // numlock + /
virtual void execute() {
  double noX = perturbScale * ((drand48() - 0.5) * 2.0);
  double noY = perturbScale * ((drand48() - 0.5) * 2.0);
  double noTheta = 3.1415926 * ((drand48() - 0.5) * 2.0);
  
  currentEEPose.px += noX;
  currentEEPose.py += noY;
  currentEEPose.oz += noTheta;
}
END_WORD

WORD(OYDown)
CODE('w'+65504) 
virtual void execute() {
  currentEEPose.oy -= bDelta;
}
END_WORD

WORD(OYUp)
CODE('s'+65504) 
virtual void execute() {
  currentEEPose.oy += bDelta;
}
END_WORD

WORD(OZDown)
CODE('q'+65504) 
virtual void execute() {
  currentEEPose.oz -= bDelta;
}
END_WORD

WORD(OZUp)
CODE('e'+65504) 
virtual void execute() {
  currentEEPose.oz += bDelta;
}
END_WORD

WORD(OXDown)
CODE('a'+65504) 
virtual void execute() {
  currentEEPose.ox -= bDelta;
}
END_WORD


WORD(OXUp)
CODE('d'+65504) 
virtual void execute() {
  currentEEPose.ox += bDelta;
}
END_WORD

WORD(SaveRegister1)
CODE(65568+1) // ! 
virtual void execute() {
  eepReg1 = currentEEPose;
}
END_WORD

WORD(SaveRegister2)
CODE(65600) // @
virtual void execute() {
  eepReg2 = currentEEPose;
}
END_WORD

WORD(SaveRegister3)
CODE(65568+3) // # 
virtual void execute() {
  eepReg3 = currentEEPose;
}
END_WORD

WORD(SaveRegister4)
CODE( 65568+4) // $ 
virtual void execute() {
  eepReg4 = currentEEPose;
}
END_WORD

WORD(MoveToRegister1)
CODE('1') 
virtual void execute() {
  currentEEPose = eepReg1;
}
END_WORD

WORD(MoveToRegister2)
CODE('2') 
virtual void execute() {
  currentEEPose = eepReg2;
}
END_WORD

WORD(MoveToRegister3)
CODE('3') 
virtual void execute() {
  currentEEPose = eepReg3;
}
END_WORD

WORD(MoveToRegister4)
CODE('4') 
virtual void execute() {
  currentEEPose = eepReg4;
}
END_WORD

WORD(MoveToRegister5)
CODE('5') 
virtual void execute() {
  currentEEPose = eepReg5;
}
END_WORD


WORD(MoveToRegister6)
CODE('6') 
virtual void execute() {
  currentEEPose = eepReg6;
}
END_WORD


WORD(XDown)
CODE('q') 
virtual void execute() {
  currentEEPose.px -= bDelta;
}
END_WORD


WORD(XUp)
CODE('e') 
virtual void execute() {
  currentEEPose.px += bDelta;
}
END_WORD

WORD(YDown)
CODE('a') 
virtual void execute() {
  currentEEPose.py -= bDelta;
}
END_WORD


WORD(YUp)
CODE('d') 
virtual void execute() {
  currentEEPose.py += bDelta;
}
END_WORD


WORD(ZUp)
CODE('w')
virtual void execute()
{
  currentEEPose.pz += bDelta;
}
END_WORD

WORD(ZDown)
CODE('s')
virtual void execute()
{
    currentEEPose.pz -= bDelta;
}
END_WORD

WORD(SetGripperThresh)
CODE(1179713)     // capslock + numlock + a
virtual void execute() {
  gripperThresh = lastMeasuredClosed + lastMeasuredBias;
  cout << "lastMeasuredClosed: " << lastMeasuredClosed << " lastMeasuredBias: " << lastMeasuredBias << endl;
  cout << "gripperThresh = " << gripperThresh << endl;
}
END_WORD

WORD(CalibrateGripper)
CODE('i') 
virtual void execute() {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
  command.id = 65538;
  gripperPub.publish(command);
}
END_WORD

WORD(CloseGripper)
CODE('j')
virtual void execute() {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 0.0}";
  command.id = 65538;
  gripperPub.publish(command);
}
END_WORD

WORD(OpenGripper)
CODE('k')
virtual void execute() {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 100.0}";
  command.id = 65538;
  gripperPub.publish(command);
  lastMeasuredClosed = gripperPosition;
}
END_WORD



WORD(SetMovementSpeedNowThatsFast)
CODE(1114193)    // numlock + Q
virtual void execute() {
  bDelta = NOW_THATS_FAST;
}
END_WORD

WORD(SetMovementSpeedMoveEvenFaster)
CODE(1114199)     // numlock + W
virtual void execute() {
  bDelta = MOVE_EVEN_FASTER;
}
END_WORD


WORD(SetMovementSpeedMoveFaster)
CODE(1114181)  // numlock + E
virtual void execute() {
  bDelta = MOVE_FASTER;
}
END_WORD

WORD(SetMovementSpeedMoveFast)
CODE(1048674)     // numlock + b
virtual void execute()  {
  bDelta = MOVE_FAST;
}
END_WORD

WORD(SetMovementSpeedMoveMedium)
CODE(1048686)   // numlock + n
virtual void execute() {
  bDelta = MOVE_MEDIUM;
}
END_WORD

WORD(SetMovementSpeedMoveSlow)
CODE(1114190) // numlock + N
virtual void execute() {
  bDelta = MOVE_SLOW;
}
END_WORD

WORD(SetMovementSpeedMoveVerySlow)
CODE(1114178) // numlock + B
virtual void execute() {
	bDelta = MOVE_VERY_SLOW;
}
END_WORD

WORD(ChangeToHeight0)
CODE(1245217) // capslock + numlock + !
virtual void execute() {
  currentThompsonHeightIdx = 0;
  currentThompsonHeight = convertHeightIdxToGlobalZ(currentThompsonHeightIdx);
  currentEEPose.pz = currentThompsonHeight;
  // ATTN 23
  reticle = vanishingPointReticle;
  //reticle = heightReticles[currentThompsonHeightIdx];
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD

WORD(ChangeToHeight1)
CODE(1245248)     // capslock + numlock + @
virtual void execute() {
  currentThompsonHeightIdx = 1;
  currentThompsonHeight = convertHeightIdxToGlobalZ(currentThompsonHeightIdx);
  currentEEPose.pz = currentThompsonHeight;
  // ATTN 23
  reticle = vanishingPointReticle;
  //reticle = heightReticles[currentThompsonHeightIdx];
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD

WORD(ChangeToHeight2)
CODE(1245219)  // capslock + numlock + #
virtual void execute()  {
  currentThompsonHeightIdx = 2;
  currentThompsonHeight = convertHeightIdxToGlobalZ(currentThompsonHeightIdx);
  currentEEPose.pz = currentThompsonHeight;
  // ATTN 23
  reticle = vanishingPointReticle;
  //reticle = heightReticles[currentThompsonHeightIdx];
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD

WORD(ChangeToHeight3)
CODE(1245220) // capslock + numlock + $
virtual void execute() {
  currentThompsonHeightIdx = 3;
  currentThompsonHeight = convertHeightIdxToGlobalZ(currentThompsonHeightIdx);
  currentEEPose.pz = currentThompsonHeight;
  // ATTN 23
  reticle = vanishingPointReticle;
  //reticle = heightReticles[currentThompsonHeightIdx];
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD

WORD(HundredthImpulse)
virtual void execute() {
  currentEESpeedRatio = 0.01;
}
END_WORD

WORD(TenthImpulse)
virtual void execute() {
  currentEESpeedRatio = 0.1;
}
END_WORD

WORD(QuarterImpulse)
virtual void execute() {
  currentEESpeedRatio = 0.25;
}
END_WORD

WORD(HalfImpulse)
virtual void execute() {
  currentEESpeedRatio = 0.5;
}
END_WORD

WORD(FullImpulse)
virtual void execute() {
  currentEESpeedRatio = 1.0;
}
END_WORD

WORD(CruisingSpeed)
virtual void execute() {
  //w1GoThresh = 0.40;
  //currentEESpeedRatio = 0.75;
  currentEESpeedRatio = 1.0;
}
END_WORD

WORD(ApproachSpeed)
virtual void execute() {
  //w1GoThresh = 0.01;
  currentEESpeedRatio = 0.035;//0.07;//0.05;
}
END_WORD

WORD(DepartureSpeed)
virtual void execute() {
  //w1GoThresh = 0.05;
  currentEESpeedRatio = 0.5;
}
END_WORD

WORD(ResetW1ThreshToDefault)
virtual void execute() {
  w1GoThresh = 0.03;
}
END_WORD

WORD(RasterScanningSpeed)
virtual void execute() {
  //w1GoThresh = 0.05;
  currentEESpeedRatio = 0.02;
}
END_WORD

WORD(Hover)
virtual void execute() {
  lastHoverTrueEEPoseEEPose = trueEEPoseEEPose;
  pushWord("hoverA");
  endThisStackCollapse = 1;
  shouldIDoIK = 1;
  lastHoverRequest = ros::Time::now();
  lastJointCallbackRequest = lastHoverRequest;
}
END_WORD

WORD(HoverA)
virtual void execute() {
  if (lastJointCallbackRequest >= lastJointCallbackReceived) {
    pushWord("hoverA");
    cout << "hoverA waiting for jointCallback." << endl;
    endThisStackCollapse = 1;
  } else {
    double dx = (lastHoverTrueEEPoseEEPose.px - trueEEPoseEEPose.px);
    double dy = (lastHoverTrueEEPoseEEPose.py - trueEEPoseEEPose.py);
    double dz = (lastHoverTrueEEPoseEEPose.pz - trueEEPoseEEPose.pz);
    double distance = dx*dx + dy*dy + dz*dz;
    
    double qx = (fabs(lastHoverTrueEEPoseEEPose.qx) - fabs(trueEEPoseEEPose.qx));
    double qy = (fabs(lastHoverTrueEEPoseEEPose.qy) - fabs(trueEEPoseEEPose.qy));
    double qz = (fabs(lastHoverTrueEEPoseEEPose.qz) - fabs(trueEEPoseEEPose.qz));
    double qw = (fabs(lastHoverTrueEEPoseEEPose.qw) - fabs(trueEEPoseEEPose.qw));
    double angleDistance = qx*qx + qy*qy + qz*qz + qw*qw;
  
    if ( ros::Time::now() - lastHoverRequest < ros::Duration(hoverTimeout) ) {
      if ((distance > hoverGoThresh*hoverGoThresh) || (angleDistance > hoverAngleThresh*hoverAngleThresh)) {
	pushWord("hoverA"); 
	endThisStackCollapse = 1;
	shouldIDoIK = 1;
	cout << "hoverA distance requirement not met, distance angleDistance: " << distance << " " << angleDistance << endl;
      } else {
	endThisStackCollapse = endCollapse;
      }
    } else {
      cout << "Warning: hover timed out, moving on." << endl;
      endThisStackCollapse = endCollapse;
    }
  }
}
END_WORD

WORD(SpawnTargetClassAtEndEffector)
virtual void execute() {
  cout << "SpawnTargetClassAtEndEffector called." << endl;
  if (targetClass < 0) {
    cout << "Not spawning because targetClass is " << targetClass << endl;
    return;
  }

  BoxMemory box;
  box.bTop.x = vanishingPointReticle.px-simulatedObjectHalfWidthPixels;
  box.bTop.y = vanishingPointReticle.py-simulatedObjectHalfWidthPixels;
  box.bBot.x = vanishingPointReticle.px+simulatedObjectHalfWidthPixels;
  box.bBot.y = vanishingPointReticle.py+simulatedObjectHalfWidthPixels;
  box.cameraPose = currentEEPose;
  box.top = pixelToGlobalEEPose(box.bTop.x, box.bTop.y, trueEEPose.position.z + currentTableZ);
  box.bot = pixelToGlobalEEPose(box.bBot.x, box.bBot.y, trueEEPose.position.z + currentTableZ);
  box.centroid.px = (box.top.px + box.bot.px) * 0.5;
  box.centroid.py = (box.top.py + box.bot.py) * 0.5;
  box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
  box.cameraTime = ros::Time::now();
  box.labeledClassIndex = targetClass;
  
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (SIMULATED) {
    mapBox(box);
    vector<BoxMemory> newMemories;
    for (int i = 0; i < blueBoxMemories.size(); i++) {
      newMemories.push_back(blueBoxMemories[i]);
    }
    newMemories.push_back(box);
    blueBoxMemories = newMemories;
  }
}

END_WORD




