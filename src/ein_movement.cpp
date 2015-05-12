WORD(AssumeDeliveryPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double oldz = currentEEPose.pz;
  currentEEPose = deliveryPoses[currentDeliveryPose];
  currentEEPose.pz = oldz;
  currentDeliveryPose = (currentDeliveryPose + 1) % deliveryPoses.size();
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeDeliveryPose)

WORD(WaitUntilAtCurrentPosition)
CODE(131154)    // capslock + r
virtual void execute(std::shared_ptr<MachineState> ms) {

  ms->config.currentMovementState = MOVING;
  lastTrueEEPoseEEPose = trueEEPoseEEPose;
  lastMovementStateSet = ros::Time::now();

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
    ms->pushWord("waitUntilAtCurrentPositionB"); 
    endThisStackCollapse = 1;
    shouldIDoIK = 1;
  } else {
    endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(WaitUntilAtCurrentPosition)

WORD(WaitUntilAtCurrentPositionB)
virtual void execute(std::shared_ptr<MachineState> ms) {

  if ( (ms->config.currentMovementState == STOPPED) ||
       (ms->config.currentMovementState == BLOCKED) ) {

    if (ms->config.currentMovementState == STOPPED) {
      cout << "Warning: waitUntilAtCurrentPosition ms->config.currentMovementState = STOPPED, moving on." << endl;
      endThisStackCollapse = endCollapse;
    }
    if (ms->config.currentMovementState == BLOCKED) {
      cout << "Warning: waitUntilAtCurrentPosition ms->config.currentMovementState = BLOCKED, moving on." << endl;
      endThisStackCollapse = endCollapse;
    }
    
    currentEEPose.pz = trueEEPose.position.z + 0.001;
    cout << "  backing up just a little to dislodge, then waiting again." << endl;

    ms->pushWord("waitUntilAtCurrentPosition"); 
    return;
  }

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
      ms->pushWord("waitUntilAtCurrentPositionB"); 
      endThisStackCollapse = 1;
      shouldIDoIK = 1;
    } else {
      endThisStackCollapse = endCollapse;
    }
  } else {
    cout << "Warning: waitUntilAtCurrentPosition timed out, moving on." << endl;
    endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(WaitUntilAtCurrentPositionB)

WORD(WaitUntilGripperNotMoving)
virtual void execute(std::shared_ptr<MachineState> ms) {
  waitUntilGripperNotMovingCounter = 0;
  lastGripperCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilGripperNotMovingB"); 
  endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilGripperNotMoving)

WORD(WaitUntilGripperNotMovingB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (lastGripperCallbackRequest >= lastGripperCallbackReceived) {
    ms->pushWord("waitUntilGripperNotMovingB"); 
  } else {
    lastGripperCallbackRequest = ros::Time::now();
    if (waitUntilGripperNotMovingCounter < waitUntilGripperNotMovingTimeout) {
      if (gripperMoving) {
	waitUntilGripperNotMovingCounter++;
	ms->pushWord("waitUntilGripperNotMovingB"); 
      } else {
	ms->pushWord("waitUntilGripperNotMovingC"); 
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
REGISTER_WORD(WaitUntilGripperNotMovingB)

WORD(WaitUntilGripperNotMovingC)
virtual void execute(std::shared_ptr<MachineState> ms) {
// waits until gripper has not been moving for gripperNotMovingConfirmTime
  if (lastGripperCallbackRequest >= lastGripperCallbackReceived) {
    ms->pushWord("waitUntilGripperNotMovingC"); 
  } else {
    lastGripperCallbackRequest = ros::Time::now();
    if (waitUntilGripperNotMovingCounter < waitUntilGripperNotMovingTimeout) {
      ros::Duration deltaSinceUpdate = gripperLastUpdated - waitUntilGripperNotMovingStamp;
      if (deltaSinceUpdate.toSec() <= gripperNotMovingConfirmTime) {
	waitUntilGripperNotMovingCounter++;
	ms->pushWord("waitUntilGripperNotMovingC"); 
      }
    } else {
      cout << "Warning: waitUntilGripperNotMovingC timed out, moving on." << endl;
    }
  }
  endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilGripperNotMovingC)

WORD(PerturbPosition)
CODE(1048623)     // numlock + /
virtual void execute(std::shared_ptr<MachineState> ms) {
  double noX = perturbScale * ((drand48() - 0.5) * 2.0);
  double noY = perturbScale * ((drand48() - 0.5) * 2.0);
  double noTheta = 3.1415926 * ((drand48() - 0.5) * 2.0);
  
  currentEEPose.px += noX;
  currentEEPose.py += noY;

  currentEEDeltaRPY.pz += noTheta;
}
END_WORD
REGISTER_WORD(PerturbPosition)

WORD(OYDown)
CODE('w'+65504) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEDeltaRPY.py -= bDelta;
}
END_WORD
REGISTER_WORD(OYDown)

WORD(OYUp)
CODE('s'+65504) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEDeltaRPY.py += bDelta;
}
END_WORD
REGISTER_WORD(OYUp)

WORD(OZDown)
CODE('q'+65504) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEDeltaRPY.pz -= bDelta;
}
END_WORD
REGISTER_WORD(OZDown)

WORD(OZUp)
CODE('e'+65504) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Changing pose. " << endl;
  currentEEDeltaRPY.pz += bDelta;
}
END_WORD
REGISTER_WORD(OZUp)

WORD(OXDown)
CODE('a'+65504) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEDeltaRPY.px -= bDelta;
}
END_WORD
REGISTER_WORD(OXDown)

WORD(OXUp)
CODE('d'+65504) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEDeltaRPY.px += bDelta;
}
END_WORD
REGISTER_WORD(OXUp)


WORD(SaveRegister1)
CODE(65568+1) // ! 
virtual void execute(std::shared_ptr<MachineState> ms) {
  eepReg1 = currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister1)

WORD(SaveRegister2)
CODE(65600) // @
virtual void execute(std::shared_ptr<MachineState> ms) {
  eepReg2 = currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister2)

WORD(SaveRegister3)
CODE(65568+3) // # 
virtual void execute(std::shared_ptr<MachineState> ms) {
  eepReg3 = currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister3)

WORD(SaveRegister4)
CODE( 65568+4) // $ 
virtual void execute(std::shared_ptr<MachineState> ms) {
  eepReg4 = currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister4)

WORD(MoveToRegister1)
CODE('1') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = eepReg1;
}
END_WORD
REGISTER_WORD(MoveToRegister1)

WORD(MoveToRegister2)
CODE('2') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = eepReg2;
}
END_WORD
REGISTER_WORD(MoveToRegister2)

WORD(MoveToRegister3)
CODE('3') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = eepReg3;
}
END_WORD
REGISTER_WORD(MoveToRegister3)

WORD(MoveToRegister4)
CODE('4') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = eepReg4;
}
END_WORD
REGISTER_WORD(MoveToRegister4)

WORD(MoveToRegister5)
CODE('5') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = eepReg5;
}
END_WORD
REGISTER_WORD(MoveToRegister5)


WORD(MoveToRegister6)
CODE('6') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = eepReg6;
}
END_WORD
REGISTER_WORD(MoveToRegister6)

WORD(LocalXDown)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(trueEEPoseEEPose, &localUnitX, &localUnitY, &localUnitZ);
  currentEEPose = currentEEPose.minusP(bDelta * localUnitX);
}
END_WORD
REGISTER_WORD(LocalXDown)


WORD(LocalXUp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(trueEEPoseEEPose, &localUnitX, &localUnitY, &localUnitZ);
  currentEEPose = currentEEPose.plusP(bDelta * localUnitX);
}
END_WORD
REGISTER_WORD(LocalXUp)

WORD(LocalYDown)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(trueEEPoseEEPose, &localUnitX, &localUnitY, &localUnitZ);
  currentEEPose = currentEEPose.minusP(bDelta * localUnitY);
}
END_WORD
REGISTER_WORD(LocalYDown)


WORD(LocalYUp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(trueEEPoseEEPose, &localUnitX, &localUnitY, &localUnitZ);
  currentEEPose = currentEEPose.plusP(bDelta * localUnitY);
}
END_WORD
REGISTER_WORD(LocalYUp)


WORD(LocalZUp)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(trueEEPoseEEPose, &localUnitX, &localUnitY, &localUnitZ);
  currentEEPose = currentEEPose.plusP(bDelta * localUnitZ);
}
END_WORD
REGISTER_WORD(LocalZUp)

WORD(LocalZDown)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(trueEEPoseEEPose, &localUnitX, &localUnitY, &localUnitZ);
  currentEEPose = currentEEPose.minusP(bDelta * localUnitZ);
}
END_WORD
REGISTER_WORD(LocalZDown)

WORD(XDown)
CODE('q') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.px -= bDelta;
}
END_WORD
REGISTER_WORD(XDown)


WORD(XUp)
CODE('e') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.px += bDelta;
}
END_WORD
REGISTER_WORD(XUp)

WORD(YDown)
CODE('a') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.py -= bDelta;
}
END_WORD
REGISTER_WORD(YDown)


WORD(YUp)
CODE('d') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.py += bDelta;
}
END_WORD
REGISTER_WORD(YUp)


WORD(ZUp)
CODE('w')
virtual void execute(std::shared_ptr<MachineState> ms)
{
  currentEEPose.pz += bDelta;
}
END_WORD
REGISTER_WORD(ZUp)

WORD(ZDown)
CODE('s')
virtual void execute(std::shared_ptr<MachineState> ms)
{
    currentEEPose.pz -= bDelta;
}
END_WORD
REGISTER_WORD(ZDown)

WORD(SetGripperThresh)
CODE(1179713)     // capslock + numlock + a
virtual void execute(std::shared_ptr<MachineState> ms) {
  gripperThresh = lastMeasuredClosed + lastMeasuredBias;
  cout << "lastMeasuredClosed: " << lastMeasuredClosed << " lastMeasuredBias: " << lastMeasuredBias << endl;
  cout << "gripperThresh = " << gripperThresh << endl;
}
END_WORD
REGISTER_WORD(SetGripperThresh)

WORD(CalibrateGripper)
CODE('i') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  //baxter_core_msgs::EndEffectorCommand command;
  //command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
  //command.id = 65538;
  //gripperPub.publish(command);
  calibrateGripper(ms);
}
END_WORD
REGISTER_WORD(CalibrateGripper)

WORD(CloseGripper)
CODE('j')
virtual void execute(std::shared_ptr<MachineState> ms) {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 0.0}";
  command.id = 65538;
  gripperPub.publish(command);
}
END_WORD
REGISTER_WORD(CloseGripper)

WORD(OpenGripper)
CODE('k')
virtual void execute(std::shared_ptr<MachineState> ms) {
  baxter_core_msgs::EndEffectorCommand command;
  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  command.args = "{\"position\": 100.0}";
  command.id = 65538;
  gripperPub.publish(command);
  lastMeasuredClosed = gripperPosition;
}
END_WORD
REGISTER_WORD(OpenGripper)


WORD(SetMovementSpeedNowThatsFast)
CODE(1114193)    // numlock + Q
virtual void execute(std::shared_ptr<MachineState> ms) {
  bDelta = NOW_THATS_FAST;
}
END_WORD
REGISTER_WORD(SetMovementSpeedNowThatsFast)

WORD(SetMovementSpeedMoveEvenFaster)
CODE(1114199)     // numlock + W
virtual void execute(std::shared_ptr<MachineState> ms) {
  bDelta = MOVE_EVEN_FASTER;
}
END_WORD
REGISTER_WORD(SetMovementSpeedMoveEvenFaster)


WORD(SetMovementSpeedMoveFaster)
CODE(1114181)  // numlock + E
virtual void execute(std::shared_ptr<MachineState> ms) {
  bDelta = MOVE_FASTER;
}
END_WORD
REGISTER_WORD(SetMovementSpeedMoveFaster)

WORD(SetMovementSpeedMoveFast)
CODE(1048674)     // numlock + b
virtual void execute(std::shared_ptr<MachineState> ms)  {
  bDelta = MOVE_FAST;
}
END_WORD
REGISTER_WORD(SetMovementSpeedMoveFast)

WORD(SetMovementSpeedMoveMedium)
CODE(1048686)   // numlock + n
virtual void execute(std::shared_ptr<MachineState> ms) {
  bDelta = MOVE_MEDIUM;
}
END_WORD
REGISTER_WORD(SetMovementSpeedMoveMedium)

WORD(SetMovementSpeedMoveSlow)
CODE(1114190) // numlock + N
virtual void execute(std::shared_ptr<MachineState> ms) {
  bDelta = MOVE_SLOW;
}
END_WORD
REGISTER_WORD(SetMovementSpeedMoveSlow)

WORD(SetMovementSpeedMoveVerySlow)
CODE(1114178) // numlock + B
virtual void execute(std::shared_ptr<MachineState> ms) {
	bDelta = MOVE_VERY_SLOW;
}
END_WORD
REGISTER_WORD(SetMovementSpeedMoveVerySlow)

WORD(ChangeToHeight0)
CODE(1245217) // capslock + numlock + !
virtual void execute(std::shared_ptr<MachineState> ms) {
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
REGISTER_WORD(ChangeToHeight0)

WORD(ChangeToHeight1)
CODE(1245248)     // capslock + numlock + @
virtual void execute(std::shared_ptr<MachineState> ms) {
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
REGISTER_WORD(ChangeToHeight1)

WORD(ChangeToHeight2)
CODE(1245219)  // capslock + numlock + #
virtual void execute(std::shared_ptr<MachineState> ms)  {
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
REGISTER_WORD(ChangeToHeight2)

WORD(ChangeToHeight3)
CODE(1245220) // capslock + numlock + $
virtual void execute(std::shared_ptr<MachineState> ms) {
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
REGISTER_WORD(ChangeToHeight3)

WORD(HundredthImpulse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 0.01;
}
END_WORD
REGISTER_WORD(HundredthImpulse)

WORD(TenthImpulse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 0.1;
}
END_WORD
REGISTER_WORD(TenthImpulse)

WORD(QuarterImpulse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 0.25;
}
END_WORD
REGISTER_WORD(QuarterImpulse)

WORD(HalfImpulse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 0.5;
}
END_WORD
REGISTER_WORD(HalfImpulse)

WORD(FullImpulse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 1.0;
}
END_WORD
REGISTER_WORD(FullImpulse)

WORD(CruisingSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //w1GoThresh = 0.40;
  //currentEESpeedRatio = 0.75;
  currentEESpeedRatio = 1.0;
}
END_WORD
REGISTER_WORD(CruisingSpeed)

WORD(ApproachSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //w1GoThresh = 0.01;
  currentEESpeedRatio = 0.05;//0.035;//0.07;//0.05;
}
END_WORD
REGISTER_WORD(ApproachSpeed)

WORD(DepartureSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //w1GoThresh = 0.05;
  currentEESpeedRatio = 0.5;
}
END_WORD
REGISTER_WORD(DepartureSpeed)

WORD(ResetW1ThreshToDefault)
virtual void execute(std::shared_ptr<MachineState> ms) {
  w1GoThresh = 0.03;
}
END_WORD
REGISTER_WORD(ResetW1ThreshToDefault)

WORD(RasterScanningSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //w1GoThresh = 0.05;
  currentEESpeedRatio = 0.02;
}
END_WORD
REGISTER_WORD(RasterScanningSpeed)

WORD(FasterRasterScanningSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 0.1;
}
END_WORD
REGISTER_WORD(FasterRasterScanningSpeed)

WORD(IRCalibrationSpeed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEESpeedRatio = 0.04;
}
END_WORD
REGISTER_WORD(IRCalibrationSpeed)

WORD(Hover)
virtual void execute(std::shared_ptr<MachineState> ms) {
  lastHoverTrueEEPoseEEPose = trueEEPoseEEPose;
  ms->pushWord("hoverA");
  endThisStackCollapse = 1;
  shouldIDoIK = 1;
  lastHoverRequest = ros::Time::now();
  lastEndpointCallbackRequest = lastHoverRequest;
}
END_WORD
REGISTER_WORD(Hover)

WORD(HoverA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (lastEndpointCallbackRequest >= lastEndpointCallbackReceived) {
    ms->pushWord("hoverA");
    cout << "hoverA waiting for endpointCallback." << endl;
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
	ms->pushWord("hoverA"); 
	endThisStackCollapse = 1;
	shouldIDoIK = 1;
	cout << "hoverA distance requirement not met, distance angleDistance: " << distance << " " << angleDistance << endl;
	lastHoverTrueEEPoseEEPose = trueEEPoseEEPose;
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
REGISTER_WORD(HoverA)

WORD(SpawnTargetClassAtEndEffector)
CODE(65379) // insert
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "SpawnTargetClassAtEndEffector called." << endl;
  if (targetClass < 0) {
    cout << "Not spawning because targetClass is " << targetClass << endl;
    return;
  }

  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
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
  
    mapBox(box);
    vector<BoxMemory> newMemories;
    for (int i = 0; i < blueBoxMemories.size(); i++) {
      newMemories.push_back(blueBoxMemories[i]);
    }
    newMemories.push_back(box);
    blueBoxMemories = newMemories;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(SpawnTargetClassAtEndEffector)

WORD(DestroyObjectInEndEffector)
CODE(65535) // delete
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (objectInHandLabel >= 0) {
    cout << "destroyObjectInEndEffector: The " << classLabels[objectInHandLabel] << " in your hand simply vanished." << endl;
    objectInHandLabel = -1;
  } else {
    cout << "destroyObjectInEndEffector: There is nothing in your hand so there is nothing to destroy." << objectInHandLabel << endl;
  }
}
END_WORD
REGISTER_WORD(DestroyObjectInEndEffector)

WORD(PickObjectUnderEndEffector)
CODE(65365) // page up
virtual void execute(std::shared_ptr<MachineState> ms) {
  std_msgs::Empty msg;
  pickObjectUnderEndEffectorCommandCallback(msg);
}
END_WORD
REGISTER_WORD(PickObjectUnderEndEffector)

WORD(PlaceObjectInEndEffector)
CODE(65366) // page down
virtual void execute(std::shared_ptr<MachineState> ms) {
  std_msgs::Empty msg;
  placeObjectInEndEffectorCommandCallback(msg);
}
END_WORD
REGISTER_WORD(PlaceObjectInEndEffector)

WORD(SetCurrentCornellTableToZero)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Setting currentCornellTableIndex to " << "0" << " out of " << numCornellTables << "." << endl;
  currentCornellTableIndex = 0;
}
END_WORD
REGISTER_WORD(SetCurrentCornellTableToZero)

WORD(IncrementCurrentCornellTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentCornellTableIndex = (currentCornellTableIndex + 1 + numCornellTables) % numCornellTables;
  cout << "Incrementing currentCornellTableIndex to " << currentCornellTableIndex << " out of " << numCornellTables << "." << endl;
}
END_WORD
REGISTER_WORD(IncrementCurrentCornellTable)

WORD(DecrementCurrentCornellTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentCornellTableIndex = (currentCornellTableIndex - 1 + numCornellTables) % numCornellTables;
  cout << "Decrementing currentCornellTableIndex to " << currentCornellTableIndex << " out of " << numCornellTables << "." << endl;
}
END_WORD
REGISTER_WORD(DecrementCurrentCornellTable)

WORD(MoveToCurrentCornellTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (currentCornellTableIndex >= 0) {
    currentEEPose.px = cornellTables[currentCornellTableIndex].px;
    currentEEPose.py = cornellTables[currentCornellTableIndex].py;
  }
}
END_WORD
REGISTER_WORD(MoveToCurrentCornellTable)

WORD(SpawnTargetMasterSpriteAtEndEffector)
CODE(130915) // shift + insert
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "SpawnTargetMasterSpriteAtEndEffector called." << endl;
  if (targetMasterSprite < 0) {
    cout << "Not spawning because targetMasterSprite is " << targetMasterSprite << endl;
    return;
  }

  for (int s = 0; s < masterSprites.size(); s++) {
    cout << "checked " << masterSprites[s].name << " as masterSprites[" << s << "] scale " << masterSprites[s].scale << " image size " << masterSprites[s].image.size() << endl;
  }
  
  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
    Sprite sprite;
    sprite.image = masterSprites[targetMasterSprite].image.clone();
    sprite.name = masterSprites[targetMasterSprite].name;
    sprite.scale = masterSprites[targetMasterSprite].scale;
    sprite.creationTime = ros::Time::now();
    sprite.pose = currentEEPose;
    sprite.top = sprite.pose;
    sprite.bot = sprite.pose;

    // rotate image and grow size
    {
      Size sz = sprite.image.size();
      int deltaWidth = 0;
      int deltaHeight = 0;
      if (sz.width < sz.height) {
	deltaWidth = sz.height - sz.width;
	sz.width = sz.height;
      }
      if (sz.width > sz.height) {
	deltaHeight = sz.width - sz.height;
	sz.height = sz.width;
      }

      Quaternionf eeqform(sprite.pose.qw, sprite.pose.qx, sprite.pose.qy, sprite.pose.qz);
      Quaternionf crane2Orient(0, 1, 0, 0);
      Quaternionf rel = eeqform * crane2Orient.inverse();
      Quaternionf ex(0,1,0,0);
      Quaternionf zee(0,0,0,1);
	    
      Quaternionf result = rel * ex * rel.conjugate();
      Quaternionf thumb = rel * zee * rel.conjugate();
      double aY = result.y();
      double aX = result.x();

      double angle = vectorArcTan(aY, aX)*180.0/3.1415926;
      angle = (angle);
      double scale = 1.0;
      Point center = Point(sz.width/2, sz.height/2);

      Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );

      Mat trans_mat = getRotationMatrix2D( center, angle, scale );
      trans_mat.at<double>(0,0) = 1;
      trans_mat.at<double>(1,1) = 1;
      trans_mat.at<double>(0,1) = 0;
      trans_mat.at<double>(1,0) = 0;
      trans_mat.at<double>(0,2) = double(deltaWidth) / 2.0;
      trans_mat.at<double>(1,2) = double(deltaHeight) / 2.0;

      //Mat rotatedSpriteImage;
      //warpAffine(sprite.image, rotatedSpriteImage, un_rot_mat, sz, INTER_LINEAR, BORDER_REPLICATE);
      Mat translatedSpriteImage;
      warpAffine(sprite.image, translatedSpriteImage, trans_mat, sz, INTER_LINEAR, BORDER_REPLICATE);
      Mat rotatedSpriteImage;
      warpAffine(translatedSpriteImage, rotatedSpriteImage, un_rot_mat, sz, INTER_LINEAR, BORDER_REPLICATE);
      sprite.image = rotatedSpriteImage;
    }

    double halfWidthMeters = sprite.image.cols / (2.0 * sprite.scale);
    double halfHeightMeters = sprite.image.rows / (2.0 * sprite.scale);

    sprite.top.px -= halfWidthMeters;
    sprite.top.py -= halfHeightMeters;
    sprite.bot.px += halfWidthMeters;
    sprite.bot.py += halfHeightMeters;

    instanceSprites.push_back(sprite);
  } else {
    assert(0);
  }
  cout << "Now instanceSprites.size() is " << instanceSprites.size() << "." << endl;
}
END_WORD
REGISTER_WORD(SpawnTargetMasterSpriteAtEndEffector)

WORD(DestroyTargetInstanceSprite)
CODE(131071) // shift + delete
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "DestroyTargetInstanceSprite called." << endl;
  if ((targetInstanceSprite < 0) ||
      (targetInstanceSprite >= instanceSprites.size())) {
    cout << "Not destoying because targetInstanceSprite is " << targetInstanceSprite << " out of " << instanceSprites.size() << endl;
    return;
  }
  
  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
    vector<Sprite> newInstanceSprites;
    for (int s = 0; s < instanceSprites.size(); s++) {
      if (s != targetInstanceSprite) {
	newInstanceSprites.push_back(instanceSprites[s]);
      }
    }
    instanceSprites = newInstanceSprites;
  } else {
    assert(0);
  }
  cout << "Now instanceSprites.size() is " << instanceSprites.size() << "." << endl;
}
END_WORD
REGISTER_WORD(DestroyTargetInstanceSprite)

WORD(IncrementTargetInstanceSprite)
CODE(130901) // shift + page up
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
    int base = instanceSprites.size();
    targetInstanceSprite = (targetInstanceSprite + 1 + base) % max(base, 1);
    cout << "Incrementing targetInstanceSprite to " << targetInstanceSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(IncrementTargetInstanceSprite)

WORD(DecrementTargetInstanceSprite)
CODE(130902) // shift + page down
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
    int base = instanceSprites.size();
    targetInstanceSprite = (targetInstanceSprite - 1 + base) % max(base, 1);
    cout << "Decrementing targetInstanceSprite to " << targetInstanceSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(DecrementTargetInstanceSprite)

WORD(IncrementTargetMasterSprite)
CODE(130896) // shift + home 
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
    int base = masterSprites.size();
    targetMasterSprite = (targetMasterSprite + 1 + base) % max(base, 1);
    cout << "Incrementing targetMasterSprite to " << targetMasterSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(IncrementTargetMasterSprite)

WORD(DecrementTargetMasterSprite)
CODE(130903) // shift + end 
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.chosen_mode == PHYSICAL) {
    return;
  } else if (ms->config.chosen_mode == SIMULATED) {
    int base = masterSprites.size();
    targetMasterSprite = (targetMasterSprite - 1 + base) % max(base, 1);
    cout << "Decrementing targetMasterSprite to " << targetMasterSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(DecrementTargetMasterSprite)

WORD(ComeToStop)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //currentEEPose = trueEEPoseEEPose;
  ms->pushWord("comeToStopA");
  comeToStopStart = ros::Time::now();
  cout << "Waiting to come to a stop..." << endl;
}
END_WORD
REGISTER_WORD(ComeToStop)

WORD(ComeToStopA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (ms->config.currentMovementState == STOPPED) ||
       (ms->config.currentMovementState == BLOCKED) ) {
    // do nothing
    cout << "Came to a stop, moving on." << endl;
  } else {
    ros::Duration timeSinceCTS = ros::Time::now() - comeToStopStart;
    if (timeSinceCTS.toSec() < comeToStopTimeout) {
      ms->pushWord("comeToStopA");
    } else {
      ROS_WARN_STREAM("_____*____*________");
      ROS_ERROR_STREAM("comeToStop timeout reached, moving on.");
      ROS_WARN_STREAM("_____*____*________");

      // waitUntilCurrentPosition will time out, make sure that there will
      //  be no cycles introduced
      currentEEPose.pz = trueEEPose.position.z + 0.001;
      cout << "  backing up just a little to dislodge from failed hover, then waiting." << endl;
      ms->pushWord("waitUntilAtCurrentPosition"); 
    }
    endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(ComeToStopA)

WORD(ComeToHover)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //currentEEPose = trueEEPoseEEPose;
  ms->pushWord("comeToHoverA");
  comeToHoverStart = ros::Time::now();
  cout << "Waiting to come to a hover..." << endl;
}
END_WORD
REGISTER_WORD(ComeToHover)

WORD(ComeToHoverA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (ms->config.currentMovementState == HOVERING) ) {
    // do nothing
    cout << "Came to a hover, moving on." << endl;
  } else {
    ros::Duration timeSinceCTH = ros::Time::now() - comeToHoverStart;
    if (timeSinceCTH.toSec() < comeToHoverTimeout) {
      ms->pushWord("comeToHoverA");
    } else {
      ROS_WARN_STREAM("_____*____*________");
      ROS_ERROR_STREAM("comeToHover timeout reached, moving on.");
      ROS_WARN_STREAM("_____*____*________");
    }
    endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(ComeToHoverA)

WORD(WaitForTugThenOpenGripper)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("waitForTugThenOpenGripperA");
  ms->pushWord("waitUntilEndpointCallbackReceived");
  ms->pushWord("comeToHover");
  ms->pushWord("waitUntilAtCurrentPosition");
  waitForTugStart = ros::Time::now();
  cout << "Waiting to feel a tug... " << ARMED << " " << ms->config.currentMovementState << endl;
}
END_WORD
REGISTER_WORD(WaitForTugThenOpenGripper)

WORD(WaitForTugThenOpenGripperA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  endThisStackCollapse = 1;
  if (0) { // position based
    if ( ( ms->config.currentMovementState == MOVING ) ||
	 ( !gripperGripping ) ) {
      if ( !gripperGripping ) {
	cout << "There is nothing in the gripper so we should move on..." << endl;
      }
      if (ms->config.currentMovementState == MOVING) {
	cout << "Felt a tug, opening gripper." << endl;
      }
      ms->pushWord("openGripper");
    } else {
      ms->config.currentMovementState = ARMED;
      ros::Duration timeSinceWFT = ros::Time::now() - waitForTugStart;
      if (timeSinceWFT.toSec() < waitForTugTimeout) {
	ms->pushWord("waitForTugThenOpenGripperA");
      } else {
	ROS_WARN_STREAM("_____*____*________");
	ROS_ERROR_STREAM("waitForTugThenOpenGripper timeout reached, moving on.");
	ROS_WARN_STREAM("_____*____*________");
      }
    }
  } else { // wrench based
    double wrenchNorm = sqrt( squareDistanceEEPose(eePoseZero, trueEEWrench) );
    double wrenchThresh = 15;
    bool wrenchOverThresh = ( wrenchNorm > wrenchThresh );
    if ( wrenchOverThresh ||
	 ( !gripperGripping ) ) {
      if ( !gripperGripping ) {
	cout << "There is nothing in the gripper so we should move on..." << endl;
      }
      if ( wrenchOverThresh ) {
	cout << "Felt a tug, opening gripper; wrenchNorm wrenchThresh: " << wrenchNorm << " " << wrenchThresh << " " << endl;
      }
      ms->pushWord("openGripper");
    } else {
      ms->config.currentMovementState = ARMED;
      ros::Duration timeSinceWFT = ros::Time::now() - waitForTugStart;
      if (timeSinceWFT.toSec() < waitForTugTimeout) {
	ms->pushWord("waitForTugThenOpenGripperA");
      } else {
	ROS_WARN_STREAM("_____*____*________");
	ROS_ERROR_STREAM("waitForTugThenOpenGripper timeout reached, moving on.");
	ROS_WARN_STREAM("_____*____*________");
      }
    }
  }
}
END_WORD
REGISTER_WORD(WaitForTugThenOpenGripperA)

WORD(Idler)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.currentIdleMode == EMPTY) {
    // empty
  } else if (ms->config.currentIdleMode == STOPCLEAR) {
    ms->pushWord("clearStack"); 
  } else if (ms->config.currentIdleMode == PATROL) {
    ms->pushWord("clearStackIntoMappingPatrol"); 
  } else if (ms->config.currentIdleMode == CRANE) {
    ms->pushWord("clearStackAcceptFetchCommands"); 
    ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
    ms->pushWord("assumeCrane1"); 
  } else if (ms->config.currentIdleMode == SHRUG) {
    ms->pushWord("clearStackAcceptFetchCommands"); 
    ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
    ms->pushWord("assumeShrugPose"); 
  } else {
    assert(0);
  }
  ms->pushWord("setPatrolStateToIdling");
}
END_WORD
REGISTER_WORD(Idler)

WORD(SetMovementStateToMoving)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentMovementState = MOVING;
  lastTrueEEPoseEEPose = trueEEPoseEEPose;
  lastMovementStateSet = ros::Time::now();
}
END_WORD
REGISTER_WORD(SetMovementStateToMoving)

WORD(AssumeCrane1)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = crane1;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeCrane1)

WORD(AssumeShrugPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = shrugPose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeShrugPose)

WORD(AssumeHandingPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = handingPose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeHandingPose)

WORD(SetPatrolStateToIdling)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolState = IDLING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToIdling)

WORD(SetPatrolStateToPatrolling)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolState = PATROLLING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToPatrolling)

WORD(SetPatrolStateToPicking)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolState = PICKING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToPicking)

WORD(SetPatrolStateToPlacing)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolState = PLACING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToPlacing)

WORD(SetPatrolStateToHanding)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolState = HANDING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToHanding)

WORD(SetPatrolModeToLoop)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolMode = LOOP;
}
END_WORD
REGISTER_WORD(SetPatrolModeToLoop)

WORD(SetPatrolModeToOnce)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPatrolMode = ONCE;
}
END_WORD
REGISTER_WORD(SetPatrolModeToOnce)

WORD(SetPlaceModeToWarehouse)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPlaceMode = WAREHOUSE;
}
END_WORD
REGISTER_WORD(SetPlaceModeToWarehouse)

WORD(SetPlaceModeToHand)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPlaceMode = HAND;
}
END_WORD
REGISTER_WORD(SetPlaceModeToHand)

WORD(SetIdleModeToCrane)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentIdleMode = CRANE;
}
END_WORD
REGISTER_WORD(SetIdleModeToCrane)

WORD(SetIdleModeToShrug)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentIdleMode = SHRUG;
}
END_WORD
REGISTER_WORD(SetIdleModeToShrug)

WORD(SetIdleModeToEmpty)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentIdleMode = EMPTY;
}
END_WORD
REGISTER_WORD(SetIdleModeToEmpty)

WORD(SetIdleModeToStopClear)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentIdleMode = STOPCLEAR;
}
END_WORD
REGISTER_WORD(SetIdleModeToStopClear)

WORD(SetIdleModeToPatrol)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentIdleMode = PATROL;
}
END_WORD
REGISTER_WORD(SetIdleModeToPatrol)

