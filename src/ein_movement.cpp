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
  
  if ((distance > w1GoThresh*w1GoThresh) || (angleDistance > w1AngleThresh*w1AngleThresh))
    pushWord("waitUntilAtCurrentPositionB"); 
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
  
  if (waitUntilAtCurrentPositionCounter < waitUntilAtCurrentPositionCounterMax) {
    waitUntilAtCurrentPositionCounter++;
    if ((distance > w1GoThresh*w1GoThresh) || (angleDistance > w1AngleThresh*w1AngleThresh)) {
      pushWord("waitUntilAtCurrentPositionB"); 
    }
  } else {
    cout << "Warning: waitUntilAtCurrentPosition timed out, moving on." << endl;
  }
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
  reticle = heightReticles[currentThompsonHeightIdx];
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
  reticle = heightReticles[currentThompsonHeightIdx];
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
  reticle = heightReticles[currentThompsonHeightIdx];
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
  reticle = heightReticles[currentThompsonHeightIdx];
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD

