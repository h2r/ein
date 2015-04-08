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

  currentMovementState = MOVING;
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
  if (currentMovementState == STOPPED) {
    cout << "Warning: waitUntilAtCurrentPosition currentMovementState = STOPPED, moving on." << endl;
    endThisStackCollapse = endCollapse;
    return;
  }
  if (currentMovementState == BLOCKED) {
    cout << "Warning: waitUntilAtCurrentPosition currentMovementState = BLOCKED, moving on." << endl;
    endThisStackCollapse = endCollapse;
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
  //baxter_core_msgs::EndEffectorCommand command;
  //command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
  //command.id = 65538;
  //gripperPub.publish(command);
  calibrateGripper();
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
  currentEESpeedRatio = 0.1;//0.035;//0.07;//0.05;
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
  lastEndpointCallbackRequest = lastHoverRequest;
}
END_WORD

WORD(HoverA)
virtual void execute() {
  if (lastEndpointCallbackRequest >= lastEndpointCallbackReceived) {
    pushWord("hoverA");
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
	pushWord("hoverA"); 
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

WORD(SpawnTargetClassAtEndEffector)
CODE(65379) // insert
virtual void execute() {
  cout << "SpawnTargetClassAtEndEffector called." << endl;
  if (targetClass < 0) {
    cout << "Not spawning because targetClass is " << targetClass << endl;
    return;
  }

  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
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
  }
}
END_WORD

WORD(DestroyObjectInEndEffector)
CODE(65535) // delete
virtual void execute() {
  if (objectInHandLabel >= 0) {
    cout << "destroyObjectInEndEffector: The " << classLabels[objectInHandLabel] << " in your hand simply vanished." << endl;
    objectInHandLabel = -1;
  } else {
    cout << "destroyObjectInEndEffector: There is nothing in your hand so there is nothing to destroy." << objectInHandLabel << endl;
  }
}
END_WORD

WORD(PickObjectUnderEndEffector)
CODE(65365) // page up
virtual void execute() {
  std_msgs::Empty msg;
  pickObjectUnderEndEffectorCommandCallback(msg);
}
END_WORD

WORD(PlaceObjectInEndEffector)
CODE(65366) // page down
virtual void execute() {
  std_msgs::Empty msg;
  placeObjectInEndEffectorCommandCallback(msg);
}
END_WORD

WORD(SetCurrentCornellTableToZero)
virtual void execute() {
  cout << "Setting currentCornellTableIndex to " << "0" << " out of " << numCornellTables << "." << endl;
  currentCornellTableIndex = 0;
}
END_WORD

WORD(IncrementCurrentCornellTable)
virtual void execute() {
  currentCornellTableIndex = (currentCornellTableIndex + 1 + numCornellTables) % numCornellTables;
  cout << "Incrementing currentCornellTableIndex to " << currentCornellTableIndex << " out of " << numCornellTables << "." << endl;
}
END_WORD

WORD(DecrementCurrentCornellTable)
virtual void execute() {
  currentCornellTableIndex = (currentCornellTableIndex - 1 + numCornellTables) % numCornellTables;
  cout << "Decrementing currentCornellTableIndex to " << currentCornellTableIndex << " out of " << numCornellTables << "." << endl;
}
END_WORD

WORD(MoveToCurrentCornellTable)
virtual void execute() {
  if (currentCornellTableIndex >= 0) {
    currentEEPose.px = cornellTables[currentCornellTableIndex].px;
    currentEEPose.py = cornellTables[currentCornellTableIndex].py;
  }
}
END_WORD

WORD(SpawnTargetMasterSpriteAtEndEffector)
CODE(130915) // shift + insert
virtual void execute() {
  cout << "SpawnTargetMasterSpriteAtEndEffector called." << endl;
  if (targetMasterSprite < 0) {
    cout << "Not spawning because targetMasterSprite is " << targetMasterSprite << endl;
    return;
  }

  for (int s = 0; s < masterSprites.size(); s++) {
    cout << "checked " << masterSprites[s].name << " as masterSprites[" << s << "] scale " << masterSprites[s].scale << " image size " << masterSprites[s].image.size() << endl;
  }
  
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
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
  }
  cout << "Now instanceSprites.size() is " << instanceSprites.size() << "." << endl;
}
END_WORD

WORD(DestroyTargetInstanceSprite)
CODE(131071) // shift + delete
virtual void execute() {
  cout << "DestroyTargetInstanceSprite called." << endl;
  if ((targetInstanceSprite < 0) ||
      (targetInstanceSprite >= instanceSprites.size())) {
    cout << "Not destoying because targetInstanceSprite is " << targetInstanceSprite << " out of " << instanceSprites.size() << endl;
    return;
  }
  
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
    vector<Sprite> newInstanceSprites;
    for (int s = 0; s < instanceSprites.size(); s++) {
      if (s != targetInstanceSprite) {
	newInstanceSprites.push_back(instanceSprites[s]);
      }
    }
    instanceSprites = newInstanceSprites;
  }
  cout << "Now instanceSprites.size() is " << instanceSprites.size() << "." << endl;
}
END_WORD

WORD(IncrementTargetInstanceSprite)
CODE(130901) // shift + page up
virtual void execute() {
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
    int base = instanceSprites.size();
    targetInstanceSprite = (targetInstanceSprite + 1 + base) % max(base, 1);
    cout << "Incrementing targetInstanceSprite to " << targetInstanceSprite << " out of " << base << "." << endl;
  }
}
END_WORD

WORD(DecrementTargetInstanceSprite)
CODE(130902) // shift + page down
virtual void execute() {
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
    int base = instanceSprites.size();
    targetInstanceSprite = (targetInstanceSprite - 1 + base) % max(base, 1);
    cout << "Decrementing targetInstanceSprite to " << targetInstanceSprite << " out of " << base << "." << endl;
  }
}
END_WORD

WORD(IncrementTargetMasterSprite)
CODE(130896) // shift + home 
virtual void execute() {
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
    int base = masterSprites.size();
    targetMasterSprite = (targetMasterSprite + 1 + base) % max(base, 1);
    cout << "Incrementing targetMasterSprite to " << targetMasterSprite << " out of " << base << "." << endl;
  }
}
END_WORD

WORD(DecrementTargetMasterSprite)
CODE(130903) // shift + end 
virtual void execute() {
  if (chosen_mode == PHYSICAL) {
    return;
  } else if (chosen_mode == SIMULATED) {
    int base = masterSprites.size();
    targetMasterSprite = (targetMasterSprite - 1 + base) % max(base, 1);
    cout << "Decrementing targetMasterSprite to " << targetMasterSprite << " out of " << base << "." << endl;
  }
}
END_WORD

