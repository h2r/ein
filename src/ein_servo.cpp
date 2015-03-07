WORD(FindBestOfFourGraspsUsingMemory)
CODE(1048620)     // numlock + ,
virtual void execute()       {
  cout << "Selecting best of 4 grasps...  numlock + , useContinuousGraspTransform = " << useContinuousGraspTransform << endl;

  if (useContinuousGraspTransform) {
  } else {
    pushWord(1179728); // estimateGlobalGraspGear
  }

  // select max target cumulative
  pushWord("selectMaxTargetCumulative");
  // apply grasp filter for 4
  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter"); // apply grasp filter
  pushWord("prepareToApplyGraspFilterFor4"); // prepare grasp filter for 4
  // load reg1
  pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  // change gear to 4
  pushWord("shiftIntoGraspGear4");

  // select max target cumulative
  pushWord("selectMaxTargetCumulative");
  // apply grasp filter for 3
  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter"); // apply grasp filter
  pushWord("prepareToApplyGraspFilterFor3"); // prepare grasp filter for 3
  // load reg1
  pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  // change gear to 3
  pushWord("shiftIntoGraspGear3");

  // select max target cumulative
  pushWord("selectMaxTargetCumulative");
  // apply grasp filter for 2
  pushWord("drawMapRegisters"); // drawMapRegisters
  pushWord("applyGraspFilter"); // apply grasp filter
  pushWord("prepareToApplyGraspFilterFor2"); // prepare to apply grasp filter for 2
  // load reg1
  pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  // change gear to 2
  pushWord("shiftIntoGraspGear2");

  // select max target NOT cumulative
  pushWord("selectMaxTargetNotCumulative");


              
  // apply grasp filter for 1
  pushWord("drawMapRegisters"); // drawMapRegisters 
  pushWord("applyGraspFilter"); // apply grasp filter
  pushWord("prepareToApplyGraspFilterFor1"); // prepare to apply grasp filter for 1
  // load reg1
  pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap

  // change gear to 1
  pushWord("shiftIntoGraspGear1");

  // ATTN 10
  // loadSampled gives proper Thompson
  // loadMarginal is MAP estimate
  //pushWord(131117); // loadSampledGraspMemory
  //pushWord("loadMarginalGraspMemory"); // loadMarginalGraspMemory	
  switch (currentPickMode) {
  case STATIC_PRIOR:
    {
      pushWord("loadMarginalGraspMemory"); // loadMarginalGraspMemory
    }
    break;
  case LEARNING_SAMPLING:
    {
      pushWord(131117); // loadSampledGraspMemory
    }
    break;
  case LEARNING_ALGORITHMC:
  case STATIC_MARGINALS:
    {
      pushWord("loadMarginalGraspMemory"); // loadMarginalGraspMemory
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }

  pushWord("turnOffScanning"); // turn off scanning
  pushWord(1179721); // set graspMemories from classGraspMemories
}
END_WORD





WORD(AssumeWinningGgAndXyInLocalPose)
CODE(1114175)     // numlock + ?
virtual void execute()       {
  double targetX = trX;
  double targetY = trY;

  trZ = rangeMapReg1[maxX + maxY*rmWidth];

  currentEEPose.px = targetX;
  currentEEPose.py = targetY;
      
  cout << "Assuming x,y,gear: " << targetX << " " << targetY << " " << maxGG << endl;

  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  // ATTN 19
  if (useContinuousGraspTransform) {
    cout << "Assuming continuous maxGG: " << maxGG << " localMaxGG: " << localMaxGG << endl;
    setCCRotation((maxGG+4)%4); 
  } else {
    pushWord(1048631); // assume best gear
  }
}
END_WORD

WORD(MoveToTargetZAndGrasp)
CODE(1048682)     // numlock + j
virtual void execute()       {
  pushWord('j');  // close gripper
  double threshedZ = min(trZ, 0.0);

  double pickZ = (-(threshedZ + currentTableZ) - graspDepth);
  double flushZ = -currentTableZ + pickFlushFactor;
  pickZ = max(flushZ, pickZ);

  double deltaZ = pickZ - currentEEPose.pz;

  lastPickHeight = pickZ;
  double zTimes = fabs(floor(deltaZ / bDelta)); 

  int numNoOps = 2;
  if (deltaZ > 0)
    for (int zc = 0; zc < zTimes; zc++) {
      for (int cc = 0; cc < numNoOps; cc++) {
        pushWord('C');
      }
      pushWord('w');
    }
  if (deltaZ < 0)
    for (int zc = 0; zc < zTimes; zc++) {
      for (int cc = 0; cc < numNoOps; cc++) {
        pushWord('C');
      }
      pushWord('s');
    }
}
END_WORD

WORD(ShakeItUpAndDown)
CODE(131081)   // capslock + tab
virtual void execute() {
  eepReg5 = currentEEPose;

  eepReg6 = currentEEPose;
  eepReg6.pz += 0.2;

  pushSpeedSign(MOVE_FAST);    
  if (isGripperGripping()) {
    happy();
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord('5');  // assume pose at register 5
      
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord('6'); // assume pose at register 6
      
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord('5'); // assume pose at register 5
      
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord('6');

  }

  pushSpeedSign(NOW_THATS_FAST);

}
END_WORD


WORD(TryToMoveToTheLastPickHeight)
CODE( 262241)     // ctrl + a
virtual void execute() {
  double deltaZ = (lastPickHeight) - currentEEPose.pz;
  double zTimes = fabs(floor(deltaZ / bDelta)); 
  int numNoOps = 2;
  if (deltaZ > 0)
    for (int zc = 0; zc < zTimes; zc++) {
      for (int cc = 0; cc < numNoOps; cc++) {
        pushWord('C');
      }
      pushWord('w');
    }
  if (deltaZ < 0)
    for (int zc = 0; zc < zTimes; zc++) {
      for (int cc = 0; cc < numNoOps; cc++) {
        pushWord('C');
      }
      pushWord('s');
    }
}
END_WORD


WORD(AssertYesGrasp)
CODE( 131157)     // capslock + u
    // if gripper is empty, pop next instruction and return. if not, just return
    // pull from bag will push itself and assert yes grasp
    // assert yes grasp
  virtual void execute()       {
  cout << "assert yes grasp: " << gripperMoving << " " << gripperGripping << " " << gripperPosition << endl;
  // TODO push this and then a calibration message if uncalibrated
  // push this again if moving
  if (gripperMoving) {
    pushWord("assertYesGrasp"); // assert yes grasp
  } else {
    if (isGripperGripping())
      {
        popWord();
        // leave gripper in released state
        cout << "  assert yes pops back instruction." << endl;
      } else {
      cout << "  assert yes merely returns." << endl;
      // resets the gripper server
      //int sis = system("bash -c \"echo -e \'cC\003\' | rosrun baxter_examples gripper_keyboard.py\"");
      //calibrateGripper();
    }
  }
}
END_WORD



WORD(AssertNoGrasp)
CODE(196649)     // capslock + i
// if gripper is full, pop next instruction and return. if not, just return
// shake it off will push itself and then assert no grasp
  virtual void execute() {
  // TODO push this and then a calibration message if uncalibrated
  // push this again if moving

  cout << "assert no grasp: " << gripperMoving << " " << gripperGripping << " " << gripperPosition << endl;

  if (gripperMoving) {
    pushWord("assertNoGrasp"); // assert no grasp
  } else {
    if (!isGripperGripping())  {
      popWord();
      // leave gripper in released state
      cout << "  assert no pops back instruction." << endl;
      if (thisGraspReleased == UNKNOWN) {
        thisGraspReleased = SUCCESS;
      }
    } else {
      // stuck
      pushWord('Y'); // pause stack execution
      pushCopies("beep", 15); // beep
      cout << "Stuck, please reset the object. ";
      cout << " gripperPosition: " << gripperPosition;
      cout << " gripperThresh: " << gripperThresh << endl;
      if (thisGraspReleased == UNKNOWN) {
        thisGraspReleased = FAILURE;
        sad();
      }
    }
    pushWord("openGripper"); // open gripper

  }

}
END_WORD

WORD(ShakeItOff1)
CODE( 131151)     // capslock + o
  virtual void execute()       {
  int depthToPlunge = 24;
  int flexThisFar = 80;
  cout << "SHAKING IT OFF!!!" << endl;
  pushWord(196719); // shake it off 2
  pushWord("assertNoGrasp"); // assert no grasp

  pushNoOps(60);
  //pushWord('2'); // assume pose at register 2
  pushWord('j'); // close gripper
  pushNoOps(20);
  pushWord("openGripper"); // open gripper
  pushWord('j'); // close gripper
  pushNoOps(20);
  //pushCopies('w', depthToPlunge); // move up 
  pushWord("openGripper"); // open gripper
  pushWord('j'); // close gripper
  //pushNoOps(20);
  //pushCopies('s'+65504, flexThisFar); // rotate forward
  //pushCopies('e', 5); // move forward
  //pushCopies('s', depthToPlunge); // move down
  pushWord("openGripper"); // open gripper
  pushNoOps(50);
  //pushCopies('w'+65504, flexThisFar); // rotate forward

  //pushWord('2'); // assume pose at register 2
  pushSpeedSign(MOVE_FAST);

  // resets the gripper server
  //int sis = system("bash -c \"echo -e \'cC\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  //calibrateGripper();
}
END_WORD



WORD(LoadTargetClassRangeMapIntoRegister1)
CODE(131162)     // capslock + z
virtual void execute() {
  loadGlobalTargetClassRangeMap(rangeMap, rangeMapReg1);
}
END_WORD

WORD(CountGrasp)
CODE(196717)     // capslock + M
virtual void execute()       {
  // ATTN 19
  int i = localMaxX + localMaxY * rmWidth + rmWidth*rmWidth*localMaxGG;
  int j = localMaxX + localMaxY * rmWidth + rmWidth*rmWidth*0;
  graspAttemptCounter++;      
  cout << "thisGraspPicked: " << operationStatusToString(thisGraspPicked) << endl;
  cout << "thisGraspReleased: " << operationStatusToString(thisGraspReleased) << endl;
  if (ARE_GENERIC_PICK_LEARNING()) {
    //graspMemoryTries[j+0*rmWidth*rmWidth]++;
    //graspMemoryTries[j+1*rmWidth*rmWidth]++;
    //graspMemoryTries[j+2*rmWidth*rmWidth]++;
    //graspMemoryTries[j+3*rmWidth*rmWidth]++;
    if (graspMemoryTries[i] <= 1.0) {
      graspMemoryTries[i] = 1.001;
      graspMemoryPicks[i] = 0.0;
    } else {
      graspMemoryTries[i]++;
    }
  }
  if ((thisGraspPicked == SUCCESS) && (thisGraspReleased == SUCCESS)) {
    graspSuccessCounter++;
    happy();
    if (ARE_GENERIC_PICK_LEARNING()) {
      //graspMemoryPicks[j+0*rmWidth*rmWidth]++;
      //graspMemoryPicks[j+1*rmWidth*rmWidth]++;
      //graspMemoryPicks[j+2*rmWidth*rmWidth]++;
      //graspMemoryPicks[j+3*rmWidth*rmWidth]++;
      graspMemoryPicks[i]++;
    }
        
    if (ARE_GENERIC_HEIGHT_LEARNING()) {
      recordBoundingBoxSuccess();
    }

    double thisPickRate = double(graspMemoryPicks[i]) / double(graspMemoryTries[i]);
    int thisNumTries = graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
    if (currentPickMode == LEARNING_SAMPLING) {
      if ( (thisNumTries >= thompsonMinTryCutoff) && 
           (thisPickRate >= thompsonMinPassRate) ) {
        thompsonPickHaltFlag = 1;
      }
    }
    // ATTN 20
    {
      double successes = graspMemoryPicks[i];
      double failures =  graspMemoryTries[i] - graspMemoryPicks[i];
      cout << "YYY failures, successes: " << failures << " " << successes << endl;
      successes = round(successes);
      failures = round(failures);
      cout << "XXX failures, successes: " << failures << " " << successes << endl;
      // returns probability that mu <= d given successes and failures.
      double presult = cephes_incbet(successes + 1, failures + 1, algorithmCTarget);
      // we want probability that mu > d
      double result = 1.0 - presult;

      double presult2a = cephes_incbet(successes + 1, failures + 1, algorithmCTarget + algorithmCEPS);
      double presult2b = cephes_incbet(successes + 1, failures + 1, algorithmCTarget - algorithmCEPS);
      // we want probability that 
      //  algorithmCTarget - algorithmCEPS < mu < algorithmCTarget + algorithmCEPS
      double result2 = presult2a - presult2b;

      cout << "prob that mu > d: " << result << " algorithmCAT: " << algorithmCAT << endl;
      if (currentPickMode == LEARNING_ALGORITHMC) {
        thompsonPickHaltFlag = (result > algorithmCAT);
        if (result2 > algorithmCAT) {
          thompsonPickHaltFlag = 1;
        }
      }
    }
  } else {
    double thisPickRate = double(graspMemoryPicks[i]) / double(graspMemoryTries[i]);
    int thisNumTries = graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
    sad();
    if (ARE_GENERIC_HEIGHT_LEARNING()) {
      recordBoundingBoxFailure();
    }
  }
  copyGraspMemoryTriesToClassGraspMemoryTries();
  graspSuccessRate = graspSuccessCounter / graspAttemptCounter;
  ros::Time thisTime = ros::Time::now();
  ros::Duration sinceStartOfTrial = thisTime - graspTrialStart;
      
  cout << "<><><><> Grasp attempts rate time gripperPosition currentPickMode: " << graspSuccessCounter << "/" << graspAttemptCounter << " " << graspSuccessRate << " " << sinceStartOfTrial.toSec() << " seconds " << " " << pickModeToString(currentPickMode) << endl;
  {
    double successes = graspMemoryPicks[i];
    double failures =  graspMemoryTries[i] - graspMemoryPicks[i];
    cout << "YYY failures, successes: " << failures << " " << successes << endl;
    successes = round(successes);
    failures = round(failures);
    cout << "XXX failures, successes: " << failures << " " << successes << endl;
  }
}
END_WORD

WORD(CheckGrasp)
CODE(196718)     // capslock + N 
  virtual void execute()       {
  if (gripperMoving) {
    pushWord(196718); // check grasp
  } else {
    cout << "gripperPosition: " << gripperPosition << " gripperThresh: " << gripperThresh << endl;
    cout << "gripperGripping: " << gripperGripping << endl;
    if (!isGripperGripping()) {
      cout << "Failed to pick." << endl;
      thisGraspPicked = FAILURE;
      sad();
      pushCopies("beep", 15); // beep
    } else {
      cout << "Successful pick." << endl;
      thisGraspPicked = SUCCESS;
      happy();
    }
  }
}
END_WORD

WORD(CheckAndCountGrasp)
CODE(196713)     // capslock + I
  virtual void execute()       {
  int i = localMaxX + localMaxY * rmWidth + rmWidth*rmWidth*localMaxGG;
  int j = localMaxX + localMaxY * rmWidth + rmWidth*rmWidth*0;
  if (gripperMoving) {
    pushWord(196713); // check and count grasp
  } else {
    graspAttemptCounter++;
    switch (currentPickMode) {
    case STATIC_PRIOR:
      {
      }
      break;
    case LEARNING_ALGORITHMC:
    case LEARNING_SAMPLING:
      {
        if (graspMemoryTries[i] <= 1.0) {
          graspMemoryTries[i] = 1.001;
          graspMemoryPicks[i] = 0.0;
        } else {
          graspMemoryTries[i]++;
        }
        //graspMemoryTries[j+0*rmWidth*rmWidth]++;
        //graspMemoryTries[j+1*rmWidth*rmWidth]++;
        //graspMemoryTries[j+2*rmWidth*rmWidth]++;
        //graspMemoryTries[j+3*rmWidth*rmWidth]++;
      }
      break;
    case STATIC_MARGINALS:
      {
      }
      break;
    default:
      {
        assert(0);
      }
      break;
    }
    cout << "gripperPosition: " << gripperPosition << " gripperThresh: " << gripperThresh << endl;
    if (!isGripperGripping()) {
      if (ARE_GENERIC_HEIGHT_LEARNING()) {
        recordBoundingBoxFailure();
      }
      cout << "Failed grasp." << endl;
      //pushWord('Y'); // pause stack execution
      pushCopies("beep", 15); // beep
    } else {
      if (ARE_GENERIC_HEIGHT_LEARNING()) {
        recordBoundingBoxSuccess();
      }
      graspSuccessCounter++;
      cout << "Successful grasp." << endl;
      //graspMemoryPicks[i]++;
      switch (currentPickMode) {
      case STATIC_PRIOR:
        {
        }
        break;
      case LEARNING_ALGORITHMC:
      case LEARNING_SAMPLING:
        {
          graspMemoryPicks[i]++;
          //graspMemoryPicks[j]++;
          //graspMemoryPicks[j+1*rmWidth*rmWidth]++;
          //graspMemoryPicks[j+2*rmWidth*rmWidth]++;
          //graspMemoryPicks[j+3*rmWidth*rmWidth]++;
        }
        break;
      case STATIC_MARGINALS:
        {
        }
        break;
      default:
        {
          assert(0);
        }
        break;
      }
    }

    double thisPickRate = double(graspMemoryPicks[i]) / double(graspMemoryTries[i]);
    int thisNumTries = graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;

    if (currentPickMode == LEARNING_SAMPLING) {
      if ( (thisNumTries >= thompsonMinTryCutoff) && 
           (thisPickRate >= thompsonMinPassRate) ) {
        thompsonPickHaltFlag = 1;
      }
    }
    // ATTN 20
    {
      double successes = graspMemoryPicks[i];
      double failures =  graspMemoryTries[i] - graspMemoryPicks[i];
      cout << "YYY failures, successes: " << failures << " " << successes << endl;
      successes = round(successes);
      failures = round(failures);
      cout << "XXX failures, successes: " << failures << " " << successes << endl;
      // returns probability that mu <= d given successes and failures.
      double presult = cephes_incbet(successes + 1, failures + 1, algorithmCTarget);
      // we want probability that mu > d
      double result = 1.0 - presult;

      double presult2a = cephes_incbet(successes + 1, failures + 1, algorithmCTarget + algorithmCEPS);
      double presult2b = cephes_incbet(successes + 1, failures + 1, algorithmCTarget - algorithmCEPS);
      // we want probability that 
      //  algorithmCTarget - algorithmCEPS < mu < algorithmCTarget + algorithmCEPS
      double result2 = presult2a - presult2b;

      cout << "prob that mu > d: " << result << " algorithmCAT: " << algorithmCAT << endl;
      if (currentPickMode == LEARNING_ALGORITHMC) {
        thompsonPickHaltFlag = (result > algorithmCAT);
        if (result2 > algorithmCAT) {
          thompsonPickHaltFlag = 1;
        }
      }
    }

    copyGraspMemoryTriesToClassGraspMemoryTries();
    graspSuccessRate = graspSuccessCounter / graspAttemptCounter;
    ros::Time thisTime = ros::Time::now();
    ros::Duration sinceStartOfTrial = thisTime - graspTrialStart;
    cout << "<><><><> Grasp attempts rate time gripperPosition currentPickMode: " << graspSuccessCounter << "/" << graspAttemptCounter << " " << graspSuccessRate << " " << sinceStartOfTrial.toSec() << " seconds " << gripperPosition << " " << pickModeToString(currentPickMode) << endl;
  }
  {
    double successes = graspMemoryPicks[i];
    double failures =  graspMemoryTries[i] - graspMemoryPicks[i];
    cout << "YYY failures, successes: " << failures << " " << successes << endl;
    successes = round(successes);
    failures = round(failures);
    cout << "XXX failures, successes: " << failures << " " << successes << endl;
  }
}
END_WORD




WORD(PrepareForAndExecuteGraspFromMemory)
CODE(1048624)     // numlock + 0
virtual void execute()       {

  pushWord("visionCycle"); // vision cycle
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushWord("sampleHeight"); // sample height
  pushWord("shiftIntoGraspGear1"); // change to first gear

  pushWord("countGrasp"); //count grasp

  pushWord("openGripper"); // open gripper
  pushWord(131151); // shake it off 1
  pushWord("assertNoGrasp"); // assert no grasp

  pushNoOps(30);
  pushWord('j'); // close gripper
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushCopies('w', 10);
  pushNoOps(30);
  pushWord("openGripper"); // open gripper

  pushNoOps(5);
  pushWord(262241); // try to move to the last pick height 

  //count here so that if it drops it on the way it will count as a miss
  { // in case it fell out
    pushWord(196718); // check grasp

    pushNoOps(30);
    pushWord('j'); // close gripper
    pushWord(131081); // shake it up and down

    pushNoOps(5);
    pushWord('j'); // close gripper
  }

  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  if (ARE_GENERIC_HEIGHT_LEARNING()) {
    pushWord(1179687); // set random position for bblearn
  } else {
    pushWord(1048623); // numlock + /
  }

  pushCopies('s', 3);
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  if (ARE_GENERIC_HEIGHT_LEARNING())
    pushWord('4'); // assume pose at register 4
  else
    pushWord('2'); // assume pose at register 2

  pushNoOps(10);

  // XXX TODO this is broken because we no longer know the height in the transformed space
  // need to translate to current table height
  pushWord(1048682); // grasp at z inferred from target
  //pushWord(1114186); // use current range as target z and grasp
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  //pushWord(1048680); // assume x,y of target 
  pushWord(1114175); // assume x,y of target in local space

  pushWord("paintReticles"); // render reticle
  //pushWord("selectMaxTargetNotCumulative"); // find max on register 1
  pushWord("drawMapRegisters"); // render register 1

  pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap

  // ATTN 19 
  //pushWord(1048631); // assume best gear
  pushWord(1048678); // target best grasp
  pushWord(1048620); // find best grasp from memory

  pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  pushWord(1048695); // clear scan history
  pushWord("turnOffScanning"); // turn off scanning

  { // this sets the gripper closed thresh appropriately
    pushWord(1179713); // set gripperThresh 
    pushNoOps(30);
    pushWord("openGripper"); // open gripper
    pushNoOps(30);
    pushWord('j'); // close gripper
    pushWord('i'); // initialize gripper
  }
  calibrateGripper();
}
END_WORD




WORD(IncrementGraspGear)
CODE(196712)     // capslock + H
virtual void execute()       {
  cout << "increment currentGraspGear was is: " << currentGraspGear << " ";
  int thisGraspGear = (currentGraspGear + 1) % totalGraspGears;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  currentGraspGear = thisGraspGear;
  
  cout << currentGraspGear << endl;
}
END_WORD





WORD(ShiftGraspGear)
CODE(1114155)     // numlock + +
virtual void execute() {
  pushNoOps(50);
  int thisGraspGear = (currentGraspGear+4) % totalGraspGears;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD

WORD(PrepareToApplyGraspFilterFor1)
CODE(1048681)     // numlock + i
virtual void execute() {
  prepareGraspFilter1();
}
END_WORD

WORD(PrepareToApplyGraspFilterFor2)
CODE(1048687)     // numlock + o
virtual void execute() {
  prepareGraspFilter2();
}
END_WORD


WORD(PrepareToApplyGraspFilterFor3)
CODE(1048693)     // numlock + u
virtual void execute() {
  prepareGraspFilter3();
}
END_WORD

WORD(PrepareToApplyGraspFilterFor4)
CODE(1048688)     // numlock + p
virtual void execute() {
  prepareGraspFilter4();
}
END_WORD


WORD(SelectBestAvailableGrasp)
CODE(1048630)  // numlock + 6
virtual void execute() {
  cout << "Selecting best of 4 grasps... numlock + 6" << endl;

  pushWord("selectMaxTargetCumulative");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("prepareToApplyGraspFilterFor4");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("blur");

  pushWord("downsampleIrScan");

  pushWord("shiftIntoGraspGear4");


  pushWord("selectMaxTargetCumulative");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("prepareToApplyGraspFilterFor3");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("blur");

  pushWord("downsampleIrScan");

  pushWord("shiftIntoGraspGear3");


  pushWord("selectMaxTargetCumulative");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("prepareToApplyGraspFilterFor2");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("blur");

  pushWord("downsampleIrScan");

  pushWord("shiftIntoGraspGear2");


  pushWord("selectMaxTargetNotCumulative");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("prepareToApplyGraspFilterFor1");

  pushWord("drawMapRegisters"); 
  pushWord("applyGraspFilter");
  pushWord("blur");

  pushWord("downsampleIrScan");
  // change gear to 1
  pushWord("shiftIntoGraspGear1");
  pushWord("turnOffScanning"); // turn off scanning
}
END_WORD

WORD(SelectMaxTargetNotCumulative)
CODE(1048691)     // numlock + s
virtual void execute() {
  selectMaxTarget(VERYBIGNUMBER);
}
END_WORD

WORD(SelectMaxTargetCumulative)
CODE(1114195)     // numlock + S
virtual void execute() {
  selectMaxTarget(maxD);
}
END_WORD

WORD(ApplyGraspFilter)
CODE(1048692)  // numlock + t
virtual void execute() {
  applyGraspFilter(rangeMapReg1, rangeMapReg2);
}
END_WORD


WORD(Blur)
CODE(1048697)  // numlock + y
virtual void execute() {
  double tfilter[9] = { 1.0/16.0, 1.0/8.0, 1.0/16.0, 
                        1.0/8.0, 1.0/4.0, 1.0/8.0, 
                        1.0/16.0, 1.0/8.0, 1.0/16.0};
  for (int fx = 0; fx < 9; fx++) {
    filter[fx] = tfilter[fx];
  }
}
END_WORD

WORD(ShiftIntoGraspGear1)
CODE(1048625)      // numlock + 1
virtual void execute() {
  int thisGraspGear = 0;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD

WORD(ShiftIntoGraspGear2)
CODE(1048626)     // numlock + 2
virtual void execute() {
  int thisGraspGear = 1;
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD

WORD(ShiftIntoGraspGear3)
CODE(1048627)     // numlock + 3
virtual void execute() {
  int thisGraspGear = 2;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;

}
END_WORD


WORD(ShiftIntoGraspGear4)
CODE(1048628)      // numlock + 4
virtual void execute() {
  int thisGraspGear = 3;
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  //   rotate
  setGGRotation(thisGraspGear);
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD


WORD(TurnOffScanning)
CODE(1048684)     // numlock + l
virtual void execute() {
  recordRangeMap = 0;
}
END_WORD


WORD(ResetAerialGradientTemporalFrameAverage)
CODE(262237)      // ctrl + ]
virtual void execute() {
  cout << "resetting aerialGradientTemporalFrameAverage." << endl;
  aerialGradientTemporalFrameAverage *= 0.0;
}
END_WORD


WORD(SynchronicServoDoNotTakeClosest)
CODE(131139)  // capslock + c
virtual void execute() {
  synchronicTakeClosest = 0;
  cout << "synchronicTakeClosest = 0" << endl;
  synServoLockFrames = 0;
}
END_WORD


WORD(SynchronicServoTakeClosest)
CODE(196707)     // capslock + C
virtual void execute() {
  synchronicTakeClosest = 1;
  cout << "synchronicTakeClosest = 1" << endl;
  synServoLockFrames = 0;
}
END_WORD


WORD(TwoDPatrolStart)
CODE(131159)     // capslock + w
virtual void execute()       {
  eepReg2 = rssPose;
  bailAfterSynchronic = 0;
  graspAttemptCounter = 0;
  graspSuccessCounter = 0;
  graspTrialStart = ros::Time::now();
  thompsonPickHaltFlag = 0;
  thompsonHeightHaltFlag = 0;
  pilotTarget.px = -1;
  pilotTarget.py = -1;
  pilotClosestTarget.px = -1;
  pilotClosestTarget.py = -1;
  oscilStart = ros::Time::now();
  accumulatedTime = oscilStart - oscilStart;
  oscCenX = currentEEPose.px;
  oscCenY = currentEEPose.py;
  oscCenZ = currentEEPose.pz+0.1;
  pushWord("twoDPatrolContinue"); // 2D patrol continue
  pushWord("visionCycle");
  // we want to move to a higher holding position for visual patrol
  // so we assume that we are at 20 cm = IR scan height and move to 30 cm
  pushSpeedSign(MOVE_FAST);
  pushWord("changeToPantryTable"); // change to pantry table
  pushWord("setBoundingBoxModeToStaticMarginals"); // change bounding box inference mode to STATIC_MARGINALS
  pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
}
END_WORD

WORD(TwoDPatrolContinue)
CODE(131141) // capslock + e
virtual void execute() {
  thisGraspPicked = UNKNOWN;
  thisGraspReleased = UNKNOWN;
  neutral();
  
  if (ARE_GENERIC_PICK_LEARNING()) {
    if (thompsonHardCutoff) {
      if (graspAttemptCounter >= thompsonTries) {
        cout << "Clearing call stack because we did " << graspAttemptCounter << " tries." << endl;
        clearStack();
        pushCopies("beep", 15); // beep
        return;
      }
    }
    
    if (thompsonAdaptiveCutoff) {
      if ( (thompsonPickHaltFlag) ||
           (graspAttemptCounter >= thompsonTries) ) {
        cout << "Clearing call stack. thompsonPickHaltFlag = " << thompsonPickHaltFlag << 
          " and we did " << graspAttemptCounter << " tries." << endl;
        clearStack();
        pushCopies("beep", 15); // beep
        return;
      }
    }
  }
  
  synServoLockFrames = 0;
  currentGradientServoIterations = 0;
  
  ros::Duration delta = (ros::Time::now() - oscilStart) + accumulatedTime;
  
  currentEEPose.px = oscCenX + oscAmpX*sin(2.0*3.1415926*oscFreqX*delta.toSec());
  currentEEPose.py = oscCenY + oscAmpY*sin(2.0*3.1415926*oscFreqY*delta.toSec());
  pushWord("twoDPatrolContinue"); 
  
  // check to see if the target class is around, or take closest
  if ( ((pilotTarget.px != -1) && (pilotTarget.py != -1)) ||
       (synchronicTakeClosest && ((pilotClosestTarget.px != -1) && (pilotClosestTarget.py != -1))) )
    {
      // if so, push servoing command and set lock frames to 0
      pushWord("synchronicServo"); // synchronic servo
      
      if (targetClass != -1)
        cout << "Found the target " << classLabels[targetClass] << ". " << endl;
      // grab the last bit of accumulated time
      accumulatedTime = accumulatedTime + (ros::Time::now() - oscilStart);
    } else {
    // if not, potentially do vision and continue the 2D patrol
    
    // check and push vision cycle 
    ros::Duration timeSinceLast = ros::Time::now() - lastVisionCycle;
    if (timeSinceLast.toSec() > visionCycleInterval) {
      pushWord("visionCycle");
      // grab the last bit of accumulated time
      accumulatedTime = accumulatedTime + (ros::Time::now() - oscilStart);
    }
  }
  // if you are static_prior, this does nothing and defaults to the usual height
  pushWord("sampleHeight"); 
}
END_WORD

WORD(SynchronicServo)
CODE(131156)    // capslock + t
virtual void execute() { 
  synchronicServo();
}
END_WORD

WORD(GradientServo)
CODE(196728)   // capslock + X
virtual void execute() {
  gradientServo();
}
END_WORD



WORD(GradientServoTakeClosest)
// capslock + numlock + h
CODE(1179720)
  virtual void execute()
{
    gradientTakeClosest = 1;
    cout << "gradientTakeClosest = " << gradientTakeClosest << endl;
}
END_WORD




