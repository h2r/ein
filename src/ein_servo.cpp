WORD(FindBestOfFourGraspsUsingMemory)
CODE(1048620)     // numlock + ,
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "Selecting best of 4 grasps...  numlock + , useContinuousGraspTransform = " << useContinuousGraspTransform << endl;

  if (useContinuousGraspTransform) {
  } else {
    ms->pushWord(1179728); // estimateGlobalGraspGear
  }

  // select max target cumulative
  ms->pushWord("selectMaxTargetCumulative");
  // apply grasp filter for 4
  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter"); // apply grasp filter
  ms->pushWord("prepareToApplyGraspFilterFor4"); // prepare grasp filter for 4
  // load reg1
  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  // change gear to 4
  ms->pushWord("shiftIntoGraspGear4");

  // select max target cumulative
  ms->pushWord("selectMaxTargetCumulative");
  // apply grasp filter for 3
  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter"); // apply grasp filter
  ms->pushWord("prepareToApplyGraspFilterFor3"); // prepare grasp filter for 3
  // load reg1
  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  // change gear to 3
  ms->pushWord("shiftIntoGraspGear3");

  // select max target cumulative
  ms->pushWord("selectMaxTargetCumulative");
  // apply grasp filter for 2
  ms->pushWord("drawMapRegisters"); // drawMapRegisters
  ms->pushWord("applyGraspFilter"); // apply grasp filter
  ms->pushWord("prepareToApplyGraspFilterFor2"); // prepare to apply grasp filter for 2
  // load reg1
  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap
  // change gear to 2
  ms->pushWord("shiftIntoGraspGear2");

  // select max target NOT cumulative
  ms->pushWord("selectMaxTargetNotCumulative");


              
  // apply grasp filter for 1
  ms->pushWord("drawMapRegisters"); // drawMapRegisters 
  ms->pushWord("applyGraspFilter"); // apply grasp filter
  ms->pushWord("prepareToApplyGraspFilterFor1"); // prepare to apply grasp filter for 1
  // load reg1
  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); // load target classRangeMap

  // change gear to 1
  ms->pushWord("shiftIntoGraspGear1");

  // ATTN 10
  // loadSampled gives proper Thompson
  // loadMarginal is MAP estimate
  //ms->pushWord(131117); // loadSampledGraspMemory
  //ms->pushWord("loadMarginalGraspMemory"); // loadMarginalGraspMemory	
  switch (currentPickMode) {
  case STATIC_PRIOR:
    {
      ms->pushWord("loadMarginalGraspMemory"); // loadMarginalGraspMemory
    }
    break;
  case LEARNING_SAMPLING:
    {
      ms->pushWord(131117); // loadSampledGraspMemory
    }
    break;
  case LEARNING_ALGORITHMC:
  case STATIC_MARGINALS:
    {
      ms->pushWord("loadMarginalGraspMemory"); // loadMarginalGraspMemory
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }

  ms->pushWord("turnOffScanning"); // turn off scanning
  ms->pushWord(1179721); // set graspMemories from classGraspMemories
}
END_WORD
REGISTER_WORD(FindBestOfFourGraspsUsingMemory)




WORD(AssumeWinningGgAndXyInLocalPose)
CODE(1114175)     // numlock + ?
virtual void execute(std::shared_ptr<MachineState> ms)       {
  double targetX = ms->config.trX;
  double targetY = ms->config.trY;

  ms->config.trZ = ms->config.rangeMapReg1[ms->config.maxX + ms->config.maxY*ms->config.rmWidth];

  ms->config.currentEEPose.px = targetX;
  ms->config.currentEEPose.py = targetY;
      
  cout << "Assuming x,y,gear: " << targetX << " " << targetY << " " << ms->config.maxGG << endl;

  //ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  // ATTN 19
  if (useContinuousGraspTransform) {
    cout << "Assuming continuous maxGG: " << ms->config.maxGG << " localMaxGG: " << ms->config.localMaxGG << endl;
    setCCRotation(ms, (ms->config.maxGG+4)%4); 
  } else {
    ms->pushWord(1048631); // assume best gear
  }
}
END_WORD
REGISTER_WORD(AssumeWinningGgAndXyInLocalPose)

WORD(MoveToTargetZAndGrasp)
CODE(1048682)     // numlock + j
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->pushWord("closeGripper"); 

  double threshedZ = min(ms->config.trZ, 0.0);

  double pickZpre = -(threshedZ + ms->config.currentTableZ) + pickFlushFactor + graspDepthOffset;
  double flushZ = -(ms->config.currentTableZ) + pickFlushFactor;
  double pickZ = max(flushZ, pickZpre);

  int useIncrementalPick = 0;
  bool useHybridPick = 1;

  double deltaZ = pickZ - ms->config.currentEEPose.pz;
  lastPickPose = ms->config.currentEEPose;
  lastPickPose.pz = pickZ;



  cout << "moveToTargetZAndGrasp trZ pickZ flushZ pickZpre: " << ms->config.trZ << " " << pickZ << " " << flushZ << " " << pickZpre << " " << endl;

  if (useIncrementalPick) {
    double zTimes = fabs(floor(deltaZ / ms->config.bDelta)); 

    int numNoOps = 2;
    if (deltaZ > 0) {
      for (int zc = 0; zc < zTimes; zc++) {
	for (int cc = 0; cc < numNoOps; cc++) {
	  ms->pushWord('C');
	}
	ms->pushWord('w');
      }
    }
    if (deltaZ < 0) {
      for (int zc = 0; zc < zTimes; zc++) {
	for (int cc = 0; cc < numNoOps; cc++) {
	  ms->pushWord('C');
	}
	ms->pushWord('s');
      }
    }
  } else {
    if (useHybridPick) {
      int pickNoops = 20;
      int increments = 0.1/MOVE_FAST;
      ms->config.currentEEPose.pz = pickZ+increments*MOVE_FAST;

      //ms->pushCopies("endStackCollapseNoop", pickNoops);
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushCopies('s', increments);
      ms->pushWord("setMovementSpeedMoveFast");
      ms->pushWord("approachSpeed");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      //ms->pushWord("quarterImpulse");
      ms->pushWord("approachSpeed");
    } else {
      ms->config.currentEEPose.pz = pickZ;
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    }
  }
}
END_WORD
REGISTER_WORD(MoveToTargetZAndGrasp)

WORD(ShakeItUpAndDown)
CODE(131081)   // capslock + tab
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.eepReg5 = ms->config.currentEEPose;

  ms->config.eepReg6 = ms->config.currentEEPose;
  ms->config.eepReg6.pz += 0.2;

  pushSpeedSign(ms, MOVE_FAST);    
  if (isGripperGripping()) {
    happy();
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('5');  // assume pose at register 5
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('6'); // assume pose at register 6
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('5'); // assume pose at register 5
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('6');

  }

  pushSpeedSign(ms, NOW_THATS_FAST);

}
END_WORD
REGISTER_WORD(ShakeItUpAndDown)

WORD(TryToMoveToTheLastPrePickHeight)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose.pz = lastPrePickPose.pz;
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  cout << "trying to move to the last pre pick height..." << endl;
}
END_WORD
REGISTER_WORD(TryToMoveToTheLastPrePickHeight)

WORD(TryToMoveToTheLastPickHeight)
CODE( 262241)     // ctrl + a
virtual void execute(std::shared_ptr<MachineState> ms) {
  double deltaZ = (lastPickPose.pz) - ms->config.currentEEPose.pz;
  double zTimes = fabs(floor(deltaZ / ms->config.bDelta)); 
  int numNoOps = 2;
  int useIncrementalPlace = 0;
  bool useHybridPlace = 1;
  if (useIncrementalPlace) {
    if (deltaZ > 0) {
      for (int zc = 0; zc < zTimes; zc++) {
	for (int cc = 0; cc < numNoOps; cc++) {
	  ms->pushWord("endStackCollapseNoop");
	}
	ms->pushWord('w');
      }
    }
    if (deltaZ < 0) {
      for (int zc = 0; zc < zTimes; zc++) {
	for (int cc = 0; cc < numNoOps; cc++) {
	  ms->pushWord("endStackCollapseNoop");
	}
	ms->pushWord('s');
      }
    }
  } else {
    if (useHybridPlace) {
      int pickNoops = 20;
      int increments = 0.1/MOVE_FAST;
      ms->config.currentEEPose.pz = lastPickPose.pz+increments*MOVE_FAST;

      //ms->pushCopies("endStackCollapseNoop", pickNoops);
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushCopies('s', increments);
      ms->pushWord("setMovementSpeedMoveFast");
      ms->pushWord("approachSpeed");
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      //ms->pushWord("quarterImpulse");
      ms->pushWord("approachSpeed");
    } else {
      ms->config.currentEEPose.pz = lastPickPose.pz;
      ms->pushWord("waitUntilAtCurrentPosition"); 
    }
  }
  cout << "trying to move to the last pick height..." << endl;
}
END_WORD
REGISTER_WORD(TryToMoveToTheLastPickHeight)

WORD(AssertYesGrasp)
CODE( 131157)     // capslock + u
    // if gripper is empty, pop next instruction and return. if not, just return
    // pull from bag will push itself and assert yes grasp
    // assert yes grasp
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "assert yes grasp: " << gripperMoving << " " << gripperGripping << " " << gripperPosition << endl;
  // TODO push this and then a calibration message if uncalibrated
  // push this again if moving
  if (gripperMoving) {
    ms->pushWord("assertYesGrasp"); // assert yes grasp
  } else {
    if (isGripperGripping())
      {
        ms->popWord();
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
REGISTER_WORD(AssertYesGrasp)

WORD(IfNoGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (isGripperGripping())  {
    ms->popWord();
  }
}
END_WORD
REGISTER_WORD(IfNoGrasp)

WORD(IfGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (!isGripperGripping())  {
    ms->popWord();
  }
}
END_WORD
REGISTER_WORD(IfGrasp)



WORD(AssertNoGrasp)
CODE(196649)     // capslock + i
// if gripper is full, pop next instruction and return. if not, just return
// shake it off will push itself and then assert no grasp
  virtual void execute(std::shared_ptr<MachineState> ms) {
  // TODO push this and then a calibration message if uncalibrated
  // push this again if moving

  cout << "assert no grasp: " << gripperMoving << " " << gripperGripping << " " << gripperPosition << endl;

  if (gripperMoving) {
    ms->pushWord("assertNoGrasp"); // assert no grasp
  } else {
    if (!isGripperGripping())  {
      ms->popWord();
      // leave gripper in released state
      cout << "  assert no pops back instruction." << endl;
      if (thisGraspReleased == UNKNOWN) {
        thisGraspReleased = SUCCESS;
      }
    } else {
      // stuck
      ms->pushWord('Y'); // pause stack execution
      ms->pushCopies("beep", 15); // beep
      cout << "Stuck, please reset the object. ";
      cout << " gripperPosition: " << gripperPosition;
      cout << " gripperThresh: " << gripperThresh << endl;
      if (thisGraspReleased == UNKNOWN) {
        thisGraspReleased = FAILURE;
        sad();
      }
    }
    ms->pushWord("openGripper"); // open gripper

  }

}
END_WORD
REGISTER_WORD(AssertNoGrasp)

WORD(ShakeItOff1)
CODE( 131151)     // capslock + o
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  int depthToPlunge = 24;
  int flexThisFar = 80;
  cout << "SHAKING IT OFF!!!" << endl;
  ms->pushWord(196719); // shake it off 2
  ms->pushWord("assertNoGrasp"); // assert no grasp

  ms->pushNoOps(60);
  //ms->pushWord("moveToRegister2"); // assume pose at register 2
  ms->pushWord("closeGripper"); // close gripper
  ms->pushNoOps(20);
  ms->pushWord("openGripper"); // open gripper
  ms->pushWord("closeGripper"); // close gripper
  ms->pushNoOps(20);
  //ms->pushCopies('w', depthToPlunge); // move up 
  ms->pushWord("openGripper"); // open gripper
  ms->pushWord("closeGripper"); // close gripper
  //ms->pushNoOps(20);
  //ms->pushCopies("zDown"+65504, flexThisFar); // rotate forward
  //ms->pushCopies('e', 5); // move forward
  //ms->pushCopies('s', depthToPlunge); // move down
  ms->pushWord("openGripper"); // open gripper
  ms->pushNoOps(50);
  //ms->pushCopies('w'+65504, flexThisFar); // rotate forward

  //ms->pushWord("moveToRegister2"); // assume pose at register 2
  pushSpeedSign(ms, MOVE_FAST);

  // resets the gripper server
  //int sis = system("bash -c \"echo -e \'cC\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  //calibrateGripper();
}
END_WORD
REGISTER_WORD(ShakeItOff1)


WORD(LoadTargetClassRangeMapIntoRegister1)
CODE(131162)     // capslock + z
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadGlobalTargetClassRangeMap(ms, ms->config.rangeMap, ms->config.rangeMapReg1);
}
END_WORD
REGISTER_WORD(LoadTargetClassRangeMapIntoRegister1)

WORD(CountGrasp)
CODE(196717)     // capslock + M
virtual void execute(std::shared_ptr<MachineState> ms)       {
  // ATTN 19
  int i = ms->config.localMaxX + ms->config.localMaxY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.localMaxGG;
  int j = ms->config.localMaxX + ms->config.localMaxY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0;
  graspAttemptCounter++;      
  cout << "thisGraspPicked: " << operationStatusToString(thisGraspPicked) << endl;
  cout << "thisGraspReleased: " << operationStatusToString(thisGraspReleased) << endl;
  if (ARE_GENERIC_PICK_LEARNING()) {
    //ms->config.graspMemoryTries[j+0*ms->config.rmWidth*ms->config.rmWidth]++;
    //ms->config.graspMemoryTries[j+1*ms->config.rmWidth*ms->config.rmWidth]++;
    //ms->config.graspMemoryTries[j+2*ms->config.rmWidth*ms->config.rmWidth]++;
    //ms->config.graspMemoryTries[j+3*ms->config.rmWidth*ms->config.rmWidth]++;
    if (ms->config.graspMemoryTries[i] <= 1.0) {
      ms->config.graspMemoryTries[i] = 1.001;
      ms->config.graspMemoryPicks[i] = 0.0;
    } else {
      ms->config.graspMemoryTries[i]++;
    }
  }
  if ((thisGraspPicked == SUCCESS) && (thisGraspReleased == SUCCESS)) {
    graspSuccessCounter++;
    happy();
    if (ARE_GENERIC_PICK_LEARNING()) {
      //ms->config.graspMemoryPicks[j+0*ms->config.rmWidth*ms->config.rmWidth]++;
      //ms->config.graspMemoryPicks[j+1*ms->config.rmWidth*ms->config.rmWidth]++;
      //ms->config.graspMemoryPicks[j+2*ms->config.rmWidth*ms->config.rmWidth]++;
      //ms->config.graspMemoryPicks[j+3*ms->config.rmWidth*ms->config.rmWidth]++;
      ms->config.graspMemoryPicks[i]++;
    }
        
    if (ARE_GENERIC_HEIGHT_LEARNING()) {
      recordBoundingBoxSuccess(ms);
    }

    double thisPickRate = double(ms->config.graspMemoryPicks[i]) / double(ms->config.graspMemoryTries[i]);
    int thisNumTries = ms->config.graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
    if (currentPickMode == LEARNING_SAMPLING) {
      if ( (thisNumTries >= thompsonMinTryCutoff) && 
           (thisPickRate >= thompsonMinPassRate) ) {
        thompsonPickHaltFlag = 1;
      }
    }
    // ATTN 20
    {
      double successes = ms->config.graspMemoryPicks[i];
      double failures =  ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i];
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
    double thisPickRate = double(ms->config.graspMemoryPicks[i]) / double(ms->config.graspMemoryTries[i]);
    int thisNumTries = ms->config.graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
    sad();
    if (ARE_GENERIC_HEIGHT_LEARNING()) {
      recordBoundingBoxFailure(ms);
    }
  }
  copyGraspMemoryTriesToClassGraspMemoryTries(ms);
  graspSuccessRate = graspSuccessCounter / graspAttemptCounter;
  ros::Time thisTime = ros::Time::now();
  ros::Duration sinceStartOfTrial = thisTime - graspTrialStart;
      
  cout << "<><><><> Grasp attempts rate time gripperPosition currentPickMode: " << graspSuccessCounter << "/" << graspAttemptCounter << " " << graspSuccessRate << " " << sinceStartOfTrial.toSec() << " seconds " << " " << pickModeToString(currentPickMode) << endl;
  {
    double successes = ms->config.graspMemoryPicks[i];
    double failures =  ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i];
    cout << "YYY failures, successes: " << failures << " " << successes << endl;
    successes = round(successes);
    failures = round(failures);
    cout << "XXX failures, successes: " << failures << " " << successes << endl;
  }
}
END_WORD
REGISTER_WORD(CountGrasp)

WORD(CheckGrasp)
CODE(196718)     // capslock + N 
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  if (gripperMoving) {
    ms->pushWord("checkGrasp"); // check grasp
  } else {
    cout << "gripperPosition: " << gripperPosition << " gripperThresh: " << gripperThresh << endl;
    cout << "gripperGripping: " << gripperGripping << endl;
    if (!isGripperGripping()) {
      cout << "Failed to pick." << endl;
      thisGraspPicked = FAILURE;
      sad();
      ms->pushCopies("beep", 15); // beep
    } else {
      cout << "Successful pick." << endl;
      thisGraspPicked = SUCCESS;
      happy();
    }
  }
}
END_WORD
REGISTER_WORD(CheckGrasp)

WORD(CheckAndCountGrasp)
CODE(196713)     // capslock + I
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  int i = ms->config.localMaxX + ms->config.localMaxY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.localMaxGG;
  int j = ms->config.localMaxX + ms->config.localMaxY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0;
  if (gripperMoving) {
    ms->pushWord(196713); // check and count grasp
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
        if (ms->config.graspMemoryTries[i] <= 1.0) {
          ms->config.graspMemoryTries[i] = 1.001;
          ms->config.graspMemoryPicks[i] = 0.0;
        } else {
          ms->config.graspMemoryTries[i]++;
        }
        //ms->config.graspMemoryTries[j+0*ms->config.rmWidth*ms->config.rmWidth]++;
        //ms->config.graspMemoryTries[j+1*ms->config.rmWidth*ms->config.rmWidth]++;
        //ms->config.graspMemoryTries[j+2*ms->config.rmWidth*ms->config.rmWidth]++;
        //ms->config.graspMemoryTries[j+3*ms->config.rmWidth*ms->config.rmWidth]++;
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
        recordBoundingBoxFailure(ms);
      }
      cout << "Failed grasp." << endl;
      //ms->pushWord('Y'); // pause stack execution
      ms->pushCopies("beep", 15); // beep
    } else {
      if (ARE_GENERIC_HEIGHT_LEARNING()) {
        recordBoundingBoxSuccess(ms);
      }
      graspSuccessCounter++;
      cout << "Successful grasp." << endl;
      //ms->config.graspMemoryPicks[i]++;
      switch (currentPickMode) {
      case STATIC_PRIOR:
        {
        }
        break;
      case LEARNING_ALGORITHMC:
      case LEARNING_SAMPLING:
        {
          ms->config.graspMemoryPicks[i]++;
          //ms->config.graspMemoryPicks[j]++;
          //ms->config.graspMemoryPicks[j+1*ms->config.rmWidth*ms->config.rmWidth]++;
          //ms->config.graspMemoryPicks[j+2*ms->config.rmWidth*ms->config.rmWidth]++;
          //ms->config.graspMemoryPicks[j+3*ms->config.rmWidth*ms->config.rmWidth]++;
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

    double thisPickRate = double(ms->config.graspMemoryPicks[i]) / double(ms->config.graspMemoryTries[i]);
    int thisNumTries = ms->config.graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;

    if (currentPickMode == LEARNING_SAMPLING) {
      if ( (thisNumTries >= thompsonMinTryCutoff) && 
           (thisPickRate >= thompsonMinPassRate) ) {
        thompsonPickHaltFlag = 1;
      }
    }
    // ATTN 20
    {
      double successes = ms->config.graspMemoryPicks[i];
      double failures =  ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i];
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

    copyGraspMemoryTriesToClassGraspMemoryTries(ms);
    graspSuccessRate = graspSuccessCounter / graspAttemptCounter;
    ros::Time thisTime = ros::Time::now();
    ros::Duration sinceStartOfTrial = thisTime - graspTrialStart;
    cout << "<><><><> Grasp attempts rate time gripperPosition currentPickMode: " << graspSuccessCounter << "/" << graspAttemptCounter << " " << graspSuccessRate << " " << sinceStartOfTrial.toSec() << " seconds " << gripperPosition << " " << pickModeToString(currentPickMode) << endl;
  }
  {
    double successes = ms->config.graspMemoryPicks[i];
    double failures =  ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i];
    cout << "YYY failures, successes: " << failures << " " << successes << endl;
    successes = round(successes);
    failures = round(failures);
    cout << "XXX failures, successes: " << failures << " " << successes << endl;
  }
}
END_WORD
REGISTER_WORD(CheckAndCountGrasp)



// XX We should get rid of this one and put the shake and count behaviors elsewhere. 
WORD(PrepareForAndExecuteGraspFromMemoryLearning)
virtual void execute(std::shared_ptr<MachineState> ms)       {

  ms->pushWord("visionCycle"); // vision cycle
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("sampleHeight"); // sample height
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear

  ms->pushWord("countGrasp"); //count grasp

  ms->pushWord("openGripper"); // open gripper
  ms->pushWord("shakeItOff1"); // shake it off 1
  ms->pushWord("assertNoGrasp"); // assert no grasp

  ms->pushNoOps(30);
  ms->pushWord("closeGripper"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushCopies("zUp", 10);
  ms->pushNoOps(30);
  ms->pushWord("openGripper"); 

  ms->pushNoOps(5);
  ms->pushWord("tryToMoveToTheLastPickHeight"); 
  //ms->pushWord("approachSpeed"); 

  //count here so that if it drops it on the way it will count as a miss
  { // in case it fell out
    ms->pushWord("checkGrasp");

    ms->pushNoOps(30);
    ms->pushWord("closeGripper"); // close gripper
    ms->pushWord("shakeItUpAndDown"); // shake it up and down

    ms->pushNoOps(5);
    ms->pushWord("closeGripper"); // close gripper
  }

  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  if (ARE_GENERIC_HEIGHT_LEARNING()) {
    ms->pushWord("setRandomPositionAndOrientationForHeightLearning"); // set random position for bblearn
  } else {
    ms->pushWord("perturbPosition"); // numlock + /
  }

  ms->pushCopies("zDown", 3);
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  if (ARE_GENERIC_HEIGHT_LEARNING()) {
    ms->pushWord("moveToRegister4"); // assume pose at register 4
  } else {
    ms->pushWord("moveToRegister2"); // assume pose at register 2
  }

  ms->pushWord("quarterImpulse"); 
  ms->pushNoOps(10);

  ms->pushWord("moveToTargetZAndGrasp"); 
  ms->pushWord("approachSpeed"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("assumeWinningGgAndXyInLocalPose"); 

  ms->pushWord("paintReticles"); 

  ms->pushWord("drawMapRegisters"); 

  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); 



  ms->pushWord("setTargetReticleToTheMaxMappedPosition");
  ms->pushWord("findBestOfFourGraspsUsingMemory"); 

  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); 

  // ATTN 23
  {
    ms->pushWord("setRangeMapCenterFromCurrentEEPose"); 
    ms->pushWord("initDepthScan"); 
  }
  ms->pushWord("initDepthScan"); 
  ms->pushWord("turnOffScanning"); 

  ms->pushWord("openGripper"); 

}
END_WORD
REGISTER_WORD(PrepareForAndExecuteGraspFromMemoryLearning)




WORD(PrepareForAndExecuteGraspFromMemory)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("executePreparedGrasp"); 
  ms->pushWord("prepareForGraspFromMemory"); 
}
END_WORD
REGISTER_WORD(PrepareForAndExecuteGraspFromMemory)

WORD(ExecutePreparedGrasp)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  if (ARE_GENERIC_HEIGHT_LEARNING()) {
    ms->pushWord("moveToRegister4"); // assume pose at register 4
  }

  //ms->pushWord("quarterImpulse"); 
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("comeToHover"); 
  ms->pushWord("waitUntilGripperNotMoving");
  ms->pushWord("moveToTargetZAndGrasp"); 
  ms->pushWord("approachSpeed"); 
  ms->pushWord("openGripper"); 
}
END_WORD
REGISTER_WORD(ExecutePreparedGrasp)

WORD(PrepareForGraspFromMemory)
virtual void execute(std::shared_ptr<MachineState> ms) {

  //ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("assumeWinningGgAndXyInLocalPose"); 

  ms->pushWord("paintReticles"); 

  ms->pushWord("drawMapRegisters"); 

  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); 

  ms->pushWord("setTargetReticleToTheMaxMappedPosition");
  ms->pushWord("findBestOfFourGraspsUsingMemory"); 

  ms->pushWord("loadTargetClassRangeMapIntoRegister1"); 

  // ATTN 23
  {
    ms->pushWord("setRangeMapCenterFromCurrentEEPose"); 
    ms->pushWord("initDepthScan"); 
  }
  //ms->pushWord("initDepthScan"); 

  ms->pushWord("turnOffScanning"); 

  ms->pushWord("openGripper"); 
}
END_WORD
REGISTER_WORD(PrepareForGraspFromMemory)



WORD(IncrementGraspGear)
CODE(196712)     // capslock + H
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "increment ms->config.currentGraspGear was is: " << ms->config.currentGraspGear << " ";
  int thisGraspGear = (ms->config.currentGraspGear + 1) % ms->config.totalGraspGears;
  
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(ms, thisGraspGear);
  ms->config.currentGraspGear = thisGraspGear;
  
  cout << ms->config.currentGraspGear << endl;
}
END_WORD
REGISTER_WORD(IncrementGraspGear)




WORD(ShiftGraspGear)
CODE(1114155)     // numlock + +
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushNoOps(50);
  int thisGraspGear = (ms->config.currentGraspGear+4) % ms->config.totalGraspGears;
  
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(ms, thisGraspGear);
  
  //   set ms->config.currentGraspGear;
  ms->config.currentGraspGear = thisGraspGear;
}
END_WORD
REGISTER_WORD(ShiftGraspGear)

WORD(PrepareToApplyGraspFilterFor1)
CODE(1048681)     // numlock + i
virtual void execute(std::shared_ptr<MachineState> ms) {
  prepareGraspFilter1(ms);
}
END_WORD
REGISTER_WORD(PrepareToApplyGraspFilterFor1)

WORD(PrepareToApplyGraspFilterFor2)
CODE(1048687)     // numlock + o
virtual void execute(std::shared_ptr<MachineState> ms) {
  prepareGraspFilter2(ms);
}
END_WORD
REGISTER_WORD(PrepareToApplyGraspFilterFor2)

WORD(PrepareToApplyGraspFilterFor3)
CODE(1048693)     // numlock + u
virtual void execute(std::shared_ptr<MachineState> ms) {
  prepareGraspFilter3(ms);
}
END_WORD
REGISTER_WORD(PrepareToApplyGraspFilterFor3)

WORD(PrepareToApplyGraspFilterFor4)
CODE(1048688)     // numlock + p
virtual void execute(std::shared_ptr<MachineState> ms) {
  prepareGraspFilter4(ms);
}
END_WORD
REGISTER_WORD(PrepareToApplyGraspFilterFor4)

WORD(SelectBestAvailableGrasp)
CODE(1048630)  // numlock + 6
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Selecting best of 4 grasps... numlock + 6" << endl;

  ms->pushWord("selectMaxTargetCumulative");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("prepareToApplyGraspFilterFor4");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("blur");

  ms->pushWord("downsampleIrScan");

  ms->pushWord("shiftIntoGraspGear4");


  ms->pushWord("selectMaxTargetCumulative");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("prepareToApplyGraspFilterFor3");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("blur");

  ms->pushWord("downsampleIrScan");

  ms->pushWord("shiftIntoGraspGear3");


  ms->pushWord("selectMaxTargetCumulative");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("prepareToApplyGraspFilterFor2");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("blur");

  ms->pushWord("downsampleIrScan");

  ms->pushWord("shiftIntoGraspGear2");


  ms->pushWord("selectMaxTargetNotCumulative");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("prepareToApplyGraspFilterFor1");

  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("applyGraspFilter");
  ms->pushWord("blur");

  ms->pushWord("downsampleIrScan");
  // change gear to 1
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("turnOffScanning"); // turn off scanning
}
END_WORD
REGISTER_WORD(SelectBestAvailableGrasp)

WORD(SelectMaxTargetNotCumulative)
CODE(1048691)     // numlock + s
virtual void execute(std::shared_ptr<MachineState> ms) {
  selectMaxTarget(ms, VERYBIGNUMBER);
}
END_WORD
REGISTER_WORD(SelectMaxTargetNotCumulative)

WORD(SelectMaxTargetCumulative)
CODE(1114195)     // numlock + S
virtual void execute(std::shared_ptr<MachineState> ms) {
  selectMaxTarget(ms, ms->config.maxD);
}
END_WORD
REGISTER_WORD(SelectMaxTargetCumulative)

WORD(ApplyGraspFilter)
CODE(1048692)  // numlock + t
virtual void execute(std::shared_ptr<MachineState> ms) {
  applyGraspFilter(ms, ms->config.rangeMapReg1, ms->config.rangeMapReg2);
}
END_WORD
REGISTER_WORD(ApplyGraspFilter)


WORD(Blur)
CODE(1048697)  // numlock + y
virtual void execute(std::shared_ptr<MachineState> ms) {
  double tfilter[9] = { 1.0/16.0, 1.0/8.0, 1.0/16.0, 
                        1.0/8.0, 1.0/4.0, 1.0/8.0, 
                        1.0/16.0, 1.0/8.0, 1.0/16.0};
  for (int fx = 0; fx < 9; fx++) {
    ms->config.filter[fx] = tfilter[fx];
  }
}
END_WORD
REGISTER_WORD(Blur)

WORD(ShiftIntoGraspGear1)
CODE(1048625)      // numlock + 1
virtual void execute(std::shared_ptr<MachineState> ms) {
  int thisGraspGear = 0;
  
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(ms, thisGraspGear);
  
  //   set ms->config.currentGraspGear;
  ms->config.currentGraspGear = thisGraspGear;
}
END_WORD
REGISTER_WORD(ShiftIntoGraspGear1)

WORD(ShiftIntoGraspGear2)
CODE(1048626)     // numlock + 2
virtual void execute(std::shared_ptr<MachineState> ms) {
  int thisGraspGear = 1;
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(ms, thisGraspGear);
  
  //   set ms->config.currentGraspGear;
  ms->config.currentGraspGear = thisGraspGear;
}
END_WORD
REGISTER_WORD(ShiftIntoGraspGear2)

WORD(ShiftIntoGraspGear3)
CODE(1048627)     // numlock + 3
virtual void execute(std::shared_ptr<MachineState> ms) {
  int thisGraspGear = 2;
  
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(ms, thisGraspGear);
  
  //   set ms->config.currentGraspGear;
  ms->config.currentGraspGear = thisGraspGear;

}
END_WORD
REGISTER_WORD(ShiftIntoGraspGear3)

WORD(ShiftIntoGraspGear4)
CODE(1048628)      // numlock + 4
virtual void execute(std::shared_ptr<MachineState> ms) {
  int thisGraspGear = 3;
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  //   rotate
  setGGRotation(ms, thisGraspGear);
  //   set ms->config.currentGraspGear;
  ms->config.currentGraspGear = thisGraspGear;
}
END_WORD
REGISTER_WORD(ShiftIntoGraspGear4)

WORD(TurnOffScanning)
CODE(1048684)     // numlock + l
virtual void execute(std::shared_ptr<MachineState> ms) {
  recordRangeMap = 0;
}
END_WORD
REGISTER_WORD(TurnOffScanning)

WORD(ResetAerialGradientTemporalFrameAverage)
CODE(262237)      // ctrl + ]
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "resetting aerialGradientTemporalFrameAverage." << endl;
  aerialGradientTemporalFrameAverage *= 0.0;
}
END_WORD
REGISTER_WORD(ResetAerialGradientTemporalFrameAverage)


WORD(SynchronicServoDoNotTakeClosest)
CODE(131139)  // capslock + c
virtual void execute(std::shared_ptr<MachineState> ms) {
  synchronicTakeClosest = 0;
  cout << "synchronicTakeClosest = 0" << endl;
  synServoLockFrames = 0;
}
END_WORD
REGISTER_WORD(SynchronicServoDoNotTakeClosest)

WORD(SynchronicServoTakeClosest)
CODE(196707)     // capslock + C
virtual void execute(std::shared_ptr<MachineState> ms) {
  synchronicTakeClosest = 1;
  cout << "synchronicTakeClosest = 1" << endl;
  synServoLockFrames = 0;
}
END_WORD
REGISTER_WORD(SynchronicServoTakeClosest)


WORD(TwoDPatrolStart)
CODE(131159)     // capslock + w
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->config.eepReg2 = ms->config.beeHome;
  bailAfterSynchronic = 0;
  graspAttemptCounter = 0;
  graspSuccessCounter = 0;
  graspTrialStart = ros::Time::now();
  thompsonPickHaltFlag = 0;
  thompsonHeightHaltFlag = 0;
  ms->config.pilotTarget.px = -1;
  ms->config.pilotTarget.py = -1;
  ms->config.pilotClosestTarget.px = -1;
  ms->config.pilotClosestTarget.py = -1;
  oscilStart = ros::Time::now();
  accumulatedTime = oscilStart - oscilStart;
  oscCenX = ms->config.currentEEPose.px;
  oscCenY = ms->config.currentEEPose.py;
  oscCenZ = ms->config.currentEEPose.pz+0.1;
  ms->pushWord("twoDPatrolContinue"); // 2D patrol continue
  ms->pushWord("visionCycle");
  // we want to move to a higher holding position for visual patrol
  // so we assume that we are at 20 cm = IR scan height and move to 30 cm
  pushSpeedSign(ms, MOVE_FAST);
  ms->pushWord("changeToPantryTable"); // change to pantry table
  //ms->pushWord("setBoundingBoxModeToStaticMarginals"); 
  ms->pushWord("setBoundingBoxModeToStaticPrior"); 
  ms->pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
  ms->pushWord("quarterImpulse"); 
}
END_WORD
REGISTER_WORD(TwoDPatrolStart)

WORD(TwoDPatrolContinue)
CODE(131141) // capslock + e
virtual void execute(std::shared_ptr<MachineState> ms) {
  thisGraspPicked = UNKNOWN;
  thisGraspReleased = UNKNOWN;
  neutral();
  
  if (ARE_GENERIC_PICK_LEARNING()) {
    if (thompsonHardCutoff) {
      if (graspAttemptCounter >= ms->config.thompsonTries) {
        cout << "Clearing call stack because we did " << graspAttemptCounter << " tries." << endl;
        ms->clearStack();
        ms->pushCopies("beep", 15); // beep
        return;
      }
    }
    
    if (thompsonAdaptiveCutoff) {
      if ( (thompsonPickHaltFlag) ||
           (graspAttemptCounter >= ms->config.thompsonTries) ) {
        cout << "Clearing call stack. thompsonPickHaltFlag = " << thompsonPickHaltFlag << 
          " and we did " << graspAttemptCounter << " tries." << endl;
        ms->clearStack();
        ms->pushCopies("beep", 15); // beep
        return;
      }
    }
  }
  
  synServoLockFrames = 0;
  currentGradientServoIterations = 0;
  
  ros::Duration delta = (ros::Time::now() - oscilStart) + accumulatedTime;
  
  ms->config.currentEEPose.px = oscCenX + oscAmpX*sin(2.0*3.1415926*oscFreqX*delta.toSec());
  ms->config.currentEEPose.py = oscCenY + oscAmpY*sin(2.0*3.1415926*oscFreqY*delta.toSec());
  ms->pushWord("twoDPatrolContinue"); 
  
  // check to see if the target class is around, or take closest
  if ( ((ms->config.pilotTarget.px != -1) && (ms->config.pilotTarget.py != -1)) ||
       (synchronicTakeClosest && ((ms->config.pilotClosestTarget.px != -1) && (ms->config.pilotClosestTarget.py != -1))) )
    {
      // if so, push servoing command and set lock frames to 0
      ms->pushWord("synchronicServo"); // synchronic servo
      
      if (targetClass != -1)
        cout << "Found the target " << classLabels[targetClass] << ". " << endl;
      // grab the last bit of accumulated time
      accumulatedTime = accumulatedTime + (ros::Time::now() - oscilStart);
    } else {
    // if not, potentially do vision and continue the 2D patrol
    
    // check and push vision cycle 
    ros::Duration timeSinceLast = ros::Time::now() - lastVisionCycle;
    if (timeSinceLast.toSec() > visionCycleInterval) {
      ms->pushWord("visionCycle");
      // grab the last bit of accumulated time
      accumulatedTime = accumulatedTime + (ros::Time::now() - oscilStart);
    }
  }
  // if you are static_prior, this does nothing and defaults to the usual height
  ms->pushWord("sampleHeight"); 
  ms->pushWord("quarterImpulse"); 
}
END_WORD
REGISTER_WORD(TwoDPatrolContinue)

WORD(SynchronicServo)
CODE(131156)    // capslock + t
virtual void execute(std::shared_ptr<MachineState> ms) { 
  ms->pushWord("synchronicServoA");
  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
}
END_WORD
REGISTER_WORD(SynchronicServo)

WORD(SynchronicServoA)
virtual void execute(std::shared_ptr<MachineState> ms) { 
  synchronicServo(ms);
}
END_WORD
REGISTER_WORD(SynchronicServoA)

WORD(GradientServo)
CODE(196728)   // capslock + X
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("gradientServoA");
  ms->pushWord("gradientServoPrep");
}
END_WORD
REGISTER_WORD(GradientServo)

WORD(GradientServoPrep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // ATTN 8
  if (0) {
    ms->pushCopies("density", densityIterationsForGradientServo); 
    //ms->pushCopies("accumulateDensity", densityIterationsForGradientServo); 
    //ms->pushCopies("resetTemporalMap", 1); 
    ms->pushWord("resetAerialGradientTemporalFrameAverage"); 
    ms->pushCopies("density", 1); 
    //ms->pushCopies("waitUntilAtCurrentPosition", 5); 
    ms->pushWord("hover");
  }

  // ATTN 23
  {
    ms->pushWord("accumulatedDensity");
    //ms->pushCopies("waitUntilImageCallbackReceived", 10);
    ms->pushCopies("waitUntilImageCallbackReceived", 10);
    ms->pushWord("resetAccumulatedDensity");
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
  }

  ms->pushWord("waitUntilAtCurrentPosition"); 
}
END_WORD
REGISTER_WORD(GradientServoPrep)

WORD(GradientServoA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  gradientServo(ms);
}
END_WORD
REGISTER_WORD(GradientServoA)

WORD(GradientServoIfBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (bLabels.size() > 0) && (ms->config.pilotClosestBlueBoxNumber != -1) ) {
    changeTargetClass(ms, bLabels[ms->config.pilotClosestBlueBoxNumber]);
    ms->pushWord("gradientServo");
  }
}
END_WORD
REGISTER_WORD(GradientServoIfBlueBoxes)

WORD(LockTargetIfBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (bLabels.size() > 0) && (ms->config.pilotClosestBlueBoxNumber != -1) ) {
    ms->pushWord("recordTargetLock");
    ms->pushWord("prepareForGraspFromMemory");
  }
}
END_WORD
REGISTER_WORD(LockTargetIfBlueBoxes)

WORD(RecordTargetLock)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (blueBoxMemories.size() > 0) {
    BoxMemory *lastAdded = &(blueBoxMemories[blueBoxMemories.size()-1]);
    lastAdded->cameraTime = ros::Time::now();
    lastAdded->aimedPose = ms->config.currentEEPose;
    // XXX picked pose doesn't seem to mean anything here so likely doesn't matteer
    lastAdded->pickedPose = ms->config.currentEEPose;
    lastAdded->pickedPose.pz  = lastPickPose.pz;
    // XXX picked pose doesn't seem to mean anything here so likely doesn't matteer
    lastAdded->trZ  = ms->config.trZ;
    cout << "recordTargetLock saving pickedPose..." << endl;
    cout << "trZ = " << ms->config.trZ << endl;
    cout << "Current EE Position (x,y,z): " << ms->config.currentEEPose.px << " " << ms->config.currentEEPose.py << " " << ms->config.currentEEPose.pz << endl;
    cout << "Current EE Orientation (x,y,z,w): " << ms->config.currentEEPose.qx << " " << ms->config.currentEEPose.qy << " " << ms->config.currentEEPose.qz << " " << ms->config.currentEEPose.qw << endl;
    lastAdded->lockStatus = POSE_LOCK;
    
  }
}
END_WORD
REGISTER_WORD(RecordTargetLock)



WORD(GradientServoTakeClosest)
// capslock + numlock + h
CODE(1179720)
virtual void execute(std::shared_ptr<MachineState> ms)
{
    gradientTakeClosest = 1;
    cout << "gradientTakeClosest = " << gradientTakeClosest << endl;
}
END_WORD
REGISTER_WORD(GradientServoTakeClosest)

WORD(DarkServo)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  darkServoIterations = 0;
  ms->pushWord("darkServoA");
}
END_WORD
REGISTER_WORD(DarkServo)

WORD(DarkServoA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int numPause = 4;
  darkServoIterations++;
  ms->pushWord("darkServoB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 100);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition"); 
}
END_WORD
REGISTER_WORD(DarkServoA)

WORD(DarkServoB)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (darkServoIterations > darkServoTimeout) {
    cout << "darkServo timed out, continuing..." << endl;
    return;
  }

  darkServo(ms);
}
END_WORD
REGISTER_WORD(DarkServoB)




WORD(GoToPrePickPose)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentEEPose = lastPrePickPose;
}
END_WORD
REGISTER_WORD(GoToPrePickPose)


WORD(GoToLastPickPose)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentEEPose = lastPickPose;
}
END_WORD
REGISTER_WORD(GoToLastPickPose)


WORD(ReturnObject)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  cout << "Returning object." << endl;
  ms->pushWord("goToPrePickPose");
  ms->pushWord("waitUntilGripperNotMoving");
  ms->pushWord("openGripper");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("goToLastPickPose");
  ms->pushWord("approachSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("goToPrePickPose");

}
END_WORD
REGISTER_WORD(ReturnObject)

