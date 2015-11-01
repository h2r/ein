
#include "ein_words.h"
#include "ein.h"


namespace ein_words {

CONFIG_SETTER_ENUM(SetGradientServoMode, ms->config.currentGradientServoMode, (gradientServoMode));
CONFIG_GETTER_INT(GradientServoMode, ms->config.currentGradientServoMode);

WORD(FindBestOfFourGraspsUsingMemory)
CODE(1048620)     // numlock + ,
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "Selecting best of 4 grasps...  numlock + , useContinuousGraspTransform = " << ms->config.useContinuousGraspTransform << endl;

  if (ms->config.useContinuousGraspTransform) {
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
  switch (ms->config.currentPickMode) {
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
  cout << "trZ: " << ms->config.trZ << endl;

  ms->config.currentEEPose.px = targetX;
  ms->config.currentEEPose.py = targetY;
      
  cout << "Assuming x,y,gear: " << targetX << " " << targetY << " " << ms->config.maxGG << endl;

  //ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  // ATTN 19
  if (ms->config.useContinuousGraspTransform) {
    cout << "Assuming continuous maxGG: " << ms->config.maxGG << " localMaxGG: " << ms->config.localMaxGG << endl;
    setCCRotation(ms, (ms->config.maxGG+4)%4); 
  } else {
    ms->pushWord(1048631); // assume best gear
  }
}
END_WORD
REGISTER_WORD(AssumeWinningGgAndXyInLocalPose)

WORD(SetSnapToFlushGrasp)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setSnapToFlushGrasp: was " << ms->config.snapToFlushGrasp << " will be " << valToSet << endl;
  ms->config.snapToFlushGrasp = valToSet;
}
END_WORD
REGISTER_WORD(SetSnapToFlushGrasp)

WORD(MoveToTargetZAndGrasp)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  //ms->pushWord("moveToTargetZAndGraspA");

  cout << "moveToTargetZAndGrasp: snapToFlushGrasp is " << ms->config.snapToFlushGrasp << endl;
  if (ms->config.snapToFlushGrasp) {
    if (0) {
      // using position
      ms->pushWord("pressAndGrasp");
    } else if (0) {
      // using effort only 
      ms->pushWord("closeGripper");
      ms->pushWord("pressUntilEffort");

      ms->pushWord("setEffortThresh");
      ms->pushWord("8.0");


      ms->pushWord("setSpeed");
      ms->pushWord("0.03");

      ms->pushWord("pressUntilEffortInit");
      ms->pushWord("comeToStop");
      ms->pushWord("setMovementStateToMoving");
      ms->pushWord("comeToStop");
      ms->pushWord("waitUntilAtCurrentPosition");

      ms->pushWord("setSpeed");
      ms->pushWord("0.05");

      ms->pushWord("replicateWord");
      ms->pushWord("16");
      ms->pushData("localZUp");

      ms->pushWord("setGridSizeCoarse");
    } else {
      // using twist and effort
      ms->pushWord("closeGripper");
      ms->pushWord("pressUntilEffortOrTwist");

      ms->pushWord("setTwistThresh");
      ms->pushWord("0.006");

      ms->pushWord("setEffortThresh");
      ms->pushWord("20.0");

      ms->pushWord("setSpeed");
      ms->pushWord("0.03");

      ms->pushWord("pressUntilEffortOrTwistInit");
      ms->pushWord("comeToStop");
      ms->pushWord("setMovementStateToMoving");
      ms->pushWord("comeToStop");
      ms->pushWord("waitUntilAtCurrentPosition");

      ms->pushWord("setSpeed");
      ms->pushWord("0.05");

      ms->pushWord("replicateWord");
      ms->pushWord("16");
      ms->pushData("localZUp");

      ms->pushWord("setGridSizeCoarse");
    
    }
  } else {
    ms->pushWord("moveToTargetZAndGraspA");
  }
}
END_WORD
REGISTER_WORD(MoveToTargetZAndGrasp)

WORD(MoveToTargetZAndGraspA)
CODE(1048682)     // numlock + j
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->pushWord("closeGripper"); 

  cout << this->name() <<": "  << ms->config.classGraspZsSet.size() << " " << ms->config.classGraspZs.size() << endl;
  double pickZ;

/* Don't use for now.
  if ( (ms->config.classGraspZsSet.size() > ms->config.targetClass) && 
       (ms->config.classGraspZs.size() > ms->config.targetClass) &&
       (ms->config.classGraspZsSet[ms->config.targetClass] == 1)) {
    // -(-trZ - graspDepthOffset)
    // trZ holds the height above the table that we want the end effector to be. 
    pickZ = -ms->config.currentTableZ + -ms->config.classGraspZs[ms->config.targetClass];
    cout << "delivering class " << ms->config.classLabels[ms->config.targetClass] << " with classGraspZ " << ms->config.classGraspZs[ms->config.targetClass] << endl;
  }  
  else 
*/
  {
    double threshedZ = min(ms->config.trZ, 0.0);
    
    // trZ is the height of the object above the table. 
    // pickFlushFactor is distance from the end effector Z to the tip of the gripper. 
    double worldZOfObjectTop = -(threshedZ + ms->config.currentTableZ) ;
    double pickZpre = worldZOfObjectTop + ms->config.pickFlushFactor + ms->config.graspDepthOffset;
    double flushZ = -(ms->config.currentTableZ) + ms->config.pickFlushFactor;
    pickZ = max(flushZ, pickZpre);
    cout << "moveToTargetZAndGrasp trZ pickZ flushZ pickZpre: " << ms->config.trZ << " " << pickZ << " " << flushZ << " " << pickZpre << " " << endl;    
  }

  cout << "pickZ: " << pickZ << endl;
  bool useHybridPick = 1;
  
  double deltaZ = pickZ - ms->config.currentEEPose.pz;
  ms->config.lastPickPose = ms->config.currentEEPose;
  ms->config.lastPickPose.pz = pickZ;

  int tbb = ms->config.targetBlueBox;
  if (tbb < ms->config.blueBoxMemories.size()) {
    ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
  } else {
    assert(0);
  }

  if (useHybridPick) {
    int pickNoops = 20;
    int increments = 0.1/GRID_COARSE;
    ms->config.currentEEPose.pz = pickZ+increments*GRID_COARSE;
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushCopies('s', increments);
    ms->pushWord("setGridSizeCoarse");
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
END_WORD
REGISTER_WORD(MoveToTargetZAndGraspA)

WORD(ShakeItUpAndDown)
CODE(131081)   // capslock + tab
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.eepReg5 = ms->config.currentEEPose;

  ms->config.eepReg6 = ms->config.currentEEPose;
  ms->config.eepReg6.pz += 0.2;

  pushGridSign(ms, GRID_COARSE);    
  if (isGripperGripping(ms)) {
    happy(ms);
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('5');  // assume pose at register 5
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('6'); // assume pose at register 6
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('5'); // assume pose at register 5
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('6');

  }

  pushGridSign(ms, NOW_THATS_COARSE);
  ms->pushWord("fullImpulse");

}
END_WORD
REGISTER_WORD(ShakeItUpAndDown)

WORD(TryToMoveToTheLastPrePickHeight)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose.pz = ms->config.lastPrePickPose.pz;
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  cout << "trying to move to the last pre pick height..." << endl;
}
END_WORD
REGISTER_WORD(TryToMoveToTheLastPrePickHeight)

WORD(TryToMoveToTheLastPickHeight)
CODE( 262241)     // ctrl + a
virtual void execute(std::shared_ptr<MachineState> ms) {
  double deltaZ = (ms->config.lastPickPose.pz) - ms->config.currentEEPose.pz;
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
      int increments = 0.1/GRID_COARSE;
      ms->config.currentEEPose.pz = ms->config.lastPickPose.pz+increments*GRID_COARSE;

      //ms->pushCopies("endStackCollapseNoop", pickNoops);
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushCopies('s', increments);
      ms->pushWord("setGridSizeCoarse");
      ms->pushWord("approachSpeed");
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      //ms->pushWord("quarterImpulse");
      ms->pushWord("approachSpeed");
    } else {
      ms->config.currentEEPose.pz = ms->config.lastPickPose.pz;
      ms->pushWord("waitUntilAtCurrentPosition"); 
    }
  }
  cout << "trying to move to the last pick height..." << endl;
}
END_WORD
REGISTER_WORD(TryToMoveToTheLastPickHeight)


WORD(IfNoGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (isGripperGripping(ms))  {
    ms->popWord();
  }
}
END_WORD
REGISTER_WORD(IfNoGrasp)

WORD(IfGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (!isGripperGripping(ms))  {
    ms->popWord();
  }
}
END_WORD
REGISTER_WORD(IfGrasp)




WORD(ShakeItOff1)
CODE( 131151)     // capslock + o
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  int depthToPlunge = 24;
  int flexThisFar = 80;
  cout << "SHAKING IT OFF!!!" << endl;

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
  pushGridSign(ms, GRID_COARSE);

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
  ms->config.graspAttemptCounter++;      
  cout << "thisGraspPicked: " << operationStatusToString(ms->config.thisGraspPicked) << endl;
  cout << "thisGraspReleased: " << operationStatusToString(ms->config.thisGraspReleased) << endl;
  if (ARE_GENERIC_PICK_LEARNING(ms)) {
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
  if ((ms->config.thisGraspPicked == SUCCESS) && (ms->config.thisGraspReleased == SUCCESS)) {
    ms->config.graspSuccessCounter++;
    happy(ms);
    if (ARE_GENERIC_PICK_LEARNING(ms)) {
      //ms->config.graspMemoryPicks[j+0*ms->config.rmWidth*ms->config.rmWidth]++;
      //ms->config.graspMemoryPicks[j+1*ms->config.rmWidth*ms->config.rmWidth]++;
      //ms->config.graspMemoryPicks[j+2*ms->config.rmWidth*ms->config.rmWidth]++;
      //ms->config.graspMemoryPicks[j+3*ms->config.rmWidth*ms->config.rmWidth]++;
      ms->config.graspMemoryPicks[i]++;
    }
        
    if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
      recordBoundingBoxSuccess(ms);
    }

    double thisPickRate = double(ms->config.graspMemoryPicks[i]) / double(ms->config.graspMemoryTries[i]);
    int thisNumTries = ms->config.graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
    if (ms->config.currentPickMode == LEARNING_SAMPLING) {
      if ( (thisNumTries >= ms->config.thompsonMinTryCutoff) && 
           (thisPickRate >= ms->config.thompsonMinPassRate) ) {
        ms->config.thompsonPickHaltFlag = 1;
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
      double presult = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget);
      // we want probability that mu > d
      double result = 1.0 - presult;

      double presult2a = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget + ms->config.algorithmCEPS);
      double presult2b = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget - ms->config.algorithmCEPS);
      // we want probability that 
      //  ms->config.algorithmCTarget - ms->config.algorithmCEPS < mu < ms->config.algorithmCTarget + ms->config.algorithmCEPS
      double result2 = presult2a - presult2b;

      cout << "prob that mu > d: " << result << " algorithmCAT: " << ms->config.algorithmCAT << endl;
      if (ms->config.currentPickMode == LEARNING_ALGORITHMC) {
        ms->config.thompsonPickHaltFlag = (result > ms->config.algorithmCAT);
        if (result2 > ms->config.algorithmCAT) {
          ms->config.thompsonPickHaltFlag = 1;
        }
      }
    }
  } else {
    double thisPickRate = double(ms->config.graspMemoryPicks[i]) / double(ms->config.graspMemoryTries[i]);
    int thisNumTries = ms->config.graspMemoryTries[i];
    cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
    sad(ms);
    if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
      recordBoundingBoxFailure(ms);
    }
  }
  copyGraspMemoryTriesToClassGraspMemoryTries(ms);
  ms->config.graspSuccessRate = ms->config.graspSuccessCounter / ms->config.graspAttemptCounter;
  ros::Time thisTime = ros::Time::now();
  ros::Duration sinceStartOfTrial = thisTime - ms->config.graspTrialStart;
      
  cout << "<><><><> Grasp attempts rate time gripperPosition currentPickMode: " << ms->config.graspSuccessCounter << "/" << ms->config.graspAttemptCounter << " " << ms->config.graspSuccessRate << " " << sinceStartOfTrial.toSec() << " seconds " << " " << pickModeToString(ms->config.currentPickMode) << endl;
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
  if (ms->config.gripperMoving) {
    ms->pushWord("checkGrasp"); // check grasp
  } else {
    cout << "gripperPosition: " << ms->config.gripperPosition << " gripperThresh: " << ms->config.gripperThresh << endl;
    cout << "gripperGripping: " << ms->config.gripperGripping << endl;
    if (!isGripperGripping(ms)) {
      cout << "Failed to pick." << endl;
      ms->config.thisGraspPicked = FAILURE;
      sad(ms);
      ms->pushCopies("beep", 15); // beep
    } else {
      cout << "Successful pick." << endl;
      ms->config.thisGraspPicked = SUCCESS;
      happy(ms);
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
  if (ms->config.gripperMoving) {
    //ms->pushWord(196713); // check and count grasp
	cout << "checkAndCountGrasp: repushing because gripper is moving." << endl;
    ms->pushWord("checkAndCountGrasp"); 
	ms->pushWord("waitUntilGripperNotMoving");
  } else {
    ms->config.graspAttemptCounter++;
    switch (ms->config.currentPickMode) {
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
    cout << "gripperPosition: " << ms->config.gripperPosition << " gripperThresh: " << ms->config.gripperThresh << endl;
    if (!isGripperGripping(ms)) {
      if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
        recordBoundingBoxFailure(ms);
      }
      cout << "Failed grasp." << endl;
      //ms->pushWord("pauseStackExecution"); // pause stack execution
      ms->pushCopies("beep", 15); // beep
    } else {
      if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
        recordBoundingBoxSuccess(ms);
      }
      ms->config.graspSuccessCounter++;
      cout << "Successful grasp." << endl;
      //ms->config.graspMemoryPicks[i]++;
      switch (ms->config.currentPickMode) {
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

    if (ms->config.currentPickMode == LEARNING_SAMPLING) {
      if ( (thisNumTries >= ms->config.thompsonMinTryCutoff) && 
           (thisPickRate >= ms->config.thompsonMinPassRate) ) {
        ms->config.thompsonPickHaltFlag = 1;
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
      double presult = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget);
      // we want probability that mu > d
      double result = 1.0 - presult;

      double presult2a = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget + ms->config.algorithmCEPS);
      double presult2b = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget - ms->config.algorithmCEPS);
      // we want probability that 
      //  ms->config.algorithmCTarget - ms->config.algorithmCEPS < mu < ms->config.algorithmCTarget + ms->config.algorithmCEPS
      double result2 = presult2a - presult2b;

      cout << "prob that mu > d: " << result << " algorithmCAT: " << ms->config.algorithmCAT << endl;
      if (ms->config.currentPickMode == LEARNING_ALGORITHMC) {
        ms->config.thompsonPickHaltFlag = (result > ms->config.algorithmCAT);
        if (result2 > ms->config.algorithmCAT) {
          ms->config.thompsonPickHaltFlag = 1;
        }
      }
    }

    copyGraspMemoryTriesToClassGraspMemoryTries(ms);
    ms->config.graspSuccessRate = ms->config.graspSuccessCounter / ms->config.graspAttemptCounter;
    ros::Time thisTime = ros::Time::now();
    ros::Duration sinceStartOfTrial = thisTime - ms->config.graspTrialStart;
    cout << "<><><><> Grasp attempts rate time gripperPosition currentPickMode: " << ms->config.graspSuccessCounter << "/" << ms->config.graspAttemptCounter << " " << ms->config.graspSuccessRate << " " << sinceStartOfTrial.toSec() << " seconds " << ms->config.gripperPosition << " " << pickModeToString(ms->config.currentPickMode) << endl;
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

  if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
    ms->pushWord("setRandomPositionAndOrientationForHeightLearning"); // set random position for bblearn
  } else {
    ms->pushWord("perturbPosition"); // numlock + /
  }

  ms->pushCopies("zDown", 3);
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position

  if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
    ms->pushWord("moveToRegister4"); // assume pose at register 4
  } else {
    ms->pushWord("moveToRegister2"); // assume pose at register 2
  }

  ms->pushWord("fullImpulse"); 
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

WORD(SetGraspModeToCrane)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Setting grasp mode to GRASP_CRANE." << endl;
  ms->config.currentGraspMode = GRASP_CRANE;
}
END_WORD
REGISTER_WORD(SetGraspModeToCrane)

WORD(SetGraspModeTo3D)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Setting grasp mode to GRASP_3D." << endl;
  ms->config.currentGraspMode = GRASP_3D;
}
END_WORD
REGISTER_WORD(SetGraspModeTo3D)

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

  if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
    ms->pushWord("moveToRegister4"); // assume pose at register 4
  }

  //ms->pushWord("quarterImpulse"); 
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("comeToHover"); 
  ms->pushWord("waitUntilGripperNotMoving");

  if (ms->config.currentGraspMode == GRASP_CRANE || ms->config.class3dGrasps[ms->config.targetClass].size() == 0) {
    ms->pushWord("moveToTargetZAndGrasp"); 
  } else if (ms->config.currentGraspMode == GRASP_3D) {
    ms->pushWord("closeGripper"); 
    //ms->pushWord("assumeCurrent3dGrasp"); 
    ms->pushWord("assumeAny3dGrasp"); 
  } else {
    assert(0);
  }

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

WORD(DecrementGraspGear)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "decrement ms->config.currentGraspGear was is: " << ms->config.currentGraspGear << " ";
  int thisGraspGear = (ms->config.currentGraspGear -1 + ms->config.totalGraspGears) % ms->config.totalGraspGears;
  
  //   set drX
  ms->config.drX = ms->config.ggX[thisGraspGear];
  ms->config.drY = ms->config.ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(ms, thisGraspGear);
  ms->config.currentGraspGear = thisGraspGear;
  
  cout << ms->config.currentGraspGear << endl;
}
END_WORD
REGISTER_WORD(DecrementGraspGear)




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

WORD(ResetAerialGradientTemporalFrameAverage)
CODE(262237)      // ctrl + ]
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "resetting aerialGradientTemporalFrameAverage." << endl;
  ms->config.aerialGradientTemporalFrameAverage *= 0.0;
}
END_WORD
REGISTER_WORD(ResetAerialGradientTemporalFrameAverage)


WORD(SynchronicServoDoNotTakeClosest)
CODE(131139)  // capslock + c
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.synchronicTakeClosest = 0;
  cout << "synchronicTakeClosest = 0" << endl;
  ms->config.synServoLockFrames = 0;
}
END_WORD
REGISTER_WORD(SynchronicServoDoNotTakeClosest)

WORD(SynchronicServoTakeClosest)
CODE(196707)     // capslock + C
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX deprecate
  ms->config.synchronicTakeClosest = 1;
  cout << "synchronicTakeClosest = 1" << endl;
  ms->config.synServoLockFrames = 0;
}
END_WORD
REGISTER_WORD(SynchronicServoTakeClosest)


WORD(TwoDPatrolStart)
CODE(131159)     // capslock + w
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.bailAfterSynchronic = 0;
  ms->config.graspAttemptCounter = 0;
  ms->config.graspSuccessCounter = 0;
  ms->config.graspTrialStart = ros::Time::now();
  ms->config.thompsonPickHaltFlag = 0;
  ms->config.thompsonHeightHaltFlag = 0;
  ms->config.pilotTarget.px = -1;
  ms->config.pilotTarget.py = -1;
  ms->config.pilotClosestTarget.px = -1;
  ms->config.pilotClosestTarget.py = -1;
  ms->config.oscilStart = ros::Time::now();
  ms->config.accumulatedTime = ms->config.oscilStart - ms->config.oscilStart;
  ms->config.oscCenX = ms->config.currentEEPose.px;
  ms->config.oscCenY = ms->config.currentEEPose.py;
  ms->config.oscCenZ = ms->config.currentEEPose.pz+0.1;
  ms->pushWord("twoDPatrolContinue"); // 2D patrol continue
  ms->pushWord("visionCycle");
  // we want to move to a higher holding position for visual patrol
  // so we assume that we are at 20 cm = IR scan height and move to 30 cm
  pushGridSign(ms, GRID_COARSE);
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
  ms->config.thisGraspPicked = UNKNOWN;
  ms->config.thisGraspReleased = UNKNOWN;
  neutral(ms);
  
  if (ARE_GENERIC_PICK_LEARNING(ms)) {
    if (ms->config.thompsonHardCutoff) {
      if (ms->config.graspAttemptCounter >= ms->config.thompsonTries) {
        cout << "Clearing call stack because we did " << ms->config.graspAttemptCounter << " tries." << endl;
        ms->clearStack();
        ms->pushCopies("beep", 15); // beep
        return;
      }
    }
    
    if (ms->config.thompsonAdaptiveCutoff) {
      if ( (ms->config.thompsonPickHaltFlag) ||
           (ms->config.graspAttemptCounter >= ms->config.thompsonTries) ) {
        cout << "Clearing call stack. thompsonPickHaltFlag = " << ms->config.thompsonPickHaltFlag << 
          " and we did " << ms->config.graspAttemptCounter << " tries." << endl;
        ms->clearStack();
        ms->pushCopies("beep", 15); // beep
        return;
      }
    }
  }
  
  ms->config.synServoLockFrames = 0;
  ms->config.currentGradientServoIterations = 0;
  
  ros::Duration delta = (ros::Time::now() - ms->config.oscilStart) + ms->config.accumulatedTime;
  
  ms->config.currentEEPose.px = ms->config.oscCenX + ms->config.oscAmpX*sin(2.0*3.1415926*ms->config.oscFreqX*delta.toSec());
  ms->config.currentEEPose.py = ms->config.oscCenY + ms->config.oscAmpY*sin(2.0*3.1415926*ms->config.oscFreqY*delta.toSec());
  ms->pushWord("twoDPatrolContinue"); 
  
  // check to see if the target class is around, or take closest
  if ( ((ms->config.pilotTarget.px != -1) && (ms->config.pilotTarget.py != -1)) ||
       (ms->config.synchronicTakeClosest && ((ms->config.pilotClosestTarget.px != -1) && (ms->config.pilotClosestTarget.py != -1))) )
    {
      // if so, push servoing command and set lock frames to 0
      ms->pushWord("synchronicServo"); // synchronic servo
      
      if (ms->config.targetClass != -1)
        cout << "Found the target " << ms->config.classLabels[ms->config.targetClass] << ". " << endl;
      // grab the last bit of accumulated time
      ms->config.accumulatedTime = ms->config.accumulatedTime + (ros::Time::now() - ms->config.oscilStart);
    } else {
    // if not, potentially do vision and continue the 2D patrol
    
    // check and push vision cycle 
    ros::Duration timeSinceLast = ros::Time::now() - ms->config.lastVisionCycle;
    if (timeSinceLast.toSec() > ms->config.visionCycleInterval) {
      ms->pushWord("visionCycle");
      // grab the last bit of accumulated time
      ms->config.accumulatedTime = ms->config.accumulatedTime + (ros::Time::now() - ms->config.oscilStart);
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
  ms->pushWord("synchronicServoRepeat");
  ms->pushWord("synchronicServoTakeClosest");
  // XXX TODO take closest needs to be set to the default and a better way of selecting the target factored.
}
END_WORD
REGISTER_WORD(SynchronicServo)

WORD(SynchronicServoRepeat)
virtual void execute(std::shared_ptr<MachineState> ms) { 
  ms->pushWord("synchronicServoA");
  if (ms->config.currentBoundingBoxMode == MAPPING) {
    ms->pushWord("visionCycleNoClassify");
  } else {
    ms->pushWord("visionCycle"); // vision cycle
  }
  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  //ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition"); 
}
END_WORD
REGISTER_WORD(SynchronicServoRepeat)

WORD(SynchronicServoA)
virtual void execute(std::shared_ptr<MachineState> ms) { 
  synchronicServo(ms);
}
END_WORD
REGISTER_WORD(SynchronicServoA)

WORD(GradientServo)
CODE(196728)   // capslock + X
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentGradientServoIterations = 0;
  ms->config.gshHistogram = eePose::zero();
  ms->config.gshCounts = 0.0;

  ms->pushWord("gradientServoA");
}
END_WORD
REGISTER_WORD(GradientServo)

WORD(ContinuousServoL)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("continuousServoL");
  ms->pushWord("continuousServo");
  //ms->pushWord("comeToStop");
}
END_WORD
REGISTER_WORD(ContinuousServoL)

WORD(ContinuousServo)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX there is some issue when the orientation is changing, 
  //  convergence isn't instantaneous unless waitUntilAtCurrentPosition is called
  ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("waitForSeconds");
  //ms->pushWord("\"0.2\"");
  //ms->pushWord("endStackCollapseNoop");
  ms->pushWord("waitUntilEndpointCallbackReceived");
  ms->pushWord("continuousServoA");
  ms->pushWord("continuousServoPrep");
}
END_WORD
REGISTER_WORD(ContinuousServo)

WORD(ContinuousServoPrep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("densityA");
  ms->pushWord("waitUntilEndpointCallbackReceived");
  ms->pushWord("waitUntilImageCallbackReceived");
}
END_WORD
REGISTER_WORD(ContinuousServoPrep)

WORD(ContinuousServoA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  continuousServo(ms);
}
END_WORD
REGISTER_WORD(ContinuousServoA)

WORD(GradientServoPrep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // ATTN 8
  if (0) {
    ms->pushCopies("density", ms->config.densityIterationsForGradientServo); 
    ms->pushWord("resetAerialGradientTemporalFrameAverage"); 
    ms->pushCopies("density", 1); 
    ms->pushWord("hover");
  }

  // ATTN 23
  {
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("accumulatedDensity");
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

WORD(GradientServoB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.currentGradientServoMode == FOCUSED_CLASS) {
    gradientServo(ms);
  } else if (ms->config.currentGradientServoMode == LATENT_CLASS) {
    gradientServoLatentClass(ms);
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(GradientServoB)

WORD(GradientServoA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("gradientServoB");
  ms->pushWord("gradientServoPrep");
}
END_WORD
REGISTER_WORD(GradientServoA)


WORD(GradientServoIfBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (ms->config.bLabels.size() > 0) && (ms->config.pilotClosestBlueBoxNumber != -1) ) {
    // XXX changeTargetClass should come after?
    ms->pushWord("gradientServo");
    changeTargetClass(ms, ms->config.bLabels[ms->config.pilotClosestBlueBoxNumber]);
  }
}
END_WORD
REGISTER_WORD(GradientServoIfBlueBoxes)

WORD(LockTargetIfBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (ms->config.bLabels.size() > 0) && (ms->config.pilotClosestBlueBoxNumber != -1) ) {
    ms->pushWord("recordPostTargetLock");
    ms->pushWord("prepareForGraspFromMemory");
    ms->pushWord("recordPreTargetLock");
  }
}
END_WORD
REGISTER_WORD(LockTargetIfBlueBoxes)

WORD(RecordPostTargetLock)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.blueBoxMemories.size() > 0) {
    BoxMemory *lastAdded = &(ms->config.blueBoxMemories[ms->config.blueBoxMemories.size()-1]);
    lastAdded->cameraTime = ros::Time::now();
    lastAdded->aimedPose = ms->config.currentEEPose;
    lastAdded->trZ  = ms->config.trZ;
    cout << "recordPostTargetLock saving pickedPose..." << endl;
    cout << "trZ = " << ms->config.trZ << endl;
    cout << "Current EE Position (x,y,z): " << ms->config.currentEEPose.px << " " << ms->config.currentEEPose.py << " " << ms->config.currentEEPose.pz << endl;
    cout << "Current EE Orientation (x,y,z,w): " << ms->config.currentEEPose.qx << " " << ms->config.currentEEPose.qy << " " << ms->config.currentEEPose.qz << " " << ms->config.currentEEPose.qw << endl;
    lastAdded->lockStatus = POSE_LOCK;
  }
}
END_WORD
REGISTER_WORD(RecordPostTargetLock)

WORD(RecordPreTargetLock)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.blueBoxMemories.size() > 0) {
    BoxMemory *lastAdded = &(ms->config.blueBoxMemories[ms->config.blueBoxMemories.size()-1]);
    lastAdded->lockedPose = ms->config.currentEEPose;
    cout << "recordPreTargetLock saving lockedPose..." << endl;

    int thisTargetClassIdx = ms->config.targetClass;

    // calculate the affordance poses post-lock
cout << "about to calc 3d poses" << endl;
    {
      int tnc = ms->config.class3dGrasps.size();
      if ( (thisTargetClassIdx > -1) && (thisTargetClassIdx < tnc) ) {
	int tnp = ms->config.class3dGrasps[thisTargetClassIdx].size();
cout << "tnc, tnp: " << tnc << " " << tnp << endl;
	lastAdded->aff3dGraspPoses.resize(tnp);
	for (int i = 0; i < tnp; i++) {
	  eePose toApply = ms->config.class3dGrasps[thisTargetClassIdx][i];  
	  eePose toWhichWasApplied = ms->config.currentEEPose;
	  toWhichWasApplied.pz = -ms->config.currentTableZ;
	  // this order is important because quaternion multiplication is not commutative
	  //toWhichWasApplied = toWhichWasApplied.plusP(toWhichWasApplied.applyQTo(toApply));
	  //toWhichWasApplied = toWhichWasApplied.multQ(toApply);
	  toWhichWasApplied = toApply.applyAsRelativePoseTo(toWhichWasApplied);
	  lastAdded->aff3dGraspPoses[i] = toWhichWasApplied;
	}
      }
    }
cout << "about to calc pup poses" << endl;
    {
      int tnc = ms->config.classPlaceUnderPoints.size();
      if ( (thisTargetClassIdx > -1) && (thisTargetClassIdx < tnc) ) {
	int tnp = ms->config.classPlaceUnderPoints[thisTargetClassIdx].size();
cout << "tnc, tnp: " << tnc << " " << tnp << endl;
	lastAdded->affPlaceUnderPoses.resize(tnp);
	for (int i = 0; i < tnp; i++) {
	  eePose toApply = ms->config.classPlaceUnderPoints[thisTargetClassIdx][i];  
	  eePose toWhichWasApplied = ms->config.currentEEPose;
	  toWhichWasApplied.pz = -ms->config.currentTableZ;
	  // this order is important because quaternion multiplication is not commutative
	  //toWhichWasApplied = toWhichWasApplied.plusP(toWhichWasApplied.applyQTo(toApply));
	  //toWhichWasApplied = toWhichWasApplied.multQ(toApply);
	  toWhichWasApplied = toApply.applyAsRelativePoseTo(toWhichWasApplied);
	  lastAdded->affPlaceUnderPoses[i] = toWhichWasApplied;
cout << "added " << lastAdded->affPlaceUnderPoses[i] << endl << " and current is " << endl << ms->config.currentEEPose << endl;
	}
      }
    }
cout << "about to calc pop poses" << endl;
    {
      int tnc = ms->config.classPlaceOverPoints.size();
      if ( (thisTargetClassIdx > -1) && (thisTargetClassIdx < tnc) ) {
	int tnp = ms->config.classPlaceOverPoints[thisTargetClassIdx].size();
cout << "tnc, tnp: " << tnc << " " << tnp << endl;
	lastAdded->affPlaceOverPoses.resize(tnp);
	for (int i = 0; i < tnp; i++) {
	  eePose toApply = ms->config.classPlaceOverPoints[thisTargetClassIdx][i];  
	  eePose toWhichWasApplied = ms->config.currentEEPose;
	  toWhichWasApplied.pz = -ms->config.currentTableZ;
	  // this order is important because quaternion multiplication is not commutative
	  //toWhichWasApplied = toWhichWasApplied.plusP(toWhichWasApplied.applyQTo(toApply));
	  //toWhichWasApplied = toWhichWasApplied.multQ(toApply);
	  toWhichWasApplied = toApply.applyAsRelativePoseTo(toWhichWasApplied);
	  lastAdded->affPlaceOverPoses[i] = toWhichWasApplied;
cout << "added " << lastAdded->affPlaceUnderPoses[i] << endl << " and current is " << endl << ms->config.currentEEPose << endl;
	}
      }
    }
  }
}
END_WORD
REGISTER_WORD(RecordPreTargetLock)



WORD(GradientServoTakeClosest)
// capslock + numlock + h
CODE(1179720)
virtual void execute(std::shared_ptr<MachineState> ms)
{
    // XXX deprecate
    ms->config.gradientTakeClosest = 1;
    cout << "gradientTakeClosest = " << ms->config.gradientTakeClosest << endl;
}
END_WORD
REGISTER_WORD(GradientServoTakeClosest)

WORD(DarkServo)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.darkServoIterations = 0;
  ms->pushWord("darkServoA");
}
END_WORD
REGISTER_WORD(DarkServo)

WORD(DarkServoA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int numPause = 4;
  ms->config.darkServoIterations++;
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
  if (ms->config.darkServoIterations > ms->config.darkServoTimeout) {
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
  ms->config.currentEEPose = ms->config.lastPrePickPose;
}
END_WORD
REGISTER_WORD(GoToPrePickPose)

WORD(GoToLastPickPose)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentEEPose = ms->config.lastPickPose;
}
END_WORD
REGISTER_WORD(GoToLastPickPose)

WORD(AssumeLastPickOrientation)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentEEPose.copyQ(ms->config.lastPickPose);
}
END_WORD
REGISTER_WORD(AssumeLastPickOrientation)

WORD(SetAerialGradientsToBarsLengthWidthGap)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int barHalfLength = 0;
  int barWidth = 0;
  int barHalfGap = 0;

  GET_NUMERIC_ARG(ms, barHalfGap);
  GET_NUMERIC_ARG(ms, barWidth);
  GET_NUMERIC_ARG(ms, barHalfLength);

  int f_width = 100;
  int f_half_width = f_width/2;
  Mat t_gradient(f_width, f_width, CV_64F);
  t_gradient = 0.0;

  // evidently this is the fast order
  for (int y = (f_half_width - barHalfGap - barWidth); y < (f_half_width - barHalfGap) ; y++) {
    for (int x = (f_half_width - barHalfLength); x < (f_half_width + barHalfLength); x++) {
      t_gradient.at<double>(y,x) = 1.0;
    }
  }

  for (int y = (f_half_width + barHalfGap); y < (f_half_width + barHalfGap + barWidth) ; y++) {
    for (int x = (f_half_width - barHalfLength); x < (f_half_width + barHalfLength); x++) {
      t_gradient.at<double>(y,x) = 1.0;
    }
  }

  
  ms->config.classAerialGradients[ms->config.targetClass] = t_gradient; 
  ms->config.classHeight0AerialGradients[ms->config.targetClass] = t_gradient;
  ms->config.classHeight1AerialGradients[ms->config.targetClass] = t_gradient;
  ms->config.classHeight2AerialGradients[ms->config.targetClass] = t_gradient;
  ms->config.classHeight3AerialGradients[ms->config.targetClass] = t_gradient;
}
END_WORD
REGISTER_WORD(SetAerialGradientsToBarsLengthWidthGap)

}
