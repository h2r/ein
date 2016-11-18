#include "ein_words.h"
#include "ein.h"

void targetBoxMemory(MachineState * ms, int memoryIdx) {
  BoxMemory memory = ms->config.blueBoxMemories[memoryIdx];
  ms->config.targetBlueBox = memoryIdx;

  cout << "Aimed pose: " << memory.aimedPose << endl;
  ms->config.currentEEPose = memory.aimedPose;
  ms->config.lastPrePickPose = memory.aimedPose;
  ms->config.lastLockedPose = memory.lockedPose;
  ms->config.trZ = memory.trZ;
}



namespace ein_words {

WORD(TwoPartPlaceObjectOnObject)
virtual void execute(MachineState * ms) {
  
  string firstObjectLabel;
  GET_ARG(ms, StringWord, firstObjectLabel);
  string secondObjectLabel;
  GET_ARG(ms, StringWord, secondObjectLabel);
  // this is specified in mm
  int amountMms;
  GET_ARG(ms, IntegerWord, amountMms);

  ms->pushWord("placeHeldObjectOnObject");
  ms->pushWord(std::make_shared<StringWord>(secondObjectLabel));
  ms->pushWord(std::make_shared<IntegerWord>(amountMms));

  ms->pushWord("setPlaceModeToRegister");


  ms->pushWord("deliverObject");
  ms->pushWord(std::make_shared<StringWord>(firstObjectLabel));

  ms->pushWord("setPlaceModeToHold");

  cout << "twoPartPlaceObjectOnObject: placing " << secondObjectLabel << " on " << firstObjectLabel << " by " << amountMms << endl;
}
END_WORD
REGISTER_WORD(TwoPartPlaceObjectOnObject)

WORD(PlaceHeldObjectOnObject)
virtual void execute(MachineState * ms) {
  string secondObjectLabel;
  GET_ARG(ms, StringWord, secondObjectLabel);
  // this is specified in mm
  int amountMms;
  GET_ARG(ms, IntegerWord, amountMms);
  double cTableHeight = 0.001 * amountMms;


  {
    eePose placePose;
    int success = placementPoseHeldAboveLabel2By(ms, secondObjectLabel, cTableHeight, &placePose);
    if (success) {
      cout << "placeHeldObjectOnObject: placing held object onto " << secondObjectLabel << " by " << amountMms << endl;
      ms->config.placeTarget = placePose;
      ms->pushWord("placeObjectInDeliveryZone");
    } else {
      cout << "placeHeldObjectOnObject: failed" << endl; 
    }
  }
}
END_WORD
REGISTER_WORD(PlaceHeldObjectOnObject)

WORD(DucksInARow)
virtual void execute(MachineState * ms) {
  eePose destPose;
  CONSUME_EEPOSE(destPose,ms);

  ms->pushWord("ducksInARow");

  ms->pushWord("moveObjectToPose");
  ms->pushWord(std::make_shared<EePoseWord>(destPose));
  ms->pushWord("duck");

  ms->pushWord("setIdleModeToEmpty");
}
END_WORD
REGISTER_WORD(DucksInARow)

WORD(CornellMugsOnTables)
virtual void execute(MachineState * ms) {
  
  // cTableHeight moves with the flush factor
  double cTableHeight = -0.017;//-0.045;//0.0;//0.025;

  ms->pushWord("moveObjectToObjectByAmount");
  ms->pushWord("brownMug");
  ms->pushWord("table3");
  ms->pushWord(std::make_shared<IntegerWord>(cTableHeight));


  ms->pushWord("moveObjectToObjectByAmount");
  ms->pushWord("metalMug");
  ms->pushWord("table2");
  ms->pushWord(std::make_shared<IntegerWord>(cTableHeight));


  ms->pushWord("moveObjectToObjectByAmount");
  ms->pushWord("redMug");
  ms->pushWord("table1");
  ms->pushWord(std::make_shared<IntegerWord>(cTableHeight));


}
END_WORD
REGISTER_WORD(CornellMugsOnTables)


WORD(SqueezeDuck)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("returnObject .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt .20 waitForSeconds 0 openGripperInt .20 waitForSeconds 40 openGripperInt pickFocusedClass setPlaceModeToHold 1 changeToHeight assumeBeeHome endArgs \"duck\" setClassLabels ;");
}
END_WORD
REGISTER_WORD(SqueezeDuck)

WORD(QuiveringPalm)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("setGridSizeCoarse \"0.03\" waitForSeconds oZDown 6 replicateWord \"0.03\" waitForSeconds oZUp 6 replicateWord \"0.03\" waitForSeconds oZDown 6 replicateWord \"0.03\" waitForSeconds oZUp 6 replicateWord \"0.03\" waitForSeconds oZDown 6 replicateWord \"0.03\" waitForSeconds oZUp 6 replicateWord \"0.03\" waitForSeconds oZDown 6 replicateWord \"0.03\" waitForSeconds oZUp 6 replicateWord \"0.03\" waitForSeconds oZDown 6 replicateWord \"0.03\" waitForSeconds oZUp 6 replicateWord setGridSizeCoarse \"0.03\" waitForSeconds localXUp 4 replicateWord \"0.03\" waitForSeconds localXDown 8 replicateWord \"0.03\" waitForSeconds localXUp 4 replicateWord \"0.03\" waitForSeconds localXUp 4 replicateWord \"0.03\" waitForSeconds localXDown 8 replicateWord \"0.03\" waitForSeconds localXUp 4 replicateWord \"0.03\" waitForSeconds localYUp 4 replicateWord \"0.03\" waitForSeconds localYDown 8 replicateWord \"0.03\" waitForSeconds localYUp 4 replicateWord \"0.03\" waitForSeconds localYUp 4 replicateWord \"0.03\" waitForSeconds localYDown 8 replicateWord \"0.03\" waitForSeconds localYUp 4 replicateWord zDown setMovementSpeedMoveSlow");
}
END_WORD
REGISTER_WORD(QuiveringPalm)

WORD(MoveObjectToObjectByAmount)
virtual void execute(MachineState * ms) {
  string firstObjectLabel;
  string secondObjectLabel;
  double cTableHeight = 0;
  // cTableHeight is specified in meters 
  GET_ARG(ms, StringWord, firstObjectLabel);
  GET_ARG(ms, StringWord, secondObjectLabel);
  GET_NUMERIC_ARG(ms, cTableHeight);
  {
    eePose placePose;
    int success = placementPoseLabel1AboveLabel2By(ms, firstObjectLabel, secondObjectLabel, cTableHeight, &placePose);
    if (success) {
      ms->pushWord("moveObjectToPose");
      ms->pushWord(std::make_shared<EePoseWord>(placePose));
      ms->pushWord(firstObjectLabel);
    }
  }
}
END_WORD
REGISTER_WORD(MoveObjectToObjectByAmount)

WORD(MoveObjectBetweenObjectAndObject)
virtual void execute(MachineState * ms) {
  string firstObjectLabel;
  string secondObjectLabel;
  string thirdObjectLabel;
  GET_ARG(ms, StringWord, firstObjectLabel);
  GET_ARG(ms, StringWord, secondObjectLabel);
  GET_ARG(ms, StringWord, thirdObjectLabel);
  {
    eePose placePose;
    int success = placementPoseLabel1BetweenLabel2AndLabel3(ms, firstObjectLabel, secondObjectLabel, thirdObjectLabel, &placePose);
    if (success) {
      ms->pushWord("moveObjectToPose");
      ms->pushWord(std::make_shared<EePoseWord>(placePose));
      ms->pushWord(firstObjectLabel);

    }
  }
}
END_WORD
REGISTER_WORD(MoveObjectBetweenObjectAndObject)

WORD(SetTheYcbTable)
virtual void execute(MachineState * ms) {

// XXX broken unless you do something with AssumeDeliveryPose
// XXX ms->config.currentEEPose.pz = oldz;
  ms->pushWord("assumeCrane1"); 

  //eePose platePose = {.px = 0.602935, .py = 0.599482, .pz = -0.0395161,
                      //.qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  //ms->pushWord("moveObjectToPose");
  //ms->pushWord(std::make_shared<EePoseWord>(platePose));
  //ms->pushWord("redPlate");


  eePose mugPose = eePose(0.428236, 0.688348, -0.026571,
                          0.0, 1.0, 0.0, 0.0); 
  ms->pushWord("moveObjectToPose");
  ms->pushWord(std::make_shared<EePoseWord>(mugPose));
  ms->pushWord("redMugLow");


  eePose bowlPose = eePose(0.429551, 0.355954, -0.02713,
                           0.0, 1.0, 0.0, 0.0); 
  ms->pushWord("moveObjectToPose");
  ms->pushWord(std::make_shared<EePoseWord>(bowlPose));
  ms->pushWord("redBowlLow");


  eePose knifePose = eePose(0.645808, 0.75673, -0.0605177,
                            0.0, 1.0, 0.0, 0.0); 
  ms->pushWord("moveObjectToPose");
  ms->pushWord(std::make_shared<EePoseWord>(knifePose));
  ms->pushWord("redKnifeLow");



  eePose forkPose =  eePose(0.632388, 0.417448, -0.0487945,
                            0.0, 1.0,  0.0, 0.0); 
  ms->pushWord("moveObjectToPose");
  ms->pushWord(std::make_shared<EePoseWord>(forkPose));
  ms->pushWord("redForkLow");



  eePose spoonPose = eePose(0.624147, 0.811554, -0.0696885,
                            0.0, 1.0, 0.0, 0.0); 
  ms->pushWord("moveObjectToPose");
  ms->pushWord(std::make_shared<EePoseWord>(spoonPose));
  ms->pushWord("redSpoonLow");

}
END_WORD
REGISTER_WORD(SetTheYcbTable)

WORD(MoveObjectToPose)
virtual void execute(MachineState * ms) {
  string className;
  GET_ARG(ms, StringWord, className);
  {
    int class_idx = classIdxForName(ms, className);
    if (class_idx != -1) {
      ms->pushWord("moveTargetObjectToPose");
      changeTargetClass(ms, class_idx);
    } else {
      cout << "No class for " << className << " for " << this->name() << endl;
    }
  }
}
END_WORD
REGISTER_WORD(MoveObjectToPose)

WORD(MoveTargetObjectToPose)
virtual void execute(MachineState * ms) {
  eePose destPose;
  GET_ARG(ms, EePoseWord, destPose);
  {
    ms->config.placeTarget = destPose;
  
    ms->pushWord("deliverTargetObject");
    ms->pushWord("setPlaceModeToRegister");
  }
}
END_WORD
REGISTER_WORD(MoveTargetObjectToPose)

WORD(DeliverObject)
virtual void execute(MachineState * ms) {
  string className;
  GET_ARG(ms, StringWord, className);
  {
    int class_idx = classIdxForName(ms, className);
    if (class_idx != -1) {
      ms->pushWord("deliverTargetObject");
      changeTargetClass(ms, class_idx);
    } else {
      cout << "No class for " << className << " for " << this->name() << endl;
    }
  }
}
END_WORD
REGISTER_WORD(DeliverObject)

WORD(UnmapTargetBlueBox)
virtual void execute(MachineState * ms) {
  int idxToRemove = ms->config.targetBlueBox;
  if ((idxToRemove >= 0) && (idxToRemove < ms->config.blueBoxMemories.size())) {
  } else {
    cout << "unmapTargetBlueBox: invalid blue box, returning." << endl;
    return;
  }

  BoxMemory memory = ms->config.blueBoxMemories[idxToRemove];

  { // set the old box's lastMappedTime to moments after the start of time
    int iStart=-1, iEnd=-1, jStart=-1, jEnd=-1;
    int iTop=-1, iBot=-1, jTop=-1, jBot=-1;
    double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;
    {
      double x, y;
      int i, j;
      pixelToGlobal(ms, memory.top.px-ms->config.mapBlueBoxPixelSkirt, memory.top.py-ms->config.mapBlueBoxPixelSkirt, z, &x, &y);
      mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep,x, y, &i, &j);
      iTop=i;
      jTop=j;
    }
    {
      double x, y;
      int i, j;
      pixelToGlobal(ms, memory.bot.px+ms->config.mapBlueBoxPixelSkirt, memory.bot.py+ms->config.mapBlueBoxPixelSkirt, z, &x, &y);
      mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep,x, y, &i, &j);
      iBot=i;
      jBot=j;
    }
    iStart = min(iBot, iTop);
    iEnd = max(iBot, iTop);
    jStart = min(jBot, jTop);
    jEnd = max(jBot, jTop);
    cout << "DeliverObject erasing iStart iEnd jStart jEnd: " << iStart << " " << iEnd << " " << jStart << " " << jEnd << endl;
    for (int i = iStart; i <= iEnd; i++) {
      for (int j = jStart; j <= jEnd; j++) {
	if (i >= 0 && i < ms->config.mapWidth && j >= 0 && j < ms->config.mapHeight) {
	  ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time(0.001);
	  randomizeNanos(ms, &ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime);
	}
      }
    }
  }
  
  { // remove this blue box; above stamping would cause it to be naturally eliminated IF it was 
    // observed in a searched region...
    vector<BoxMemory> newMemories;
    for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
      if (i != idxToRemove) {
	cout << "Retaining blue box " << i << " while booting " << idxToRemove << endl; 
	newMemories.push_back(ms->config.blueBoxMemories[i]);
      }
    }
    ms->config.blueBoxMemories = newMemories;
  }
}
END_WORD
REGISTER_WORD(UnmapTargetBlueBox)

WORD(DeliverTargetObject)
virtual void execute(MachineState * ms) {
  ms->config.bailAfterGradient = 1;

  ms->config.pilotTarget.px = -1;
  ms->config.pilotTarget.py = -1;
  ms->config.pilotClosestTarget.px = -1;
  ms->config.pilotClosestTarget.py = -1;
  
  int idxOfFirst = -1;
  // we need to remove this from the blue box memories later
  vector<BoxMemory> focusedClassMemories = memoriesForClass(ms, ms->config.focusedClass, &idxOfFirst);
  if (focusedClassMemories.size() == 0) {
    cout << "No memories of the focused class. " << endl;

    if (ms->config.currentPlaceMode == PLACE_REGISTER) {
      ms->pushWord("idler"); 
    } else if (ms->config.currentPlaceMode == HAND) {
      ms->pushWord("idler"); 
    } else if (ms->config.currentPlaceMode == HOLD) {
    } else if (ms->config.currentPlaceMode == SHAKE) {
      ms->pushWord("idler"); 
    } else {
      assert(0);
    }

    return;
  }

  if (focusedClassMemories.size() > 1) {
    cout << "More than one bounding box for class.  Looking for first POSE_REPORTED." << focusedClassMemories.size() << endl;
  } else {
  } // do nothing

  if (idxOfFirst == -1) {
    cout << "No POSE_REPORTED objects of the focused class." << endl;

    if (ms->config.currentPlaceMode == PLACE_REGISTER) {
      ms->pushWord("idler"); 
    } else if (ms->config.currentPlaceMode == HAND) {
      ms->pushWord("idler"); 
    } else if (ms->config.currentPlaceMode == HOLD) {
    } else if (ms->config.currentPlaceMode == SHAKE) {
      ms->pushWord("idler"); 
    } else {
      assert(0);
    }

    return;
  } else {
  } // do nothing

  targetBoxMemory(ms, idxOfFirst);
  ms->pushWord("deliverTargetBoxMemory");
}
END_WORD
REGISTER_WORD(DeliverTargetObject)


WORD(DeliverTargetBoxMemory)
virtual void execute(MachineState * ms) {


  //ms->pushWord("unmapTargetBlueBox");

  // order is crucial here
  ms->pushWord("placeObjectInDeliveryZone");
  ms->pushWord("ifGrasp");

  if (ms->config.currentPlaceMode == PLACE_REGISTER) {
  } else if (ms->config.currentPlaceMode == HAND) {
    ms->pushWord("idler"); 
    ms->pushWord("ifNoGrasp");
  } else if (ms->config.currentPlaceMode == HOLD) {
  } else if (ms->config.currentPlaceMode == SHAKE) {
  } else {
    assert(0);
  }

  ms->pushWord("openGripper");
  ms->pushWord("ifNoGrasp");

  ms->pushWord("tryToMoveToTheLastPrePickHeight");   
  ms->pushWord("ifNoGrasp");

  ms->pushWord("unmapTargetBlueBox");
  ms->pushWord("ifNoGrasp");

  ms->pushWord("checkAndCountGrasp");
  ms->pushWord("ifNoGrasp");

  ms->pushWord("streamGraspResult");
  ms->pushWord("ifNoGrasp");

  ms->pushWord("executePreparedGrasp"); 
  
  ms->pushWord("waitUntilAtCurrentPosition");

  ms->pushWord("sampleHeight"); 
  ms->pushWord("assumeAimedPose"); 

  ms->pushWord("moveAndStreamAimedShot"); 

  ms->pushWord("setBoundingBoxModeToMapping"); 
  ms->pushWord("openGripper");
  ms->pushWord("setPatrolStateToPicking");
}
END_WORD
REGISTER_WORD(DeliverTargetBoxMemory)

WORD(PlaceObjectInDeliveryZone)
virtual void execute(MachineState * ms) {
  if (ms->config.currentPlaceMode == PLACE_REGISTER) {
  ms->pushWord("idler"); 
  ms->pushWord("unmapTargetBlueBox");
  ms->pushWord("checkIfJammed"); 
  ms->pushWord("openGripper"); 
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("tryToMoveToTheLastPrePickHeight");   
  ms->pushWord("departureSpeed");

  if (ms->config.snapToFlushGrasp) {
    ms->evaluateProgram("comeToStop pressUntilEffortInit 0.03 setSpeed pressUntilEffort openGripper quarterImpulse 0.01 setGridSize ( zUp ) 5 setMovementStateToMoving replicateWord comeToHover");
  } else {
    ms->pushWord("openGripper"); 
    ms->pushWord("tryToMoveToTheLastPickHeight");   
    ms->pushWord("approachSpeed"); 
    //ms->pushWord("shiftIntoGraspGear1"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
  }

    ms->pushWord("assumeDeliveryPose");
    ms->pushWord("setPatrolStateToPlacing");
  } else if (ms->config.currentPlaceMode == HAND) {
    ms->pushWord("idler"); 
    ms->pushWord("unmapTargetBlueBox");
    ms->pushWord("openGripper"); 
    ms->pushWord("cruisingSpeed"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord("tryToMoveToTheLastPrePickHeight");   

    ms->pushCopies("localZDown", 5);
    ms->pushWord("setGridSizeCoarse");

    if (1) {
      ms->pushWord("waitForTugThenOpenGripper");

      ms->pushWord("setEffortHere");
      ms->pushWord("setEffortThresh");
      ms->pushWord("7.0");
      ms->pushWord("waitForSeconds");
      ms->pushWord("4.0");
      ms->pushWord("comeToHover");
      ms->pushWord("waitUntilAtCurrentPosition");

    } else {
      // effort logic was moved into waitForTugThenOpenGripper since it
      //  incorporates knowledge about the gripper. 
      // enchanced handoff
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("assumeHandingPose"); 
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("waitUntilGripperNotMoving"); 
      ms->pushWord("openGripper");
      /*
      ms->pushWord("replicateWord"); 
      ms->pushWord("80"); 
      ms->pushData("oXUp"); 
      */


      ms->pushWord("waitUntilEffort");
      ms->pushWord("setEffortThresh");
      ms->pushWord("8.0");


      ms->pushWord("waitForSeconds");
      ms->pushWord("4.0");

      /*
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("replicateWord"); 
      ms->pushWord("80"); 
      ms->pushData("oXDown"); 
      ms->pushWord("replicateWord"); 
      ms->pushWord("10"); 
      ms->pushData("localZDown"); 
      ms->pushWord("setGridSizeCoarse"); 
      */
    }

    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord("assumeHandingPose");
    ms->pushWord("setPatrolStateToHanding");
  } else if (ms->config.currentPlaceMode == HOLD) {
    ms->pushWord("setPatrolStateToHanding");
    ms->pushWord("checkAndCountGrasp");
  } else if (ms->config.currentPlaceMode == SHAKE) {
  ms->pushWord("idler"); 
  ms->pushWord("unmapTargetBlueBox");
  ms->pushWord("checkIfJammed"); 
  ms->pushWord("openGripper"); 
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("tryToMoveToTheLastPrePickHeight");   
  ms->pushWord("departureSpeed");

  ms->config.placeTarget = ms->config.lastPickPose;

  if (ms->config.snapToFlushGrasp) {
    ms->evaluateProgram("comeToStop pressUntilEffortInit 0.03 setSpeed pressUntilEffort openGripper quarterImpulse 0.01 setGridSize ( zUp ) 5 setMovementStateToMoving replicateWord comeToHover");
  } else {
    ms->pushWord("openGripper"); 
    ms->pushWord("tryToMoveToTheLastPickHeight");   
    ms->pushWord("approachSpeed"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
  }

	if (ms->config.setRandomPositionAfterPick) {
	  ms->pushWord("pointToClearanceMap");
	  ms->pushWord("setRandomPositionAndOrientationForHeightLearning");
	} else {
	} // do nothing

    // XXX if SHAKE still works this should be removed
    //ms->pushWord("assumeDeliveryPose");

    ms->pushWord("checkAndCountGrasp");
    ms->pushWord("streamGraspResult");

    ms->pushWord("waitUntilGripperNotMoving");
    ms->pushWord("closeGripper"); 
    ms->pushWord("shakeItUpAndDown"); 
    ms->pushWord("setPatrolStateToPicking");
  } else {
    assert(0);
  }

  if (ms->config.currentPlaceMode == HOLD) {
  } else {
    ms->pushWord("cruisingSpeed");
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord("tryToMoveToTheLastPrePickHeight");   
    ms->pushWord("departureSpeed");
  }
}
END_WORD
REGISTER_WORD(PlaceObjectInDeliveryZone)



WORD(AssumeDeliveryPose)
virtual void execute(MachineState * ms) {
  double oldz = ms->config.currentEEPose.pz;
  ms->config.currentEEPose = ms->config.placeTarget;
  cout << "assumeDeliveryPose: " << ms->config.currentEEPose << endl;
  // so that we hover above where we want to be
  ms->config.currentEEPose.pz = oldz;
  ms->config.lastPickPose.pz = ms->config.placeTarget.pz;
  ms->pushWord("waitUntilAtCurrentPosition");
  //ms->config.currentEEPose.copyQ(ms->config.lastPrePickPose);
  ms->pushWord("assumeLastPickOrientation");
}
END_WORD
REGISTER_WORD(AssumeDeliveryPose)

  
WORD(SetPlaceModeToHold)
virtual void execute(MachineState * ms) {
  ms->config.currentPlaceMode = HOLD;
}
END_WORD
REGISTER_WORD(SetPlaceModeToHold)

WORD(SetPlaceModeToShake)
virtual void execute(MachineState * ms) {
  ms->config.currentPlaceMode = SHAKE;
}
END_WORD
REGISTER_WORD(SetPlaceModeToShake)

WORD(SetPlaceModeToHand)
virtual void execute(MachineState * ms) {
  ms->config.currentPlaceMode = HAND;
}
END_WORD
REGISTER_WORD(SetPlaceModeToHand)

WORD(SetPlaceModeToRegister)
virtual void execute(MachineState * ms) {
  ms->config.currentPlaceMode = PLACE_REGISTER;
}
END_WORD
REGISTER_WORD(SetPlaceModeToRegister)

WORD(ReturnObject)
virtual void execute(MachineState * ms)
{
  cout << "Returning object." << endl;
  if (ms->config.snapToFlushGrasp) {
    ms->pushWord("cruisingSpeed");
    ms->pushWord("goToPrePickPose");
    ms->evaluateProgram("comeToStop pressUntilEffortInit 0.03 setSpeed pressUntilEffort openGripper quarterImpulse 0.01 setGridSize ( zUp ) 5 setMovementStateToMoving replicateWord comeToHover");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("goToPrePickPose");
  } else {
    ms->pushWord("cruisingSpeed");
    ms->pushWord("goToPrePickPose");
    ms->pushWord("waitUntilGripperNotMoving");
    ms->pushWord("openGripper");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("goToLastPickPose");
    ms->pushWord("approachSpeed");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("goToPrePickPose");
  }
}
END_WORD
REGISTER_WORD(ReturnObject)

WORD(SetBreakGraspTiesWithNoise)
virtual void execute(MachineState * ms)
{
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setBreakGraspTiesWithNoise: got value " << valToSet << endl;
  ms->config.breakGraspTiesWithNoise = valToSet;
}
END_WORD
REGISTER_WORD(SetBreakGraspTiesWithNoise)

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
  ms->config.currentStiffnessCommand.data = stiff;
  ms->config.stiffPub.publish(ms->config.currentStiffnessCommand);
}
END_WORD
REGISTER_WORD(SetStiffness)


WORD(TurnAboutY)
virtual void execute(MachineState * ms)
{
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("replicateWord");
    ms->pushWord("160");
    ms->pushData("oYDown");


//    ms->pushWord("waitUntilAtCurrentPosition");
//    ms->pushWord("setGridSizeCoarse");
//    ms->pushWord("shiftIntoGraspGear1");
//    ms->pushWord("changeToHeight");
//    ms->pushWord("1");
//    ms->pushWord("assumeBeeHome");
}
END_WORD
REGISTER_WORD(TurnAboutY)

WORD(UnTurnAboutY)
virtual void execute(MachineState * ms)
{
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("replicateWord");
    ms->pushWord("160");
    ms->pushData("oYUp");


//    ms->pushWord("waitUntilAtCurrentPosition");
//    ms->pushWord("setGridSizeCoarse");
//    ms->pushWord("shiftIntoGraspGear1");
//    ms->pushWord("changeToHeight");
//    ms->pushWord("1");
//    ms->pushWord("assumeBeeHome");
}
END_WORD
REGISTER_WORD(UnTurnAboutY)

WORD(PressAndGrasp)
virtual void execute(MachineState * ms)
{
  double p_rushToHeight = 0.05;
  ms->config.currentEEPose.pz = -ms->config.currentTableZ + ms->config.pickFlushFactor + p_rushToHeight;
  ms->pushWord("pressAndGraspA");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("approachSpeed");
}
END_WORD
REGISTER_WORD(PressAndGrasp)

WORD(PressAndGraspA)
virtual void execute(MachineState * ms)
{

  if (ms->config.currentMovementState == BLOCKED) {

    /*
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    */

    ms->pushWord("waitUntilGripperNotMoving");
    ms->pushWord("closeGripper");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("setMovementStateToMoving");

    ms->pushWord("setGridSizeCoarse");
    ms->pushWord("zDown");
    ms->pushWord("zDown");
    ms->pushWord("zDown");
    ms->pushWord("setGridSizeFine");

    ms->pushWord("zDown");
    ms->pushWord("zDown");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("zUp");
    ms->pushWord("zUp");
    ms->pushWord("zeroGOff");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("zeroGOn");
  } else {
    ms->pushWord("pressAndGraspA");
    ms->pushWord("waitUntilEndpointCallbackReceived");

    ms->pushWord("waitForSeconds");
    ms->pushWord("1.0"); 

    ms->pushWord("zDown");
    ms->pushWord("setGridSizeCoarse");
  }
}
END_WORD
REGISTER_WORD(PressAndGraspA)

WORD(SetPressPose)
virtual void execute(MachineState * ms)
{
  ms->config.pressPose = ms->config.currentEEPose;
}
END_WORD
REGISTER_WORD(SetPressPose)

WORD(PressAndRelease)
virtual void execute(MachineState * ms)
{
  ms->pushWord("pressAndReleaseA");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("approachSpeed");
  ms->pushWord("setPressPose");
}
END_WORD
REGISTER_WORD(PressAndRelease)

WORD(PressAndReleaseA)
virtual void execute(MachineState * ms)
{

  double qdistance = eePose::distanceQ(ms->config.pressPose, ms->config.trueEEPoseEEPose);

cout << "pressAndReleaseA: qdistance " << qdistance << endl;

  // qdistance might actually be enough by itself
  if ( (ms->config.currentMovementState == BLOCKED) || (qdistance > ms->config.twistThresh) ){


    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");
    ms->pushWord("localZDown");


    ms->pushWord("waitForSeconds");
    ms->pushWord("0.50"); 

    ms->pushWord("waitUntilGripperNotMoving");

    ms->pushWord("openGripper");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("zDown");
    ms->pushWord("zDown");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("zUp");
    ms->pushWord("zUp");

    ms->pushWord("zeroGOff");
    ms->pushWord("waitUntilEndpointCallbackReceived");
    ms->pushWord("zeroGOn");
  } else {
    ms->pushWord("pressAndReleaseA");

    ms->pushWord("waitForSeconds");
    ms->pushWord("1.0"); 
    ms->pushWord("zDown");
    ms->pushWord("setGridSizeCoarse");
  }
}
END_WORD
REGISTER_WORD(PressAndReleaseA)

WORD(RegisterWrench)
virtual void execute(MachineState * ms)
{
  ms->config.targetWrench = ms->config.trueEEWrench; 

  ms->pushWord("registerWrenchA");
}
END_WORD
REGISTER_WORD(RegisterWrench)

WORD(RegisterWrenchA)
virtual void execute(MachineState * ms)
{
  cout << "registerWrenchA: ";
  double totalDiff = 0.0;

  totalDiff = eePose::distance(ms->config.targetWrench, ms->config.trueEEWrench);

  cout << endl << "  totalDiff: " << totalDiff << "   actual_effort_thresh: " << ms->config.actual_effort_thresh << endl;

  if (totalDiff > ms->config.actual_effort_thresh) {
    cout << "~~~~~~~~" << endl << endl << endl << endl;
    //ms->pushWord("registerWrench");
    ms->pushWord("registerWrenchA");
  } else {
    ms->pushWord("registerWrenchA");
  }

  ms->pushWord("endStackCollapseNoop");
}
END_WORD
REGISTER_WORD(RegisterWrenchA)

WORD(WaitUntilEffort)
virtual void execute(MachineState * ms)
{
  ms->config.waitUntilEffortCounter = 0;
  ms->pushWord("waitUntilEffortA");
  ms->pushWord("setEffortHere");
}
END_WORD
REGISTER_WORD(WaitUntilEffort)

WORD(WaitUntilEffortA)
virtual void execute(MachineState * ms)
{
  if (ms->config.waitUntilEffortCounter < ms->config.waitUntilEffortCounterTimeout) {

    double totalDiff = 0.0;
    for (int i = 0; i < NUM_JOINTS; i++) {
      double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
      //cout << ms->config.target_joint_actual_effort[i] << " " << ms->config.last_joint_actual_effort[i] << " " << thisDiff << " ";
      totalDiff = totalDiff + (thisDiff * thisDiff);
    }

    //cout << endl << "  totalDiff: " << totalDiff << "   actual_effort_thresh: " << ms->config.actual_effort_thresh << endl;

    ms->config.waitUntilEffortCounter++;

    if (totalDiff > ms->config.actual_effort_thresh) {
      //cout << "~~~~~~~~" << endl << endl << endl << endl;
      cout << "~~~~~~~~ felt effort" << endl;
    } else {
      ms->pushWord("waitUntilEffortA");
    }
  } else {
    cout << "Warning: waitUntilEffort timed out, moving on." << endl;
    cout << "Warning: not moving on so we don't drop objects..." << endl;
    ms->pushWord("waitUntilEffortA");
  }

  ms->pushWord("endStackCollapseNoop");
}
END_WORD
REGISTER_WORD(WaitUntilEffortA)

WORD(RegisterEffort)
virtual void execute(MachineState * ms)
{
  ms->pushWord("registerEffortA");
  ms->pushWord("setEffortHere");
}
END_WORD
REGISTER_WORD(RegisterEffort)

WORD(RegisterEffortA)
virtual void execute(MachineState * ms)
{
  cout << "registerEffortA: ";
  double totalDiff = 0.0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
    cout << ms->config.target_joint_actual_effort[i] << " " << ms->config.last_joint_actual_effort[i] << " " << thisDiff << " ";
    totalDiff = totalDiff + (thisDiff * thisDiff);
  }

  cout << endl << "  totalDiff: " << totalDiff << "   actual_effort_thresh: " << ms->config.actual_effort_thresh << endl;

  if (totalDiff > ms->config.actual_effort_thresh) {
    cout << "~~~~~~~~" << endl << endl << endl << endl;
    //ms->pushWord("registerEffort");
    ms->pushWord("registerEffortA");
  } else {
    ms->pushWord("registerEffortA");
  }

  ms->pushWord("endStackCollapseNoop");
}
END_WORD
REGISTER_WORD(RegisterEffortA)

WORD(SetEffortHere)
virtual void execute(MachineState * ms)
{
  for (int i = 0; i < NUM_JOINTS; i++) {
    ms->config.target_joint_actual_effort[i] = ms->config.last_joint_actual_effort[i];
  }
  cout << "setEffortHere" << endl;
}
END_WORD
REGISTER_WORD(SetEffortHere)


WORD(PressUntilEffortAllInOne)
virtual string description() {
  return "Push down until there is effort sensed; top level word that can be called all in one.";
}
virtual void execute(MachineState * ms)
{

  ms->evaluateProgram("comeToStop pressUntilEffortInit 0.03 setSpeed pressUntilEffortCombo");
}
END_WORD
REGISTER_WORD(PressUntilEffortAllInOne)


WORD(PressUntilEffortInit)
virtual string description() {
  return "Initialize parameters for pressUntilEffort.";
}
virtual void execute(MachineState * ms)
{
  ms->pushWord("setEffortThresh");
  ms->pushWord("7.0");

  //ms->pushWord("hundredthImpulse");
  ms->pushWord("setSpeed");
  ms->pushWord("0.01");
  ms->pushWord("setGridSize");
  ms->pushWord("0.005");
  ms->pushWord("setW1GoThresh");
  ms->pushWord("0.005");
}
END_WORD
REGISTER_WORD(PressUntilEffortInit)

WORD(PressUntilEffort)
virtual void execute(MachineState * ms)
{
  ms->pushWord("waitSetCurrentWaitMode");
  ms->config.pressUntilEffortStart = ros::Time::now();
  ms->pushWord("bringUpAllNonessentialSystems");
  ms->pushWord("pressUntilEffortA");
  ms->pushWord("setEffortHere");
  ms->pushWord("shutdownToSensorsAndMovement");
  ms->pushWord("waitSetCurrentWaitMode");
    ms->pushWord("1");
    ms->pushWord("waitGetCurrentWaitMode");
}
END_WORD
REGISTER_WORD(PressUntilEffort)

WORD(PressUntilEffortA)
virtual void execute(MachineState * ms)
{
  if ( ros::Time::now().toSec() - ms->config.pressUntilEffortStart.toSec() < ms->config.pressUntilEffortTimeout ) {
    //cout << "pressUntilEffortA: ";
    double totalDiff = 0.0;
    for (int i = 0; i < NUM_JOINTS; i++) {
      double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
      //cout << ms->config.target_joint_actual_effort[i] << " " << ms->config.last_joint_actual_effort[i] << " " << thisDiff << " ";
      totalDiff = totalDiff + (thisDiff * thisDiff);
    }

    //cout << endl << "  totalDiff: " << totalDiff << "   actual_effort_thresh: " << ms->config.actual_effort_thresh << endl;

    if (totalDiff > ms->config.actual_effort_thresh) {
      cout << "~~~~~~~~" << endl << "crossed effort thresh: " << totalDiff << " thresh: " << ms->config.actual_effort_thresh << endl << endl;
      ms->pushWord("stayZOnly");
    } else {
      ms->pushWord("pressUntilEffortA");
      if (eePose::distance(ms->config.currentEEPose, ms->config.trueEEPoseEEPose) < ms->config.w1GoThresh) {
	cout << "nudging" << endl;
	// the effort measurement drifts natural when the arm moves, even under no load,
	//  and one way of dealing with this is to reset the effort every so often. It would be
	//  smoother to do this in a continuous way, like exponential average, but it is not
	//  clear what the most natural way is.
	ms->pushWord("replicateWord");
	ms->pushWord("2");
	ms->pushData("localZUp");
	ms->pushWord("setGridSizeCoarse");
	//ms->pushWord("replicateWord");
	//ms->pushWord("5");
	//ms->pushData("zDown");
	ms->pushWord("setEffortHere");
      } else {
      }
    }
  } else {
    cout << "Warning: pressUntilEffort timed out, moving on." << endl;
  }

  ms->pushWord("endStackCollapseNoop");
}
END_WORD
REGISTER_WORD(PressUntilEffortA)

WORD(PressUntilEffortOrTwistInit)
virtual void execute(MachineState * ms)
{
  ms->pushWord("setEffortThresh");
  ms->pushWord("7.0");
  ms->pushWord("setTwistThresh");
  ms->pushWord("0.02");

  //ms->pushWord("hundredthImpulse");
  ms->pushWord("setSpeed");
  ms->pushWord("0.01");
  ms->pushWord("setGridSize");
  ms->pushWord("0.005");
  ms->pushWord("setW1GoThresh");
  ms->pushWord("0.005");
}
END_WORD
REGISTER_WORD(PressUntilEffortOrTwistInit)

WORD(PressUntilEffortOrTwist)
virtual void execute(MachineState * ms)
{
  ms->config.pressUntilEffortStart = ros::Time::now();
  ms->pushWord("waitSetCurrentWaitMode");
  ms->pushWord("bringUpAllNonessentialSystems");
  ms->pushWord("pressUntilEffortOrTwistA");
  ms->pushWord("setEffortHere");
  ms->pushWord("setPressPose");
  ms->pushWord("shutdownToSensorsAndMovement");
  ms->pushWord("waitSetCurrentWaitMode");
    ms->pushWord("1");
    ms->pushWord("waitGetCurrentWaitMode");
}
END_WORD
REGISTER_WORD(PressUntilEffortOrTwist)

WORD(PressUntilEffortStart)
virtual void execute(MachineState * ms)
{
  ms->pushWord("setSpeed");
  ms->pushWord("waitSetCurrentWaitMode");
  ms->evaluateProgram("0.01 setSpeed 0.01 setW1GoThresh 0.02 setGridSize zDown waitUntilAtCurrentPosition");
  ms->pushWord("waitSetCurrentWaitMode");
    ms->pushWord("1");
    ms->pushWord("waitGetCurrentWaitMode");
    ms->pushWord("getSpeed");
}
END_WORD
REGISTER_WORD(PressUntilEffortStart)

WORD(PressUntilEffortCombo)
virtual string description() {
  return "Press down until effort is larger than a threshold.  Pushes down once more quickly with a coarse press, then does a fine-grained press (to unwined the springs), so at the end you are lightly touching the surface.";
}
virtual void execute(MachineState * ms)
{
  // restore
  ms->pushWord("setW1GoThresh");

  ms->pushWord("localZDown");
  ms->pushWord("setGridSize");
  ms->pushWord("0.00125");
  ms->pushWord("pressUntilEffort");
  ms->pushWord("setGridSize");
  ms->pushWord("0.00125");
  ms->pushWord("setW1GoThresh");
  ms->pushWord("0.00125");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("localZDown");
  ms->pushWord("setGridSize");
  ms->pushWord("0.005");
  ms->pushWord("pressUntilEffort");
  ms->pushWord("setGridSize");
  ms->pushWord("0.005");
  ms->pushWord("setW1GoThresh");
  ms->pushWord("0.005");

  
  // memorize
    ms->pushWord("w1GoThresh");

  ms->pushWord("pressUntilEffortStart");
}
END_WORD
REGISTER_WORD(PressUntilEffortCombo)

WORD(PressUntilEffortOrTwistA)
virtual void execute(MachineState * ms)
{
  double qdistance = eePose::distanceQ(ms->config.pressPose, ms->config.trueEEPoseEEPose);

cout << "pressUntilEffortOrTwistA: qdistance, twistThresh" << qdistance << ", " << ms->config.twistThresh << endl;

  if ( ros::Time::now().toSec() - ms->config.pressUntilEffortStart.toSec() < ms->config.pressUntilEffortTimeout ) {
    cout << "pressUntilEffortOrTwistA: ";
    double totalDiff = 0.0;
    for (int i = 0; i < NUM_JOINTS; i++) {
      double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
      cout << ms->config.target_joint_actual_effort[i] << " " << ms->config.last_joint_actual_effort[i] << " " << thisDiff << " ";
      totalDiff = totalDiff + (thisDiff * thisDiff);
    }

    cout << endl << "  totalDiff: " << totalDiff << "   actual_effort_thresh: " << ms->config.actual_effort_thresh << endl;

    if (totalDiff > ms->config.actual_effort_thresh) {
      cout << "~~~~~~~~" << endl << "crossed effort thresh" << endl << endl;
      ms->pushWord("stayZOnly");
    } else if (qdistance > ms->config.twistThresh) {
      cout << "^^^^^^^^" << endl << "crossed twist thresh" << endl << endl;
      ms->pushWord("stayZOnly");
    } else {
      ms->pushWord("pressUntilEffortOrTwistA");
      if (eePose::distance(ms->config.currentEEPose, ms->config.trueEEPoseEEPose) < ms->config.w1GoThresh) {
	cout << "nudging" << endl;
	// the effort measurement drifts natural when the arm moves, even under no load,
	//  and one way of dealing with this is to reset the effort every so often. It would be
	//  smoother to do this in a continuous way, like exponential average, but it is not
	//  clear what the most natural way is.
	ms->pushWord("replicateWord");
	ms->pushWord("2");
	ms->pushData("localZUp");
	ms->pushWord("setGridSizeCoarse");
	//ms->pushWord("replicateWord");
	//ms->pushWord("5");
	//ms->pushData("zDown");
	ms->pushWord("setEffortHere");
      } else {
      }
    }
  } else {
    cout << "Warning: pressUntilEffortOrTwist timed out, moving on." << endl;
  }
  ms->pushWord("endStackCollapseNoop");
}
END_WORD
REGISTER_WORD(PressUntilEffortOrTwistA)

WORD(Stay)
virtual void execute(MachineState * ms)
{
  cout << "Stay!" << endl;
  ms->config.currentEEPose = ms->config.trueEEPoseEEPose;
}
END_WORD
REGISTER_WORD(Stay)

WORD(StayNoRoll)
virtual void execute(MachineState * ms)
{
  cout << "StayNoRoll!" << endl;
  ms->config.currentEEPose.copyP(ms->config.trueEEPoseEEPose);
}
END_WORD
REGISTER_WORD(StayNoRoll)

WORD(StayZOnly)
virtual void execute(MachineState * ms)
{
  cout << "StayZOnly!" << endl;
  ms->config.currentEEPose.pz = ms->config.trueEEPoseEEPose.pz;
}
END_WORD
REGISTER_WORD(StayZOnly)

WORD(RockInit)
virtual void execute(MachineState * ms)
{

  ms->pushWord("setSpeed");
  ms->pushWord("0.07");
  //ms->pushWord("setEffortHere");
  ms->pushWord("saveRegister1");
}
END_WORD
REGISTER_WORD(RockInit)

WORD(Rock)
virtual void execute(MachineState * ms)
{

  ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("replicateWord");
  //ms->pushWord("5");
  //ms->pushData("zUp");
  ms->pushWord("tenthImpulse");
  ms->pushWord("rockD");

  ms->pushWord("rockC");

  ms->pushWord("replicateWord");
  ms->pushWord("10");
  ms->pushData("oXUp");
  ms->pushWord("setGridSizeCoarse");


  //ms->pushWord("replicateWord");
  //ms->pushWord("10");
  //ms->pushData("oXDown");

  ms->pushWord("rockB");
  //ms->pushWord("waitUntilAtCurrentPosition");

  ms->pushWord("waitForSeconds");
  ms->pushWord("1.0");

  ms->pushWord("replicateWord");
  ms->pushWord("20");
  ms->pushData("oXDown");
  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("stayNoRoll");
  ms->pushWord("pressUntilEffort");
  ms->pushWord("pressUntilEffortInit");

  ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("replicateWord");
  //ms->pushWord("5");
  //ms->pushData("zUp");
  ms->pushWord("tenthImpulse");
  ms->pushWord("rockD");

  //ms->pushWord("replicateWord");
  //ms->pushWord("10");
  //ms->pushData("oXUp");

  ms->pushWord("rockA");
  //ms->pushWord("waitUntilAtCurrentPosition");

  ms->pushWord("waitForSeconds");
  ms->pushWord("1.0");

  ms->pushWord("replicateWord");
  ms->pushWord("10");
  ms->pushData("oXUp");
  ms->pushWord("setGridSizeCoarse");
  

  ms->pushWord("stayNoRoll");
  ms->pushWord("pressUntilEffort");
  ms->pushWord("pressUntilEffortInit");

  ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("replicateWord");
  //ms->pushWord("5");
  //ms->pushData("zUp");
  ms->pushWord("tenthImpulse");
  ms->pushWord("rockD");

  if (eePose::distance(ms->config.currentEEPose, ms->config.trueEEPoseEEPose) < ms->config.w1GoThresh) {
    cout << "nudging" << endl;
//    ms->pushWord("localZUp");
    //ms->pushWord("setEffortHere");
    //ms->pushWord("setGridSize");
    //   ms->pushWord("0.004");

  } else {
  }
}
END_WORD
REGISTER_WORD(Rock)

WORD(RockA)
virtual void execute(MachineState * ms)
{
  double totalDiff = 0.0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
    totalDiff = totalDiff + (thisDiff * thisDiff);
  }

//  double rockAmp= 2.0 * 0.01;
//  double rockMax = 200.0;
//
//  double rockSnapped = min( max(0.0, totalDiff), rockMax);
//
//  ms->config.bDelta = min( max(0.0, rockAmp * (rockMax - rockSnapped) / rockMax), 0.02);
//
//
//  cout << "rockA set bDelta to " << ms->config.bDelta << endl << "  totalDiff: " << totalDiff << endl;;
  ms->config.rockDiffA = totalDiff;

  cout << "rockA totalDiff: " << totalDiff << endl;
}
END_WORD
REGISTER_WORD(RockA)

WORD(RockB)
virtual void execute(MachineState * ms)
{
  double totalDiff = 0.0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
    totalDiff = totalDiff + (thisDiff * thisDiff);
  }

  ms->config.rockDiffB = totalDiff;
  cout << "rockB totalDiff: " << totalDiff << endl;
}
END_WORD
REGISTER_WORD(RockB)

WORD(RockC)
virtual void execute(MachineState * ms)
{
  if (ms->config.rockDiffB  > ms->config.rockDiffA) {
    ms->pushWord("waitUntilAtCurrentPosition");

    ms->pushWord("waitForSeconds");
    ms->pushWord("1.0");

    ms->pushWord("replicateWord");
    ms->pushWord("5");
    ms->pushData("oXUp");
    ms->pushWord("setGridSizeCoarse");

    cout << "rockC: A won" << endl;
  } else {
    ms->pushWord("waitUntilAtCurrentPosition");

    ms->pushWord("waitForSeconds");
    ms->pushWord("1.0");

    ms->pushWord("replicateWord");
    ms->pushWord("5");
    ms->pushData("oXDown");
    ms->pushWord("setGridSizeCoarse");

    cout << "rockC: B won" << endl;
  }

}
END_WORD
REGISTER_WORD(RockC)

WORD(RockDA)
virtual void execute(MachineState * ms)
{
  ms->config.currentEEPose.copyP(ms->config.eepReg1);
}
END_WORD
REGISTER_WORD(RockDA)

WORD(RockD)
virtual void execute(MachineState * ms)
{
  ms->pushWord("rockDA");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("replicateWord");
  ms->pushWord("7");
  ms->pushData("zUp");
  ms->pushWord("tenthImpulse");
}
END_WORD
REGISTER_WORD(RockD)

WORD(Roll)
virtual void execute(MachineState * ms)
{
}
END_WORD
REGISTER_WORD(Roll)


WORD(PickClosestBlueBox)
virtual void execute(MachineState * ms) {
  double bestDistance = VERYBIGNUMBER;
  BoxMemory bestBox;
  int bestI;

  for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
    BoxMemory memory = ms->config.blueBoxMemories[i];
    string class_name = ms->config.classLabels[memory.labeledClassIndex];
    double newDistance = eePose::distance(memory.aimedPose, ms->config.currentEEPose);
    if (newDistance < bestDistance) {
      bestBox = memory;
      bestDistance = newDistance;
      bestI = i;
    }
  }

  if (bestDistance < VERYBIGNUMBER) {
    cout << "Got: " << bestBox.labeledClassIndex << ": " << ms->config.classLabels[bestBox.labeledClassIndex] << endl;
    targetBoxMemory(ms, bestI);
    ms->pushWord("deliverTargetBoxMemory");
    ms->execute_stack = 1;
  }
}
END_WORD
REGISTER_WORD(PickClosestBlueBox)


}


