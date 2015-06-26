#include "ein_words.h"
#include "ein.h"
namespace ein_words {


WORD(CornellMugsOnTables)
virtual void execute(std::shared_ptr<MachineState> ms) {
  
  double cTableHeight = -0.045;//0.0;//0.025;
  int amountMms = floor(cTableHeight / 0.001);

  /*eePose table3Pose;
  int success = 1;
  success = success && placementPoseLabel1AboveLabel2By(ms, "brownMug", "table3", cTableHeight, &table3Pose);
  if (success) {
    ms->pushWord(std::make_shared<EePoseWord>(table3Pose));
    ms->pushWord("brownMug");
    ms->pushWord("moveObjectToPose");
  }
  */

  ms->pushWord(std::make_shared<IntegerWord>(amountMms));
  ms->pushWord("table3");
  ms->pushWord("brownMug");
  ms->pushWord("moveObjectToObjectByAmount");

  ms->pushWord(std::make_shared<IntegerWord>(amountMms));
  ms->pushWord("table2");
  ms->pushWord("metalMug");
  ms->pushWord("moveObjectToObjectByAmount");

  ms->pushWord(std::make_shared<IntegerWord>(amountMms));
  ms->pushWord("table1");
  ms->pushWord("redMug");
  ms->pushWord("moveObjectToObjectByAmount");


  /*
  if (success) {
    ms->pushWord(std::make_shared<EePoseWord>(table3Pose));
    ms->pushWord("brownMug");
    ms->pushWord("moveObjectToPose");

    ms->pushWord(std::make_shared<EePoseWord>(table2Pose));
    ms->pushWord("metalMug");
    ms->pushWord("moveObjectToPose");

    ms->pushWord(std::make_shared<EePoseWord>(table1Pose));
    ms->pushWord("redMug");
    ms->pushWord("moveObjectToPose");
  } else {
    cout << "some objects not found.
  */
}
END_WORD
REGISTER_WORD(CornellMugsOnTables)

WORD(MoveObjectToObjectByAmount)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> firstObjectWord = ms->popWord();
  shared_ptr<Word> secondObjectWord = ms->popWord();
  shared_ptr<Word> awPre = ms->popWord();
  std::shared_ptr<IntegerWord> amountWord = std::dynamic_pointer_cast<IntegerWord>(awPre);

  if( (firstObjectWord == NULL) || (secondObjectWord == NULL) || (amountWord == NULL) ) {
    cout << "not enough words... clearing stack." << endl;
    ms->clearStack();
    return;
  } else {
    string firstObjectLabel = firstObjectWord->to_string();
    string secondObjectLabel = secondObjectWord->to_string();
    // this is specified in mm
    int amountMms = amountWord->value();
    double cTableHeight = 0.001 * amountMms;

    eePose placePose;
    int success = placementPoseLabel1AboveLabel2By(ms, firstObjectLabel, secondObjectLabel, cTableHeight, &placePose);
    if (success) {
      ms->pushWord(std::make_shared<EePoseWord>(placePose));
      ms->pushWord(firstObjectLabel);
      ms->pushWord("moveObjectToPose");
    }
  }
}
END_WORD
REGISTER_WORD(MoveObjectToObjectByAmount)

WORD(MoveObjectBetweenObjectAndObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> firstObjectWord = ms->popWord();
  shared_ptr<Word> secondObjectWord = ms->popWord();
  shared_ptr<Word> thirdObjectWord = ms->popWord();

  if( (firstObjectWord == NULL) || (secondObjectWord == NULL) || (thirdObjectWord == NULL) ) {
    cout << "not enough words... clearing stack." << endl;
    ms->clearStack();
    return;
  } else {
    string firstObjectLabel = firstObjectWord->to_string();
    string secondObjectLabel = secondObjectWord->to_string();
    string thirdObjectLabel = thirdObjectWord->to_string();

    eePose placePose;
    int success = placementPoseLabel1BetweenLabel2AndLabel3(ms, firstObjectLabel, secondObjectLabel, thirdObjectLabel, &placePose);
    if (success) {
      ms->pushWord(std::make_shared<EePoseWord>(placePose));
      ms->pushWord(firstObjectLabel);
      ms->pushWord("moveObjectToPose");
    }
  }
}
END_WORD
REGISTER_WORD(MoveObjectBetweenObjectAndObject)

WORD(SetTheYcbTable)
virtual void execute(std::shared_ptr<MachineState> ms) {

// XXX broken unless you do something with AssumeDeliveryPose
// XXX ms->config.currentEEPose.pz = oldz;
  ms->pushWord("assumeCrane1"); 

  //eePose platePose = {.px = 0.602935, .py = 0.599482, .pz = -0.0395161,
                      //.qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  //ms->pushWord(std::make_shared<EePoseWord>(platePose));
  //ms->pushWord("redPlate");
  //ms->pushWord("moveObjectToPose");

  eePose mugPose = {.px = 0.428236, .py = 0.688348, .pz = -0.026571,
                      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  ms->pushWord(std::make_shared<EePoseWord>(mugPose));
  ms->pushWord("redMugLow");
  ms->pushWord("moveObjectToPose");

  eePose bowlPose = {.px = 0.429551, .py = 0.355954, .pz = -0.02713,
                     .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  ms->pushWord(std::make_shared<EePoseWord>(bowlPose));
  ms->pushWord("redBowlLow");
  ms->pushWord("moveObjectToPose");

  eePose knifePose = {.px = 0.645808, .py = 0.75673, .pz = -0.0605177,
                      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  ms->pushWord(std::make_shared<EePoseWord>(knifePose));
  ms->pushWord("redKnifeLow");
  ms->pushWord("moveObjectToPose");


  eePose forkPose =  {.px = 0.632388, .py = 0.417448, .pz = -0.0487945,
                      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  ms->pushWord(std::make_shared<EePoseWord>(forkPose));
  ms->pushWord("redForkLow");
  ms->pushWord("moveObjectToPose");


  eePose spoonPose = {.px = 0.624147, .py = 0.811554, .pz = -0.0696885,
                      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  ms->pushWord(std::make_shared<EePoseWord>(spoonPose));
  ms->pushWord("redSpoonLow");
  ms->pushWord("moveObjectToPose");
}
END_WORD
REGISTER_WORD(SetTheYcbTable)

WORD(MoveObjectToPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> objectword = ms->popWord();
  if (objectword == NULL) {
    cout << "Must pass a string as an argument to " << this->name() << endl;
  } else {
    string className = objectword->to_string();
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
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> word = ms->popWord();
  std::shared_ptr<EePoseWord> destWord = std::dynamic_pointer_cast<EePoseWord>(word);
  if (destWord == NULL) {
    cout << "Must pass a pose as an argument to " << this->name() << endl;
    cout <<" Instead got word: " << word->name() << " repr: " << word->repr() << endl;
    return;
  } else {
    ms->config.placeTarget = destWord->value();
  
    ms->pushWord("deliverTargetObject");
    ms->pushWord("setPlaceModeToRegister");
  }
}
END_WORD
REGISTER_WORD(MoveTargetObjectToPose)

WORD(DeliverObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> word = ms->popWord();
  if (word == NULL) {
    cout << "Must pass a string as an argument to " << this->name() << endl;
  } else {
    string className = word->to_string();
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

WORD(DeliverTargetObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //ms->pushWord("idler"); 
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
    return;
  }

  if (focusedClassMemories.size() > 1) {
    cout << "More than one bounding box for class.  Looking for first POSE_REPORTED." << focusedClassMemories.size() << endl;
  } else {
  } // do nothing

  if (idxOfFirst == -1) {
    cout << "No POSE_REPORTED objects of the focused class." << endl;
    return;
  } else {
  } // do nothing

  BoxMemory memory = ms->config.blueBoxMemories[idxOfFirst];


  cout << "Aimed pose: " << memory.aimedPose << endl;
  //ms->config.currentEEPose = memory.cameraPose;
  ms->config.currentEEPose = memory.aimedPose;
  // lastPickPose is suspect
  ms->config.lastPickPose = memory.pickedPose;
  ms->config.lastPrePickPose = memory.aimedPose;
  ms->config.trZ = memory.trZ;

  { // set the old box's lastMappedTime to moments after the start of time
    int iStart=-1, iEnd=-1, jStart=-1, jEnd=-1;
    int iTop=-1, iBot=-1, jTop=-1, jBot=-1;
    double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;
    {
      double x, y;
      int i, j;
      pixelToGlobal(ms, memory.top.px-ms->config.mapBlueBoxPixelSkirt, memory.top.py-ms->config.mapBlueBoxPixelSkirt, z, &x, &y);
      mapxyToij(ms,x, y, &i, &j);
      iTop=i;
      jTop=j;
    }
    {
      double x, y;
      int i, j;
      pixelToGlobal(ms, memory.bot.px+ms->config.mapBlueBoxPixelSkirt, memory.bot.py+ms->config.mapBlueBoxPixelSkirt, z, &x, &y);
      mapxyToij(ms,x, y, &i, &j);
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
      if (i != idxOfFirst) {
	cout << "Retaining blue box " << i << " while booting " << idxOfFirst << endl; 
	newMemories.push_back(ms->config.blueBoxMemories[i]);
      }
    }
    ms->config.blueBoxMemories = newMemories;
  }

  //ms->pushWord("moveToNextMapPosition");
  //ms->pushWord("synchronicServoDoNotTakeClosest"); 
  ms->pushWord("openGripper"); 
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("tryToMoveToTheLastPrePickHeight");   
  ms->pushWord("departureSpeed");
  ms->pushWord("placeObjectInDeliveryZone");
  ms->pushWord("ifGrasp");
  ms->pushWord("executePreparedGrasp"); 
  //ms->pushWord("prepareForAndExecuteGraspFromMemory"); 
  //ms->pushWord("gradientServo");
  //ms->pushCopies("density", ms->config.densityIterationsForGradientServo); 
  //ms->pushCopies("resetTemporalMap", 1); 
  //ms->pushWord("synchronicServo"); 
  //ms->pushWord("visionCycle");
  //ms->pushWord("synchronicServoTakeClosest"); 
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("setPickModeToStaticMarginals"); 
  ms->pushWord("sampleHeight"); 
  ms->pushWord("setBoundingBoxModeToMapping"); 
  ms->pushWord("openGripper");
  ms->pushWord("setPatrolStateToPicking");
}
END_WORD
REGISTER_WORD(DeliverTargetObject)

WORD(PlaceObjectInDeliveryZone)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.currentPlaceMode == PLACE_REGISTER) {
    ms->pushWord("openGripper"); 
    ms->pushWord("tryToMoveToTheLastPickHeight");   
    ms->pushWord("approachSpeed"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord("assumeDeliveryPose");
    ms->pushWord("setPatrolStateToPlacing");
  } else if (ms->config.currentPlaceMode == HAND) {
    ms->pushCopies("localZDown", 5);
    ms->pushWord("setMovementSpeedMoveFast");
    ms->pushWord("waitForTugThenOpenGripper");
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord("assumeHandingPose");
    ms->pushWord("setPatrolStateToHanding");
  } else if (ms->config.currentPlaceMode == SHAKE) {
    ms->config.placeTarget = ms->config.lastPickPose;
    ms->pushWord("openGripper"); 
    ms->pushWord("tryToMoveToTheLastPickHeight");   
    ms->pushWord("approachSpeed"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord("assumeDeliveryPose");

    ms->pushWord("checkAndCountGrasp");
    ms->pushWord("waitUntilGripperNotMoving");
    ms->pushWord("closeGripper"); 
    ms->pushWord("shakeItUpAndDown"); 
    ms->pushWord("setPatrolStateToPicking");
  } else {
    assert(0);
  }

  ms->pushWord("cruisingSpeed");
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("tryToMoveToTheLastPrePickHeight");   
  ms->pushWord("departureSpeed");
}
END_WORD
REGISTER_WORD(PlaceObjectInDeliveryZone)



WORD(AssumeDeliveryPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double oldz = ms->config.currentEEPose.pz;
  ms->config.currentEEPose = ms->config.placeTarget;
  cout << "XXX assumeDeliveryPose: " << ms->config.currentEEPose << endl;
  // so that we hover above where we want to be
  ms->config.currentEEPose.pz = oldz;
  ms->config.currentEEPose.copyQ(ms->config.lastPrePickPose);
  ms->config.lastPickPose.pz = ms->config.placeTarget.pz;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeDeliveryPose)

  

WORD(SetPlaceModeToShake)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPlaceMode = SHAKE;
}
END_WORD
REGISTER_WORD(SetPlaceModeToShake)

WORD(SetPlaceModeToHand)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPlaceMode = HAND;
}
END_WORD
REGISTER_WORD(SetPlaceModeToHand)

WORD(SetPlaceModeToRegister)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentPlaceMode = PLACE_REGISTER;
}
END_WORD
REGISTER_WORD(SetPlaceModeToRegister)

WORD(ReturnObject)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  cout << "Returning object." << endl;
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
END_WORD
REGISTER_WORD(ReturnObject)


}


