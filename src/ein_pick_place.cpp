
#include "ein_words.h"
#include "ein.h"
namespace ein_words {

WORD(MoveObjectToPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> objectword = ms->popWord();
  string className = objectword->to_string();
  int class_idx = classIdxForName(ms, className);
  if (class_idx != -1) {
    ms->pushWord("moveTargetObjectToPose");
    changeTargetClass(ms, class_idx);
  } else {
    cout << "No class for " << className << " for " << this->name() << endl;
  }
}
END_WORD
REGISTER_WORD(MoveObjectToPose)

WORD(MoveTargetObjectToPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> word = ms->popWord();
  cout << "Word: " << word << " null? " << (word == NULL) << endl;
  cout <<" word: " << word->to_string() << endl;
  std::shared_ptr<EePoseWord> destWord = std::dynamic_pointer_cast<EePoseWord>(word);
  if (destWord == NULL) {
    cout << "Must pass a pose as an argument to" << this->name() << endl;
    return;
  }
  eePose dest = destWord->value();

  cout << "Actually do the move of " << ms->config.focusedClassLabel << " to " << dest << endl;

}
END_WORD
REGISTER_WORD(MoveTargetObjectToPose)



WORD(DeliverObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> word = ms->popWord();
  string className = word->to_string();
  int class_idx = classIdxForName(ms, className);
  if (class_idx != -1) {
    changeTargetClass(ms, class_idx);
    ms->pushWord("deliverTargetObject");
  } else {
    cout << "No class for " << className << " for " << this->name() << endl;
  }
}
END_WORD
REGISTER_WORD(DeliverObject)


WORD(DeliverTargetObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("idler"); 
  ms->config.bailAfterGradient = 1;

  ms->config.pilotTarget.px = -1;
  ms->config.pilotTarget.py = -1;
  ms->config.pilotClosestTarget.px = -1;
  ms->config.pilotClosestTarget.py = -1;
  
  int idxOfFirst = -1;
  vector<BoxMemory> focusedClassMemories = memoriesForClass(ms, ms->config.focusedClass, &idxOfFirst);
  if (focusedClassMemories.size() == 0) {
    cout << "No memories of the focused class. " << endl;
    return;
  }
  if (focusedClassMemories.size() > 1) {
    cout << "More than one bounding box for class.  Looking for first POSE_REPORTED." << focusedClassMemories.size() << endl;
  }
  //BoxMemory memory = focusedClassMemories[0];
  BoxMemory memory = ms->config.blueBoxMemories[idxOfFirst];

  if (idxOfFirst == -1) {
    cout << "No POSE_REPORTED objects of the focused class." << endl;
    return;
  }

  //ms->config.currentEEPose = memory.cameraPose;
  ms->config.currentEEPose = memory.aimedPose;
  ms->config.lastPickPose = memory.pickedPose;
  ms->config.lastPrePickPose = memory.aimedPose;
  ms->config.trZ = memory.trZ;

  cout << "deliverObject, " << ms->config.classGraspZsSet.size() << " " << ms->config.classGraspZs.size() << endl;
  if ( (ms->config.classGraspZsSet.size() > ms->config.targetClass) && 
       (ms->config.classGraspZs.size() > ms->config.targetClass) ) {
    if (ms->config.classGraspZsSet[ms->config.targetClass] == 1) {
      ms->config.trZ = ms->config.classGraspZs[ms->config.targetClass];
      cout << "delivering class " << ms->config.classLabels[ms->config.targetClass] << " with classGraspZ " << ms->config.trZ << endl;
    }
  }

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

  ms->pushWord("moveToNextMapPosition");
  ms->pushWord("synchronicServoDoNotTakeClosest"); 
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
  if (ms->config.currentPlaceMode == WAREHOUSE || ms->config.currentPlaceMode == PLACE_REGISTER) {
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
  if (ms->config.currentPlaceMode == WAREHOUSE) {
    ms->config.currentEEPose = ms->config.deliveryPoses[ms->config.currentDeliveryPose];
    ms->config.currentDeliveryPose = (ms->config.currentDeliveryPose + 1) % ms->config.deliveryPoses.size();
  } else if (ms->config.currentPlaceMode == PLACE_REGISTER) {
    double oldz = ms->config.currentEEPose.pz;
    ms->config.currentEEPose = ms->config.placeTarget;
  } else {
    assert(0);
  }
  ms->config.currentEEPose.pz = oldz;
  ms->pushWord("waitUntilAtCurrentPosition");

}
END_WORD
REGISTER_WORD(AssumeDeliveryPose)


}
