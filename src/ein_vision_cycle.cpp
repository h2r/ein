WORD(DeliverObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> word = ms->popWord();
  string className = word->to_string();
  int class_idx = classIdxForName(ms, className);
  if (class_idx != -1) {
    changeTargetClass(ms, class_idx);
    pMachineState->pushWord("deliverTargetObject");
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
  if (ms->config.currentPlaceMode == WAREHOUSE) {
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

WORD(ClearStackIntoMappingPatrol)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->clearStack();
  ms->pushWord("mappingPatrol");
  ms->execute_stack = 1;
}
END_WORD
REGISTER_WORD(ClearStackIntoMappingPatrol)

WORD(ClearStackAcceptFetchCommands)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->clearStack();
  ms->execute_stack = 1;
  ms->config.acceptingFetchCommands = 1;
}
END_WORD
REGISTER_WORD(ClearStackAcceptFetchCommands)

WORD(MappingPatrol)
CODE(196727) // capslock + W
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Mapping patrol" << endl;
  ms->config.bailAfterSynchronic = 1;
  ms->config.bailAfterGradient = 1;
  ms->config.acceptingFetchCommands = 1;

  ms->pushWord("mappingPatrol");
  //ms->pushWord("bringUpAllNonessentialSystems");
  //ms->pushWord("endStackCollapse");
  ms->pushWord("moveToNextMapPosition");
  ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
  //ms->pushWord("setRandomPositionAndOrientationForHeightLearning");
  //ms->pushWord("recordAllBlueBoxes");
  ms->pushWord("filterBoxMemories");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("lockTargetIfBlueBoxes");
  //ms->pushWord("collapseStack");
  ms->pushWord("gradientServoIfBlueBoxes");
  ms->pushWord("mapClosestBlueBox");
  ms->pushWord("goClassifyBlueBoxes"); 
  ms->pushWord("synchronicServo"); 
  ms->pushWord("visionCycleNoClassify");
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("moveToNextMapPosition");
  ms->pushWord("sampleHeight"); 
  ms->pushWord("setBoundingBoxModeToMapping");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("cruisingSpeed");
  //ms->pushWord("shutdownAllNonessentialSystems");
  //ms->pushWord("bringUpAllNonessentialSystems");
  ms->pushWord("setPatrolStateToPatrolling");
}
END_WORD
REGISTER_WORD(MappingPatrol)

WORD(ToggleShouldIDoIK)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.shouldIDoIK = !ms->config.shouldIDoIK;
}
END_WORD
REGISTER_WORD(ToggleShouldIDoIK)

WORD(ToggleShouldIRender)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.shouldIRender = !ms->config.shouldIRender;
}
END_WORD
REGISTER_WORD(ToggleShouldIRender)

WORD(ToggleDrawClearanceMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.drawClearanceMap = !ms->config.drawClearanceMap;
}
END_WORD
REGISTER_WORD(ToggleDrawClearanceMap)

WORD(ToggleDrawIKMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.drawIKMap = !ms->config.drawIKMap;
}
END_WORD
REGISTER_WORD(ToggleDrawIKMap)

WORD(ToggleUseGlow)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.useGlow = !ms->config.useGlow;
}
END_WORD
REGISTER_WORD(ToggleUseGlow)

WORD(ToggleUseFade)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.useFade = !ms->config.useFade;
}
END_WORD
REGISTER_WORD(ToggleUseFade)

WORD(FillClearanceMap)
int pursuitProximity = 5;
int searchProximity = 15;//20;
virtual void execute(std::shared_ptr<MachineState> ms) {
  {
    int proximity = pursuitProximity;
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
	if ( cellIsSearched(ms, i, j) ) {
	  int iIStart = max(0, i-proximity);
	  int iIEnd = min(ms->config.mapWidth-1, i+proximity);
	  int iJStart = max(0, j-proximity);
	  int iJEnd = min(ms->config.mapHeight-1, j+proximity);

	  int reject = 0;
	  for (int iI = iIStart; iI <= iIEnd; iI++) {
	    for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	      if (  ( cellIsSearched(ms, iI, iJ) ) && 
		    ( ms->config.ikMap[iI + ms->config.mapWidth * iJ] != 0 ) &&
		    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity )  ) {
		reject = 1;
	      }
	    }
	  }

	  if (reject) {
	    ms->config.clearanceMap[i + ms->config.mapWidth * j] = 0;
	  } else {
	    ms->config.clearanceMap[i + ms->config.mapWidth * j] = 1;
	  }
	} else {
	  ms->config.clearanceMap[i + ms->config.mapWidth * j] = 0;
	}
      }
    }
  }
  {
    int proximity = searchProximity;
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
	if ( cellIsSearched(ms, i, j) ) {
	  int iIStart = max(0, i-proximity);
	  int iIEnd = min(ms->config.mapWidth-1, i+proximity);
	  int iJStart = max(0, j-proximity);
	  int iJEnd = min(ms->config.mapHeight-1, j+proximity);

	  int reject = 0;
	  for (int iI = iIStart; iI <= iIEnd; iI++) {
	    for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	      if (  ( cellIsSearched(ms, iI, iJ) ) && 
		    ( ms->config.ikMap[iI + ms->config.mapWidth * iJ] != 0 ) &&
		    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity )  ) {
		reject = 1;
	      }
	    }
	  }

	  if (reject) {
	  } else {
	    ms->config.clearanceMap[i + ms->config.mapWidth * j] = 2;
	  }
	} else {
	}
      }
    }
  }
}
END_WORD
REGISTER_WORD(FillClearanceMap)

WORD(SaveIkMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ofstream ofile;
  string fileName = ms->config.data_directory + "/config/" + ms->config.left_or_right_arm + "IkMap";
  cout << "Saving ikMap to " << fileName << endl;
  ofile.open(fileName, ios::trunc | ios::binary);
  ofile.write((char*)ms->config.ikMap, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight);
  ofile.close();
}
END_WORD
REGISTER_WORD(SaveIkMap)

WORD(LoadIkMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // binary seems overkill but consider that this map is
  //  for only one height and is 360kB in binary... how
  //  big would it be in yml, and what if we want another height?
  ifstream ifile;
  string fileName = ms->config.data_directory + "/config/" + ms->config.left_or_right_arm + "IkMap";
  cout << "Loading ikMap from " << fileName << endl;
  ifile.open(fileName, ios::binary);
  ifile.read((char*)ms->config.ikMap, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight);
  ifile.close();
}
END_WORD
REGISTER_WORD(LoadIkMap)

WORD(FillIkMap)
// store these here and create accessors if they need to change
int currentI = 0;
int currentJ = 0;
int cellsPerQuery = 100;
virtual void execute(std::shared_ptr<MachineState> ms) {
  int queries = 0;
  int i=currentI, j=currentJ;
  for (; i < ms->config.mapWidth; i++) {
    if (queries < cellsPerQuery) {
      for (; j < ms->config.mapHeight; j++) {
	if (queries < cellsPerQuery) {
	  if ( cellIsSearched(ms, i, j) ) {
	    double X, Y;
	    mapijToxy(ms, i, j, &X, &Y);

	    eePose nextEEPose = ms->config.currentEEPose;
	    nextEEPose.px = X;
	    nextEEPose.py = Y;

	    baxter_core_msgs::SolvePositionIK thisIkRequest;
	    endEffectorAngularUpdate(&nextEEPose, &ms->config.currentEEDeltaRPY);
	    fillIkRequest(&nextEEPose, &thisIkRequest);

	    bool likelyInCollision = 0;
	    // ATTN 24
	    //int thisIkCallResult = ikClient.call(thisIkRequest);
	    int thisIkCallResult = 0;
	    queryIK(ms, &thisIkCallResult, &thisIkRequest);

	    int ikResultFailed = 1;
	    if (ms->config.currentRobotMode == PHYSICAL) {
	      ikResultFailed = willIkResultFail(ms, thisIkRequest, thisIkCallResult, &likelyInCollision);
	    } else if (ms->config.currentRobotMode == SIMULATED) {
	      ikResultFailed = !positionIsSearched(ms, nextEEPose.px, nextEEPose.py);
	    } else {
              assert(0);
            }

	    int foundGoodPosition = !ikResultFailed;
	    //ms->config.ikMap[i + ms->config.mapWidth * j] = ikResultFailed;
	    //ms->config.ikMap[i + ms->config.mapWidth * j] = 1;
	    //cout << i << " " << j << endl;
	    if (ikResultFailed) {
	      ms->config.ikMap[i + ms->config.mapWidth * j] = 1;
	    } else {
	      if (likelyInCollision) {
		ms->config.ikMap[i + ms->config.mapWidth * j] = 2;
	      } else {
		ms->config.ikMap[i + ms->config.mapWidth * j] = 0;
	      }
	    }
	    queries++;
	  }
	} else {
	  break;
	}
      }
      // reset here so we don't bash the initial restart
      if ( !(j < ms->config.mapHeight) ) {
	j = 0;
      }

      if (queries < cellsPerQuery) {
	continue;
      } else {
	break;
      }
    } else {
      break;
    }
  }

  if (i >= ms->config.mapWidth) {
    i = 0;
  } else {
    ms->pushWord("fillIkMap");
  }
  if (j >= ms->config.mapHeight) {
    j = 0;
  }

  currentI = i;
  currentJ = j;
}
END_WORD
REGISTER_WORD(FillIkMap)

WORD(MoveToNextMapPosition)
int maxNextTries = 100;
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int tries = 0; tries < maxNextTries; tries++) {
    //ros::Time oldestTime = ros::Time::now();
    int oldestI=-1, oldestJ=-1;
    int foundASpot = 0;
    for (int scanRestarter = 0; scanRestarter < 2; scanRestarter++) {
      ros::Time oldestTime = ms->config.lastScanStarted;
      for (int i = 0; i < ms->config.mapWidth; i++) {
	for (int j = 0; j < ms->config.mapHeight; j++) {
	  if (cellIsSearched(ms, i, j) &&
	      (ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime <= oldestTime) &&
	      (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) &&
	      (ms->config.ikMap[i + ms->config.mapWidth * j] == 0) ) {
	    oldestTime = ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
	    oldestI = i;
	    oldestJ = j;
	    foundASpot = 1;
	  }
	}
      }

      if (!foundASpot) {
	cout << "moveToNextMapPosition all spots visited, currentPatrolMode: " << ms->config.currentPatrolMode << endl;
	if (ms->config.currentPatrolMode == LOOP) {
	  ms->config.lastScanStarted = ros::Time::now();
	  cout << "Restarting mappingPatrol." << endl;
	} else if (ms->config.currentPatrolMode == ONCE) {
	  cout << "Patrolled once, idling." << endl;
	  ms->clearStack();
	  ms->execute_stack = 1;
	  ms->config.acceptingFetchCommands = 1;
	  ms->pushWord("idler");
	  return;
	} else {
	  assert(0);
	}
      }
    }

    if (oldestI == -1 || oldestJ == -1) {
      cout << "moveToNextMapPosition failed to find a position. Clearing callstack." << endl;
      ms->clearStack();
      ms->pushCopies("beep", 15); // beep
      return;
    }

    double oldestX, oldestY;
    mapijToxy(ms, oldestI, oldestJ, &oldestX, &oldestY);

    eePose nextEEPose = ms->config.currentEEPose;
    nextEEPose.px = oldestX;
    nextEEPose.py = oldestY;

    baxter_core_msgs::SolvePositionIK thisIkRequest;
    fillIkRequest(&nextEEPose, &thisIkRequest);

    bool likelyInCollision = 0;
    // ATTN 24
    //int thisIkCallResult = ikClient.call(thisIkRequest);
    int thisIkCallResult = 0;
    queryIK(ms, &thisIkCallResult, &thisIkRequest);

    int ikResultFailed = 1;
    if (ms->config.currentRobotMode == PHYSICAL) {
      ikResultFailed = willIkResultFail(ms, thisIkRequest, thisIkCallResult, &likelyInCollision);
    } else if (ms->config.currentRobotMode == SIMULATED) {
      ikResultFailed = !positionIsSearched(ms, nextEEPose.px, nextEEPose.py);
    } else {
      assert(0);
    }

    int foundGoodPosition = !ikResultFailed;

    if (foundGoodPosition) {
      ms->config.currentEEPose.qx = ms->config.straightDown.qx;
      ms->config.currentEEPose.qy = ms->config.straightDown.qy;
      ms->config.currentEEPose.qz = ms->config.straightDown.qz;
      ms->config.currentEEPose.qw = ms->config.straightDown.qw;
      ms->config.currentEEPose.px = oldestX;
      ms->config.currentEEPose.py = oldestY;
      cout << "This pose was accepted by ikClient:" << endl;
      cout << "Next EE Position (x,y,z): " << nextEEPose.px << " " << nextEEPose.py << " " << nextEEPose.pz << endl;
      cout << "Next EE Orientation (x,y,z,w): " << nextEEPose.qx << " " << nextEEPose.qy << " " << nextEEPose.qz << " " << nextEEPose.qw << endl;
      ms->pushWord("waitUntilAtCurrentPosition");
      cout << "moveToNextMapPosition tries foundGoodPosition oldestI oldestJ oldestX oldestY: "  << tries << " " << foundGoodPosition << " "  << oldestI << " " << oldestJ << " " << oldestX << " " << oldestY << endl;
      break;
    } else {
      cout << "moveToNextMapPosition tries foundGoodPosition oldestI oldestJ: "  << tries << " " << foundGoodPosition << " "  << oldestI << " " << oldestJ << " " << oldestX << " " << oldestY << endl;
      cout << "Try number try: " << tries << ", adding point to ikMap oldestI oldestJ ikMap[.]: " << " " << oldestI << " " << oldestJ;
      if (ikResultFailed) {
	ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = 1;
      } else {
	if (likelyInCollision) {
	  ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = 2;
	} else {
	  ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = 0;
	}
      }
      cout << " " << ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] << endl;
    }
  }

  // puts it back at the right height for scanning in
  //  case coming from exotic pose
  ms->pushWord("sampleHeight");
}
END_WORD
REGISTER_WORD(MoveToNextMapPosition)

WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)
virtual void execute(std::shared_ptr<MachineState> ms) {
  object_recognition_msgs::RecognizedObjectArray roaO;
  fillRecognizedObjectArrayFromBlueBoxMemory(ms, &roaO);
  rec_objs_blue_memory.publish(roaO);
}
END_WORD
REGISTER_WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)


WORD(RecordAllBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Recording blue boxes: " << ms->config.bTops.size() << endl;
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    BoxMemory box;
    box.bTop = ms->config.bTops[c];
    box.bBot = ms->config.bBots[c];
    box.cameraPose = ms->config.currentEEPose;
    box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
    box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
    box.centroid.px = (box.top.px + box.bot.px) * 0.5;
    box.centroid.py = (box.top.py + box.bot.py) * 0.5;
    box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
    box.cameraTime = ros::Time::now();
    box.labeledClassIndex = ms->config.bLabels[c];
    ms->config.blueBoxMemories.push_back(box);
  }

}
END_WORD
REGISTER_WORD(RecordAllBlueBoxes)

WORD(VoidCurrentMapRegion)
virtual void execute(std::shared_ptr<MachineState> ms) {
  voidMapRegion(ms, ms->config.currentEEPose.px, ms->config.currentEEPose.py);
  cout << "Voiding the region of the map around ms->config.currentEEPose." << endl;
}
END_WORD
REGISTER_WORD(VoidCurrentMapRegion)

WORD(ClearMapForPatrol)
virtual void execute(std::shared_ptr<MachineState> ms) {
  clearMapForPatrol(ms);
  cout << "Clearing the map for a new patrol." << endl;
}
END_WORD
REGISTER_WORD(ClearMapForPatrol)

WORD(MarkMapAsCompleted)
virtual void execute(std::shared_ptr<MachineState> ms) {
  markMapAsCompleted(ms);
  cout << "Marking whole map as completed." << endl;
}
END_WORD
REGISTER_WORD(MarkMapAsCompleted)

WORD(InitializeMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  initializeMap(ms);
}
END_WORD
REGISTER_WORD(InitializeMap)

WORD(MapEmptySpace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int px = grayTop.x+ms->config.mapGrayBoxPixelSkirt; px < grayBot.x-ms->config.mapGrayBoxPixelSkirt; px++) {
    for (int py = grayTop.y+ms->config.mapGrayBoxPixelSkirt; py < grayBot.y-ms->config.mapGrayBoxPixelSkirt; py++) {

      if (isInGripperMask(ms, px, py)) {
	continue;
      }

      //int blueBoxIdx = blueBoxForPixel(px, py);
      int blueBoxIdx = skirtedBlueBoxForPixel(ms, px, py, ms->config.mapFreeSpacePixelSkirt);

      if (blueBoxIdx == -1) {
        double x, y;
        double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;

        pixelToGlobal(ms, px, py, z, &x, &y);
        int i, j;
        mapxyToij(ms, x, y, &i, &j);

//        if (ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime > mapMemoryTimeout) {
//          ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;
//          ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
//          ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
//          ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
//        }


        ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time::now();
        randomizeNanos(ms, &ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime);
        
        ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = -2;

//	{
//	  ms->config.objectMap[i + ms->config.mapWidth * j].b += (int) ms->config.cam_img.at<cv::Vec3b>(py, px)[0];
//	  ms->config.objectMap[i + ms->config.mapWidth * j].g += (int) ms->config.cam_img.at<cv::Vec3b>(py, px)[1];
//	  ms->config.objectMap[i + ms->config.mapWidth * j].r += (int) ms->config.cam_img.at<cv::Vec3b>(py, px)[2];
//        ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount += 1.0;
//	}
	//const double spaceDecay = 0.996; // 0.7 ^ 0.01
	const double spaceDecay = 0.99821; // 0.7 ^ 0.005
	{
	  ms->config.objectMap[i + ms->config.mapWidth * j].b = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].b) + 
		    (1.0-spaceDecay)*double(ms->config.cam_img.at<cv::Vec3b>(py, px)[0]) );
	  ms->config.objectMap[i + ms->config.mapWidth * j].g = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].g) + 
		    (1.0-spaceDecay)*double(ms->config.cam_img.at<cv::Vec3b>(py, px)[1]) );
	  ms->config.objectMap[i + ms->config.mapWidth * j].r = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].r) + 
		    (1.0-spaceDecay)*double(ms->config.cam_img.at<cv::Vec3b>(py, px)[2]) );
	  ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount) + 
		    (1.0-spaceDecay)*double(1.0) );
	}

      }
    }
  }
}
END_WORD
REGISTER_WORD(MapEmptySpace)





WORD(MapClosestBlueBox)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.pilotClosestBlueBoxNumber == -1) {
    cout << "Not changing because closest bbox is " << ms->config.pilotClosestBlueBoxNumber << endl;
    return;
  }

  int c = ms->config.pilotClosestBlueBoxNumber;
  BoxMemory box;
  box.bTop = ms->config.bTops[c];
  box.bBot = ms->config.bBots[c];
  box.cameraPose = ms->config.currentEEPose;
  box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
  box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
  box.centroid.px = (box.top.px + box.bot.px) * 0.5;
  box.centroid.py = (box.top.py + box.bot.py) * 0.5;
  box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
  box.cameraTime = ros::Time::now();
  box.labeledClassIndex = ms->config.bLabels[c];
  box.lockStatus = CENTROID_LOCK;
  
  int i, j;
  mapxyToij(ms, box.centroid.px, box.centroid.py, &i, &j);

  // this only does the timestamp to avoid obsessive behavior
  mapBox(ms, box);
  
  //if ( !positionIsSearched(box.centroid.px, box.centroid.py) && 
       //!isCellInPursuitZone(i, j) ) 
  //if (!positionIsSearched(box.centroid.px, box.centroid.py)) 
  if ( !positionIsSearched(ms, box.centroid.px, box.centroid.py) || 
       !isBoxMemoryIkPossible(ms, box) ) 
  {
    return;
  } else {
    vector<BoxMemory> newMemories;
    for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
      if (!boxMemoryIntersectCentroid(box, ms->config.blueBoxMemories[i])) {
	newMemories.push_back(ms->config.blueBoxMemories[i]);
      }
    }
    newMemories.push_back(box);
    ms->config.blueBoxMemories = newMemories;
  }
}
END_WORD
REGISTER_WORD(MapClosestBlueBox)


WORD(FilterBoxMemories)
virtual void execute(std::shared_ptr<MachineState> ms) {
  set<int> boxMemoryIndexesToKeep;

  for (int b_i = 0; b_i < ms->config.blueBoxMemories.size(); b_i++) {
    int keep = 0;
    BoxMemory b = ms->config.blueBoxMemories[b_i];
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
        if (boxMemoryIntersectsMapCell(ms, b, i, j)) {
          //if (b.cameraTime.sec > ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.sec) {
          ros::Duration diff = ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime - b.cameraTime;

          if (diff < ros::Duration(2.0)) {
            boxMemoryIndexesToKeep.insert(b_i);
	    keep = 1;
          }
        }
      }
    }
    if (!keep) {
      cout << "filterBoxMemories rejecting box " << b_i << endl;
    }
  }

  vector<BoxMemory> newMemories;

  for (std::set<int>::iterator it=boxMemoryIndexesToKeep.begin(); it!=boxMemoryIndexesToKeep.end(); ++it) {
    newMemories.push_back(ms->config.blueBoxMemories[*it]);
  }
  ms->config.blueBoxMemories = newMemories;
}
END_WORD
REGISTER_WORD(FilterBoxMemories)

WORD(ClearBlueBoxMemories)
CODE(196709) // capslock + E
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Clearing blue box memory: " << ms->config.blueBoxMemories.size() << endl;
  ms->config.blueBoxMemories.resize(0);
}
END_WORD
REGISTER_WORD(ClearBlueBoxMemories)

WORD(VisionCycle)
CODE(131153)  // capslock + q
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("mapEmptySpace");
  ms->pushWord("goClassifyBlueBoxes"); 
  ms->pushWord("goFindBlueBoxes"); 
  ms->pushCopies("density", 1); 
  //ms->pushCopies("resetTemporalMap", 1); 
  //ms->pushCopies("density", 1); 
}
END_WORD
REGISTER_WORD(VisionCycle)

WORD(Density)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("densityA");
  //ms->pushWord("waitUntilImageCallbackReceived");
  //ms->pushCopies("waitUntilImageCallbackReceived", 5);
  ms->pushWord("hover");
}
END_WORD
REGISTER_WORD(Density)

WORD(DensityA)
CODE(131121)     // capslock + 1
virtual void execute(std::shared_ptr<MachineState> ms) {
  substituteLatestImageQuantities(ms);
  goCalculateDensity(ms);
}
END_WORD
REGISTER_WORD(DensityA)

WORD(AccumulatedDensity)
virtual void execute(std::shared_ptr<MachineState> ms) {
  substituteAccumulatedImageQuantities(ms);
  goCalculateDensity(ms);
  renderAccumulatedImageAndDensity(ms);
  //goAccumulateForAerial();
}
END_WORD
REGISTER_WORD(AccumulatedDensity)

WORD(ResetAccumulatedDensity)
virtual void execute(std::shared_ptr<MachineState> ms) {
  resetAccumulatedImageAndMass(ms);
}
END_WORD
REGISTER_WORD(ResetAccumulatedDensity)

WORD(ResetTemporalMap)
CODE(1179737) // capslock + numlock + y
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.temporalDensity != NULL && ms->config.preDensity != NULL) {
    //cout << "ms->config.preDensity<<<<***" << endl;
    Size sz = ms->config.objectViewerImage.size();
    int imW = sz.width;
    int imH = sz.height;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
        ms->config.temporalDensity[y*imW+x] = ms->config.preDensity[y*imW+x];
      }
    }
  }
}
END_WORD
REGISTER_WORD(ResetTemporalMap)

WORD(GoFindBlueBoxes)
CODE(131122) // capslock + 2
virtual void execute(std::shared_ptr<MachineState> ms) {
  goFindBlueBoxes(ms);
}
END_WORD
REGISTER_WORD(GoFindBlueBoxes)

WORD(GoClassifyBlueBoxes)
CODE(131123) // capslock + 3
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.lastVisionCycle = ros::Time::now();
  ms->config.oscilStart = ros::Time::now();
  goClassifyBlueBoxes(ms);
}
END_WORD
REGISTER_WORD(GoClassifyBlueBoxes)

WORD(AssumeFacePose)
virtual void execute(std::shared_ptr<MachineState> ms) {

  //eePose facePose = {.px = 1.07226, .py = 0.564963, .pz = 0.287997,
  //                   .qx = -0.234838, .qy = 0.75433, .qz = 0.106368, .qw = 0.603757};      
  eePose facePose = {.px = 0.85838, .py = 0.56957, .pz = 0.163187,
                     .qx = -0.153116, .qy = 0.717486, .qz = 0.0830483, .qw = 0.674442};

  ms->config.currentEEPose = facePose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeFacePose)


/*
WORD(DetectFaces)
virtual void execute(std::shared_ptr<MachineState> ms) {
  vector<Rect> faces = faceDetectAndDisplay(faceViewName, ms->config.faceViewImage);
}
END_WORD
REGISTER_WORD(DetectFaces)


WORD(FaceServo)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.faceServoIterations = 0;
  ms->pushWord("faceServoA");
}
END_WORD
REGISTER_WORD(FaceServo)

WORD(FaceServoA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.faceServoIterations++;
  ms->pushWord("endStackCollapseNoop");
  ms->pushWord("faceServoB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 1);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("waitUntilAtCurrentPosition"); 
}
END_WORD
REGISTER_WORD(FaceServoA)

WORD(FaceServoB)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (ms->config.faceServoIterations > ms->config.faceServoTimeout) {
    cout << "faceServo timed out, continuing..." << endl;
    return;
  }
  vector<Rect> faces = faceDetectAndDisplay(faceViewName, ms->config.faceViewImage);
  faceServo(ms, faces);
}
END_WORD
REGISTER_WORD(FaceServoB)

*/
