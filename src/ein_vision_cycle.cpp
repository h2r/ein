
#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "opencv2/contrib/contrib.hpp"

namespace ein_words {

WORD(ClearStackIntoMappingPatrol)
virtual void execute(MachineState * ms) {
  ms->clearStack();
  ms->pushWord("mappingPatrol");
  ms->execute_stack = 1;
}
END_WORD
REGISTER_WORD(ClearStackIntoMappingPatrol)



WORD(MapServo)
virtual void execute(MachineState * ms) {
  if (ms->config.currentMapServoMode == HISTOGRAM_CLASSIFY) {
    ms->pushWord("gradientServoIfBlueBoxes");
    ms->pushWord("mapClosestBlueBox");
    ms->pushWord("mapEmptySpace");
    ms->pushWord("histogramDetectionIfBlueBoxes"); 
    ms->pushWord("synchronicServo"); 
    ms->pushWord("synchronicServoTakeClosest");
  } else if (ms->config.currentMapServoMode == ONCE_CLASSIFY) {
    ms->pushWord("gradientServoIfBlueBoxes");
    ms->pushWord("mapClosestBlueBox");
    ms->pushWord("mapEmptySpace");
    ms->pushWord("goClassifyBlueBoxes"); 
    ms->pushWord("synchronicServo"); 
    ms->pushWord("synchronicServoTakeClosest");
  } else if (ms->config.currentMapServoMode == FIXED_CLASS_ACCUMULATED) {
    ms->pushWord("gradientServoIfBlueBoxes");
    ms->pushWord("mapClosestBlueBox");
    ms->pushWord("mapEmptySpace");
    ms->pushWord("replaceBlueBoxesWithFocusedClass"); 
    ms->pushWord("synchronicServo"); 
    ms->pushWord("synchronicServoTakeClosest");
  } else if (ms->config.currentMapServoMode == FIXED_CLASS_CONTINUOUS) {
    ms->pushWord("continuousServo");
    //ms->pushWord("continuousServo");
    //ms->pushWord("continuousServo");
    //ms->pushWord("continuousServo");
    //ms->pushWord("continuousServo");
    //ms->pushWord("continuousServo");
    ms->pushWord("mapClosestBlueBox");
    ms->pushWord("mapEmptySpace");
    ms->pushWord("replaceBlueBoxesWithFocusedClass"); 
    ms->pushWord("synchronicServo"); 
    ms->pushWord("synchronicServoTakeClosest");
  } else if (ms->config.currentMapServoMode == FIXED_CLASS_ACCUMULATED_NOSYN) {
    ms->pushWord("gradientServoIfBlueBoxes");
    ms->pushWord("mapClosestBlueBox");
    ms->pushWord("mapEmptySpace");
    ms->pushWord("replaceBlueBoxesWithFocusedClass"); 
    ms->pushWord("visionCycleNoClassify"); 
  } else if (ms->config.currentMapServoMode == FIXED_CLASS_CONTINUOUS_NOSYN) {
    ms->pushCopies("continuousServo", 10);
    ms->pushWord("mapClosestBlueBox");
    ms->pushWord("mapEmptySpace");
    ms->pushWord("replaceBlueBoxesWithFocusedClass"); 
    ms->pushWord("visionCycleNoClassify"); 
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(MapServo)

WORD(MapLocal)
virtual void execute(MachineState * ms) {
  ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
  ms->pushWord("filterBoxMemories");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("lockTargetIfBlueBoxes");

  ms->pushWord("mapServo"); 

  ms->pushWord("visionCycle"); 
  ms->pushWord("cruisingSpeed"); 
}
END_WORD
REGISTER_WORD(MapLocal)

WORD(MappingPatrol)
CODE(196727) // capslock + W
virtual void execute(MachineState * ms) {
  cout << "mappingPatrol" << endl;
  ms->pushWord("moveToNextMapPosition");
}
END_WORD
REGISTER_WORD(MappingPatrol)

WORD(SetMapServoMode)
virtual void execute(MachineState * ms) {
  int modeGot = 0;
  GET_ARG(ms, IntegerWord, modeGot);
  cout << "setMapServoMode was: " << ms->config.currentMapServoMode << " setting " << modeGot << endl;
  ms->config.currentMapServoMode = (mapServoMode)modeGot;
}
END_WORD
REGISTER_WORD(SetMapServoMode)

WORD(MappingPatrolA)
virtual void execute(MachineState * ms) {
  cout << "mappingPatrolA" << endl;
  ms->config.bailAfterSynchronic = 1;
  ms->config.bailAfterGradient = 1;

  ms->pushWord("moveToNextMapPosition");

  if (ms->config.mapAutoPick) {
    ms->pushWord("pickAllBlueBoxes");
  } else {
  }
 
  ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
  //ms->pushWord("setRandomPositionAndOrientationForHeightLearning");
  //ms->pushWord("recordAllBlueBoxes");
  ms->pushWord("filterBoxMemories");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("lockTargetIfBlueBoxes");
  //ms->pushWord("collapseStack");

  ms->pushWord("mapServo");

  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("sampleHeight"); 
  ms->pushWord("setBoundingBoxModeToMapping");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("cruisingSpeed");
  //ms->pushWord("shutdownAllNonessentialSystems");
  //ms->pushWord("bringUpAllNonessentialSystems");
  ms->pushWord("setPatrolStateToPatrolling");
}
END_WORD
REGISTER_WORD(MappingPatrolA)

WORD(ToggleShouldIDoIK)
virtual void execute(MachineState * ms) {
  ms->config.shouldIDoIK = !ms->config.shouldIDoIK;
}
END_WORD
REGISTER_WORD(ToggleShouldIDoIK)

WORD(ToggleShouldIRender)
virtual void execute(MachineState * ms) {
  ms->config.shouldIRender = !ms->config.shouldIRender;
}
END_WORD
REGISTER_WORD(ToggleShouldIRender)

WORD(ToggleDrawClearanceMap)
virtual void execute(MachineState * ms) {
  ms->config.drawClearanceMap = !ms->config.drawClearanceMap;
}
END_WORD
REGISTER_WORD(ToggleDrawClearanceMap)

WORD(ToggleDrawIKMap)
virtual void execute(MachineState * ms) {
  ms->config.drawIKMap = !ms->config.drawIKMap;
}
END_WORD
REGISTER_WORD(ToggleDrawIKMap)

WORD(ToggleUseGlow)
virtual void execute(MachineState * ms) {
  ms->config.useGlow = !ms->config.useGlow;
}
END_WORD
REGISTER_WORD(ToggleUseGlow)

WORD(ToggleUseFade)
virtual void execute(MachineState * ms) {
  ms->config.useFade = !ms->config.useFade;
}
END_WORD
REGISTER_WORD(ToggleUseFade)

CONFIG_GETTER_INT(PursuitProximity, ms->config.pursuitProximity)
CONFIG_SETTER_INT(SetPursuitProximity, ms->config.pursuitProximity)

CONFIG_GETTER_INT(SearchProximity, ms->config.searchProximity)
CONFIG_SETTER_INT(SetSearchProximity, ms->config.searchProximity)


WORD(FillClearanceMap)
virtual void execute(MachineState * ms) {
  {
    int proximity = ms->config.pursuitProximity;
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
	    if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
	      int iIStart = max(0, i-proximity);
	      int iIEnd = min(ms->config.mapWidth-1, i+proximity);
	      int iJStart = max(0, j-proximity);
	      int iJEnd = min(ms->config.mapHeight-1, j+proximity);

	      int reject = 0;
	      for (int iI = iIStart; iI <= iIEnd; iI++) {
	        for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	          if (  ( !cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, iI, iJ) ) || 
	    	    (( ms->config.ikMap[iI + ms->config.mapWidth * iJ] != 0 ) &&
	    	    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity ))  ) {
	    	    reject = 1;
	          } else {
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
    int proximity = ms->config.searchProximity;
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
	    if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
	      int iIStart = max(0, i-proximity);
	      int iIEnd = min(ms->config.mapWidth-1, i+proximity);
	      int iJStart = max(0, j-proximity);
	      int iJEnd = min(ms->config.mapHeight-1, j+proximity);

	      int reject = 0;
	      for (int iI = iIStart; iI <= iIEnd; iI++) {
	        for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	          if (  ( !cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, iI, iJ) ) || 
	    	    (( ms->config.ikMap[iI + ms->config.mapWidth * iJ] != 0 ) &&
	    	    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity ))  ) {
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

WORD(PointToClearanceMap)
virtual void execute(MachineState * ms) {

  int currentI, currentJ;
  mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, ms->config.currentEEPose.px, ms->config.currentEEPose.py, &currentI, &currentJ);

  int minI, minJ;
  double minDistanceToGreen = INFINITY;

  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      if (ms->config.clearanceMap[i + ms->config.mapWidth * j] == CLEARANCE_SEARCH) {
	double thisDistSq = ((i-currentI)*(i-currentI)+(j-currentJ)*(j-currentJ));
	if ( thisDistSq < minDistanceToGreen ) {
	  minDistanceToGreen = thisDistSq;
	  minI = i;
	  minJ = j; 
	} else {
	}
      } else {
      }
    }
  }

  double minX, minY;
  mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, minI, minJ, &minX, &minY);
  
  double crane1I = 1;
  double crane1J = 0;
  double craneAngle = vectorArcTan(ms->p, crane1I, crane1J);

  double maxDirI = (minI-currentI);
  double maxDirJ = (minJ-currentJ);
  double maxDirAngle = vectorArcTan(ms->p, maxDirI, maxDirJ);

  ms->config.currentEEPose.px = minX;
  ms->config.currentEEPose.py = minY;

  // XXX NOT DONE
  ms->config.currentEEDeltaRPY = eePose::zero();
  ms->config.currentEEDeltaRPY.pz = ( maxDirAngle - craneAngle );
  ms->config.currentEEPose.qx = 0.0;
  ms->config.currentEEPose.qy = 1.0;
  ms->config.currentEEPose.qz = 0.0;
  ms->config.currentEEPose.qw = 0.0;
  endEffectorAngularUpdate( &ms->config.currentEEPose, &ms->config.currentEEDeltaRPY );
}
END_WORD
REGISTER_WORD(PointToClearanceMap)

WORD(SaveIkMapAtHeight)
virtual void execute(MachineState * ms) {
  ofstream ofile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMapAtHeight";
  cout << "Saving ikMapAtHeight to " << fileName << endl;
  ofile.open(fileName, ios::trunc | ios::binary);
  ofile.write((char*)ms->config.ikMapAtHeight, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight*ms->config.numIkMapHeights);
  ofile.close();
}
END_WORD
REGISTER_WORD(SaveIkMapAtHeight)

WORD(LoadIkMapAtHeight)
virtual void execute(MachineState * ms) {
  // binary seems overkill but consider that this map is
  //  for only one height and is 360kB in binary... how
  //  big would it be in yml, and what if we want another height?
  ifstream ifile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMapAtHeight";
  cout << "Loading ikMapAtHeight from " << fileName << endl;
  ifile.open(fileName, ios::binary);
  ifile.read((char*)ms->config.ikMapAtHeight, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight*ms->config.numIkMapHeights);
  ifile.close();
}
END_WORD
REGISTER_WORD(LoadIkMapAtHeight)


WORD(SaveIkMap)
virtual void execute(MachineState * ms) {
  ofstream ofile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMap";
  cout << "Saving ikMap to " << fileName << endl;
  ofile.open(fileName, ios::trunc | ios::binary);
  ofile.write((char*)ms->config.ikMap, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight);
  ofile.close();
}
END_WORD
REGISTER_WORD(SaveIkMap)

WORD(LoadIkMap)
virtual void execute(MachineState * ms) {
  // binary seems overkill but consider that this map is
  //  for only one height and is 360kB in binary... how
  //  big would it be in yml, and what if we want another height?
  ifstream ifile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMap";
  cout << "Loading ikMap from " << fileName << endl;
  ifile.open(fileName, ios::binary);
  ifile.read((char*)ms->config.ikMap, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight);
  ifile.close();
}
END_WORD
REGISTER_WORD(LoadIkMap)




WORD(FillIkMapAtHeights)
virtual string description() {
  return "Fill the IK map at different heights.";
}
virtual void execute(MachineState * ms) {
  double ikStep = (ms->config.ikMapEndHeight - ms->config.ikMapStartHeight) / (ms->config.numIkMapHeights - 1);

  for (int i = 0; i < ms->config.numIkMapHeights; i++) {
    
    double height = ms->config.ikMapStartHeight + ikStep * i;
    stringstream program;
    program << "0 0 " << height << " fillIkMap " << i << " copyIkMapToHeightIdx";
    ms->evaluateProgram(program.str());
  }
}
END_WORD
REGISTER_WORD(FillIkMapAtHeights)

WORD(FillIkMapFromCachedHeights)
virtual string description() {
  return "Fill the IK map by taking the and of the result at all the different heights.";
}
virtual void execute(MachineState * ms) {
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      bool result = IK_GOOD;
      for (int heightIdx = 0; heightIdx < ms->config.numIkMapHeights; heightIdx++) {
	if (ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] == IK_FAILED ||
	    ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] == IK_LIKELY_IN_COLLISION) {
	  result = IK_FAILED;
	  break;
	}
      }
      ms->config.ikMap[i + ms->config.mapWidth * j] = result;
    }
  }
}

END_WORD
REGISTER_WORD(FillIkMapFromCachedHeights)


WORD(FillIkMapFromCachedHeightIdx)
virtual string description() {
  return "Fill the IK map by taking the height idx from the cache.";
}
virtual void execute(MachineState * ms) {
  int heightIdx;
  GET_INT_ARG(ms, heightIdx);
  if (heightIdx >= ms->config.numIkMapHeights) {
    cout << "Ooops, height out of bounds. " << heightIdx << endl;
    ms->pushWord("pauseStackExecution");   
    return;
  }
  
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.ikMap[i + ms->config.mapWidth * j] = ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx];
    }
  }
}

END_WORD
REGISTER_WORD(FillIkMapFromCachedHeightIdx)




WORD(CopyIkMapToHeightIdx)
virtual string description() {
  return "Copy the ik map to the height index.";
}
virtual void execute(MachineState * ms) {
  int heightIdx;
  GET_INT_ARG(ms, heightIdx);
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] = ms->config.ikMap[i + ms->config.mapWidth * j];
    }
  }
}
END_WORD
REGISTER_WORD(CopyIkMapToHeightIdx)


WORD(ClearIkMap)
virtual string description() {
  return "Reset the IK Map so that every cell is good.";
}
virtual void execute(MachineState * ms) {
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.ikMap[i + ms->config.mapWidth * j] = IK_GOOD;
    }
  }
}
END_WORD
REGISTER_WORD(ClearIkMap)


WORD(FillIkMap)

virtual string description() {
  return "Fill the IK map for the current range starting at the i and j and height on the stack.";
}
virtual void execute(MachineState * ms) {
  int cellsPerQuery = 200;

  int currentI, currentJ;
  double height;
  GET_NUMERIC_ARG(ms, height);
  GET_INT_ARG(ms, currentJ);
  GET_INT_ARG(ms, currentI);



  int queries = 0;
  int i=currentI, j=currentJ;
  vector<eePose> poses;
  vector<tuple<int, int> > mapIndexes;

  for (; i < ms->config.mapWidth; i++) {
    if (queries < cellsPerQuery) {
      for (; j < ms->config.mapHeight; j++) {
	if (queries < cellsPerQuery) {
	  if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
	    double X, Y;
	    mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j, &X, &Y);

	    eePose nextEEPose = ms->config.straightDown;
	    nextEEPose.px = X;
	    nextEEPose.py = Y;
	    nextEEPose.pz = height;
            poses.push_back(nextEEPose);
            mapIndexes.push_back(make_tuple(i, j));

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
  //cout << "Sending " << poses.size() << " poses to the IK Service..." << endl;
  vector<ikMapState> results = ikAtPoses(ms, poses);
  //cout << "Received " << results.size() << " results from the IK Service." << endl;
  //assert(results.size() == poses.size());
  //cout << "Sending " << poses.size() << " poses to the IK Service... separately" << endl;
  for (int k = 0; k < mapIndexes.size(); k++) {
    int ci = std::get<0>(mapIndexes[k]);
    int cj = std::get<1>(mapIndexes[k]);
    //ikMapState result = ikAtPose(ms, poses[k]);
    ms->config.ikMap[ci + ms->config.mapWidth * cj] = results[k];
    //ms->config.ikMap[ci + ms->config.mapWidth * cj] = result;
  }
  //cout << "Done." << endl;

  if (j >= ms->config.mapHeight) {
    j = 0;
  }

  if (i >= ms->config.mapWidth) {
    i = 0;
  } else {

    ms->pushWord("fillIkMap");
    ms->pushWord(make_shared<DoubleWord>(height));  
    ms->pushWord(make_shared<IntegerWord>(j));  
    ms->pushWord(make_shared<IntegerWord>(i));  
  }

  ms->config.endThisStackCollapse = 1;


}
END_WORD
REGISTER_WORD(FillIkMap)

WORD(FillIkMapAtCurrentHeight)

virtual string description() {
  return "Fill the IK map using data at the current EE height.  We run at height 2 usually.";
}

virtual void execute(MachineState * ms) {
  ms->evaluateProgram("0 0 currentPose eePosePZ fillIkMap");
}
END_WORD
REGISTER_WORD(FillIkMapAtCurrentHeight)

WORD(MoveToNextMapPosition)
virtual void execute(MachineState * ms) {
  int p_maxNextTries = 100;
  for (int tries = 0; tries < p_maxNextTries; tries++) {
    //ros::Time oldestTime = ros::Time::now();
    int oldestI=-1, oldestJ=-1;
    int foundASpot = 0;
    for (int scanRestarter = 0; scanRestarter < 2; scanRestarter++) {
      ros::Time oldestTime = ms->config.lastScanStarted;
      for (int i = 0; i < ms->config.mapWidth; i++) {
	    for (int j = 0; j < ms->config.mapHeight; j++) {
	      if (cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) &&
	          (ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime <= oldestTime) &&
	          (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) &&
	          (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_GOOD) ) {
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
	  ms->execute_stack = 1;
	  ms->pushWord("idler");
	  return;
	} else {
	  assert(0);
	}
      }
    }

    if (oldestI == -1 || oldestJ == -1) {
      cout << "moveToNextMapPosition: failed to find a position but is looping, logical error. Clearing callstack." << endl;
      ms->clearStack();
      ms->pushCopies("beep", 15); // beep
      return;
    }

    double oldestX, oldestY;
    mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, oldestI, oldestJ, &oldestX, &oldestY);

    eePose nextEEPose = ms->config.currentEEPose;
    nextEEPose.px = oldestX;
    nextEEPose.py = oldestY;

    baxter_core_msgs::SolvePositionIK thisIkRequest;
    fillIkRequest(nextEEPose, &thisIkRequest);

    bool likelyInCollision = 0;
    // ATTN 24
    //int thisIkCallResult = ms->config.ikClient.call(thisIkRequest);
    int thisIkCallResult = 0;
    queryIK(ms, &thisIkCallResult, &thisIkRequest);

    int ikResultFailed = 1;
    if (ms->config.currentRobotMode == PHYSICAL) {
      ikResultFailed = willIkResultFail(ms, thisIkRequest, thisIkCallResult, &likelyInCollision, 0);
    } else if (ms->config.currentRobotMode == SIMULATED) {
      ikResultFailed = !positionIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, nextEEPose.px, nextEEPose.py);
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
	ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = IK_FAILED;
      } else {
	if (likelyInCollision) {
	  ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = IK_LIKELY_IN_COLLISION;
	} else {
	  ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = IK_GOOD;
	}
      }
      cout << " " << ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] << endl;
    }
  }

  // puts it back at the right height for scanning in
  //  case coming from exotic pose
  ms->pushWord("mappingPatrolA");
  ms->pushWord("sampleHeight");
}
END_WORD
REGISTER_WORD(MoveToNextMapPosition)

WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)
virtual void execute(MachineState * ms) {
  object_recognition_msgs::RecognizedObjectArray roaO;
  fillRecognizedObjectArrayFromBlueBoxMemory(ms, &roaO);
  ms->config.rec_objs_blue_memory.publish(roaO);
}
END_WORD
REGISTER_WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)


WORD(RecordAllBlueBoxes)
virtual void execute(MachineState * ms) {
  cout << "Recording blue boxes: " << ms->config.bTops.size() << endl;
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    BoxMemory box;
    box.bTop = ms->config.bTops[c];
    box.bBot = ms->config.bBots[c];
    box.cameraPose = ms->config.currentEEPose;
    box.top = pixelToGlobalEEPose(ms->p, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
    box.bot = pixelToGlobalEEPose(ms->p, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
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
virtual void execute(MachineState * ms) {
  voidMapRegion(ms, ms->config.currentEEPose.px, ms->config.currentEEPose.py);
  cout << "Voiding the region of the map around ms->config.currentEEPose." << endl;
}
END_WORD
REGISTER_WORD(VoidCurrentMapRegion)

WORD(ClearMapForPatrol)
virtual void execute(MachineState * ms) {
  clearMapForPatrol(ms);
  cout << "Clearing the map for a new patrol." << endl;
}
END_WORD
REGISTER_WORD(ClearMapForPatrol)

WORD(MarkMapAsCompleted)
virtual void execute(MachineState * ms) {
  markMapAsCompleted(ms);
  cout << "Marking whole map as completed." << endl;
}
END_WORD
REGISTER_WORD(MarkMapAsCompleted)

WORD(InitializeMap)
virtual void execute(MachineState * ms) {
  initializeMap(ms);
}
END_WORD
REGISTER_WORD(InitializeMap)

CONFIG_GETTER_INT(MapGrayBoxPixelSkirtCols, ms->config.mapGrayBoxPixelSkirtCols)
CONFIG_SETTER_INT(SetMapGrayBoxPixelSkirtCols, ms->config.mapGrayBoxPixelSkirtCols)

CONFIG_GETTER_INT(MapGrayBoxPixelSkirtRows, ms->config.mapGrayBoxPixelSkirtRows)
CONFIG_SETTER_INT(SetMapGrayBoxPixelSkirtRows, ms->config.mapGrayBoxPixelSkirtRows)

CONFIG_GETTER_INT(MapGrayBoxPixelWaistCols, ms->config.mapGrayBoxPixelWaistCols)
CONFIG_SETTER_INT(SetMapGrayBoxPixelWaistCols, ms->config.mapGrayBoxPixelWaistCols)

CONFIG_GETTER_INT(MapGrayBoxPixelWaistRows, ms->config.mapGrayBoxPixelWaistRows)
CONFIG_SETTER_INT(SetMapGrayBoxPixelWaistRows, ms->config.mapGrayBoxPixelWaistRows)

CONFIG_GETTER_INT(MapFreeSpacePixelSkirt, ms->config.mapFreeSpacePixelSkirt)
CONFIG_SETTER_INT(SetMapFreeSpacePixelSkirt, ms->config.mapFreeSpacePixelSkirt)

WORD(MapEmptySpace)
virtual void execute(MachineState * ms) {
  for (int px = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; px < ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; px++) {
    for (int py = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; py < ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; py++) {
      
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      
      //int blueBoxIdx = blueBoxForPixel(px, py);
      int blueBoxIdx = skirtedBlueBoxForPixel(ms, px, py, ms->config.mapFreeSpacePixelSkirt);
      
      if (blueBoxIdx == -1) {
        double x, y;
        double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;

        pixelToGlobal(ms->p, px, py, z, &x, &y);
        int i, j;
        mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, x, y, &i, &j);
	
	//        if (ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime > mapMemoryTimeout) {
	//          ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;
	//          ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
	//          ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
	//          ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
	//        }
	
	// if we are mapping too close to the table these can exceed bounds
	if ( (i < 0) || (j < 0) || (i >= ms->config.mapWidth) || (j >= ms->config.mapHeight) ) {
	  continue;
	}


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
virtual void execute(MachineState * ms) {

  if (ms->config.pilotClosestBlueBoxNumber == -1) {
    cout << "Not mapping closest bbox since it is " << ms->config.pilotClosestBlueBoxNumber << endl;
    return;
  } else {
    cout << "Mapping closest blue box, number " << ms->config.pilotClosestBlueBoxNumber << endl;
  }

  int c = ms->config.pilotClosestBlueBoxNumber;
  BoxMemory box;
  box.bTop = ms->config.bTops[c];
  box.bBot = ms->config.bBots[c];
  box.cameraPose = ms->config.currentEEPose;
  box.top = pixelToGlobalEEPose(ms->p, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
  box.bot = pixelToGlobalEEPose(ms->p, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
  box.centroid.px = (box.top.px + box.bot.px) * 0.5;
  box.centroid.py = (box.top.py + box.bot.py) * 0.5;
  box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
  box.cameraTime = ros::Time::now();
  box.labeledClassIndex = ms->config.bLabels[c];
  box.lockStatus = CENTROID_LOCK;
  
  int i, j;
  mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, box.centroid.px, box.centroid.py, &i, &j);

  // this only does the timestamp to avoid obsessive behavior
  mapBox(ms, box);
  
  //if ( !positionIsSearched(box.centroid.px, box.centroid.py) && 
       //!isCellInPursuitZone(i, j) ) 
  //if (!positionIsSearched(box.centroid.px, box.centroid.py)) 
  if ( !positionIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, box.centroid.px, box.centroid.py) || 
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  cout << "Clearing blue box memory: " << ms->config.blueBoxMemories.size() << endl;
  ms->config.blueBoxMemories.resize(0);
}
END_WORD
REGISTER_WORD(ClearBlueBoxMemories)

WORD(VisionCycle)
CODE(131153)  // capslock + q
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  ms->pushWord("densityA");
  //ms->pushWord("waitUntilImageCallbackReceived");
  //ms->pushCopies("waitUntilImageCallbackReceived", 5);
  ms->pushWord("hover");
}
END_WORD
REGISTER_WORD(Density)

WORD(DensityA)
CODE(131121)     // capslock + 1
virtual void execute(MachineState * ms) {
  substituteLatestImageQuantities(ms);
  goCalculateDensity(ms);
}
END_WORD
REGISTER_WORD(DensityA)

WORD(StreamedDensity)
virtual void execute(MachineState * ms) {
  substituteStreamImageQuantities(ms);
  goCalculateDensity(ms);
  renderAccumulatedImageAndDensity(ms);
}
END_WORD
REGISTER_WORD(StreamedDensity)

WORD(StreamedAccumulatedDensity)
virtual void execute(MachineState * ms) {
  substituteStreamAccumulatedImageQuantities(ms);
  goCalculateDensity(ms);
  renderAccumulatedImageAndDensity(ms);
}
END_WORD
REGISTER_WORD(StreamedAccumulatedDensity)

WORD(AccumulatedDensity)
virtual void execute(MachineState * ms) {
  substituteAccumulatedImageQuantities(ms);
  goCalculateDensity(ms);
  renderAccumulatedImageAndDensity(ms);
}
END_WORD
REGISTER_WORD(AccumulatedDensity)

WORD(ResetAccumulatedDensity)
virtual void execute(MachineState * ms) {
  resetAccumulatedImageAndMass(ms);
}
END_WORD
REGISTER_WORD(ResetAccumulatedDensity)

WORD(ResetTemporalMap)
CODE(1179737) // capslock + numlock + y
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  goFindBlueBoxes(ms);
}
END_WORD
REGISTER_WORD(GoFindBlueBoxes)

WORD(GoClassifyBlueBoxes)
CODE(131123) // capslock + 3
virtual void execute(MachineState * ms) {
  ms->config.lastVisionCycle = ros::Time::now();
  ms->config.oscilStart = ros::Time::now();
  goClassifyBlueBoxes(ms);
}
END_WORD
REGISTER_WORD(GoClassifyBlueBoxes)

WORD(AssumeFacePose)
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {
  vector<Rect> faces = faceDetectAndDisplay(faceViewName, ms->config.faceViewImage);
}
END_WORD
REGISTER_WORD(DetectFaces)


WORD(FaceServo)
virtual void execute(MachineState * ms)
{
  ms->config.faceServoIterations = 0;
  ms->pushWord("faceServoA");
}
END_WORD
REGISTER_WORD(FaceServo)

WORD(FaceServoA)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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

WORD(StereoPair)
virtual void execute(MachineState * ms) {
  // display the result 
  ms->pushWord("stereoDisplay");
  // calculate disparity and depth
  ms->pushWord("stereoCalculate");
  // cache a second accumulated image
  ms->pushWord("stereoPairCache2");
  ms->pushWord("stereoPrep");
  // move a tad
  ms->pushWord("localYUp");
  // cache an accumulated image
  ms->pushWord("stereoPairCache1");
  ms->pushWord("stereoPrep");
  ms->pushWord("setGridSizeCoarse");
}
END_WORD
REGISTER_WORD(StereoPair)

WORD(StereoPrep)
virtual void execute(MachineState * ms) {
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
}
END_WORD
REGISTER_WORD(StereoPrep)

WORD(StereoPairCache1)
virtual void execute(MachineState * ms) {
  ms->config.stereoImage1 = ms->config.accumulatedImage.clone();
  Size sz = ms->config.stereoImage1.size();
  int imW = sz.width;
  int imH = sz.height;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      assert(denom > 0);
      ms->config.stereoImage1.at<Vec3d>(y,x)[0] = ms->config.stereoImage1.at<Vec3d>(y,x)[0] / denom;
      ms->config.stereoImage1.at<Vec3d>(y,x)[1] = ms->config.stereoImage1.at<Vec3d>(y,x)[1] / denom;
      ms->config.stereoImage1.at<Vec3d>(y,x)[2] = ms->config.stereoImage1.at<Vec3d>(y,x)[2] / denom;
    }
  }
}
END_WORD
REGISTER_WORD(StereoPairCache1)

WORD(StereoPairCache2)
virtual void execute(MachineState * ms) {
  ms->config.stereoImage2 = ms->config.accumulatedImage.clone();
  Size sz = ms->config.stereoImage2.size();
  int imW = sz.width;
  int imH = sz.height;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      assert(denom > 0);
      ms->config.stereoImage2.at<Vec3d>(y,x)[0] = ms->config.stereoImage2.at<Vec3d>(y,x)[0] / denom;
      ms->config.stereoImage2.at<Vec3d>(y,x)[1] = ms->config.stereoImage2.at<Vec3d>(y,x)[1] / denom;
      ms->config.stereoImage2.at<Vec3d>(y,x)[2] = ms->config.stereoImage2.at<Vec3d>(y,x)[2] / denom;
    }
  }
}
END_WORD
REGISTER_WORD(StereoPairCache2)

WORD(StereoCalculate)
virtual void execute(MachineState * ms) {
  // simplifying assumption that the movement between the two views
  //  was along the x axis

  // this can be made to run in time independent of patch size using
  //  running sums over (Im1-Im2_shift)^2
  // XXX should try this in RGB distance but also YCbCr distance
  Size sz = ms->config.stereoImage1.size();
  int imW = sz.width;
  int imH = sz.height;

  // instead of guarding some places we should be calling create(), which checks
  //  for existence.
  ms->config.stereoDisparity.create(sz, CV_64F);
  ms->config.stereoDepth.create(sz, CV_64F);

  ms->config.stereoDisparity = 0.0;
  ms->config.stereoDepth = double(VERYBIGNUMBER);

  // patchWidth must be odd
  int param_patchWidth = 21;
  assert((param_patchWidth % 2) == 1);
  int param_patchHalfWidth = (param_patchWidth-1)/2;
  int disparityMax = ms->config.stereoMaxDisparity;

  Mat sIm1 = ms->config.stereoImage1;
  Mat sIm2 = ms->config.stereoImage2;

  Mat minDifferenceValues(sz, CV_64F);
  minDifferenceValues = VERYBIGNUMBER;

/*
  for (int d = 0; d < disparityMax; d++) {
    Mat shiftedDifference(sz, CV_64F);
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if (x < d) {
	  shiftedDifference.at<double>(y,x) = VERYBIGNUMBER;
	} else {
	  shiftedDifference.at<double>(y,x) = 
	    pow(sIm1.at<Vec3d>(y,x)[0] - sIm2.at<Vec3d>(y,x-d)[0], 2.0) +  
	    pow(sIm1.at<Vec3d>(y,x)[1] - sIm2.at<Vec3d>(y,x-d)[1], 2.0) +  
	    pow(sIm1.at<Vec3d>(y,x)[2] - sIm2.at<Vec3d>(y,x-d)[2], 2.0); 
	}
      }
    }

    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if ( (x < param_patchHalfWidth) || (x > imW - 1 - param_patchHalfWidth) ||
	     (y < param_patchHalfWidth) || (y > imH - 1 - param_patchHalfWidth) ) {
	  continue;
	}

	double thisSSD = 0.0;
	for (int px = -param_patchHalfWidth; px <= param_patchHalfWidth; px++) {
	  for (int py = -param_patchHalfWidth; py <= param_patchHalfWidth; py++) {
	    thisSSD = thisSSD + shiftedDifference.at<double>(y+py, x+px);
	  }
	}
	if (thisSSD < minDifferenceValues.at<double>(y,x)) {
	  ms->config.stereoDisparity.at<double>(y,x) = d;
	  minDifferenceValues.at<double>(y,x) = thisSSD;
	}
      }
    }
  }
*/

  StereoVar myStereoVar;
  // set parameters for disparity
  myStereoVar.levels = 3;
  myStereoVar.pyrScale = 0.5;
  myStereoVar.nIt = 25;
  myStereoVar.minDisp = 0;
  myStereoVar.maxDisp = ms->config.stereoMaxDisparity;
  myStereoVar.poly_n = 7;
  myStereoVar.poly_sigma = 0.0;
  myStereoVar.fi = 15.0f;
  myStereoVar.lambda = 0.03f;
  myStereoVar.penalization = myStereoVar.PENALIZATION_TICHONOV;
  myStereoVar.cycle = myStereoVar.CYCLE_V;
  myStereoVar.flags = myStereoVar.USE_SMART_ID 
		    | myStereoVar.USE_AUTO_PARAMS 
		    | myStereoVar.USE_INITIAL_DISPARITY 
		    | myStereoVar.USE_MEDIAN_FILTERING ;

  Mat dispOut;
  Mat sIm1Byte(sIm1.size(), CV_8UC3);
  Mat sIm2Byte(sIm2.size(), CV_8UC3);


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      sIm1Byte.at<Vec3b>(y,x)[0] = floor(sIm1.at<Vec3d>(y,x)[0]);
      sIm1Byte.at<Vec3b>(y,x)[1] = floor(sIm1.at<Vec3d>(y,x)[1]);
      sIm1Byte.at<Vec3b>(y,x)[2] = floor(sIm1.at<Vec3d>(y,x)[2]);
      sIm2Byte.at<Vec3b>(y,x)[0] = floor(sIm2.at<Vec3d>(y,x)[0]);
      sIm2Byte.at<Vec3b>(y,x)[1] = floor(sIm2.at<Vec3d>(y,x)[1]);
      sIm2Byte.at<Vec3b>(y,x)[2] = floor(sIm2.at<Vec3d>(y,x)[2]);
//cout << sIm2.at<Vec3d>(y,x)[0] << " ";
    }
  }

  myStereoVar(sIm1Byte, sIm2Byte, dispOut);
  ms->config.stereoDisparity = dispOut;
//  cvFindStereoCorrespondenceGC(const CvArr *left, const CvArr *right, CvArr *disparityLeft, CvArr *disparityRight, CvStereoGCState *state, int useDisparityGuess = 0)

  double eff = ms->config.stereoFocal;
  double bee = ms->config.stereoBaseline;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (ms->config.stereoDisparity.at<double>(y,x) <= 0) {
	ms->config.stereoDepth.at<double>(y,x) = VERYBIGNUMBER;
      } else {
	ms->config.stereoDepth.at<double>(y,x) = eff * bee / ms->config.stereoDisparity.at<uchar>(y,x);
      }
    }
  }
}
END_WORD
REGISTER_WORD(StereoCalculate)

WORD(StereoDisplay)
virtual void execute(MachineState * ms) {
  Size sz = ms->config.objectViewerYCbCrBlur.size();
  int imW = sz.width;
  int imH = sz.height;

  ms->config.stereoViewerImage.create(sz.height*2, sz.width*2, CV_8UC3);

  Mat sIm1 = ms->config.stereoImage1;
  Mat sIm2 = ms->config.stereoImage2;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double disparityMax = ms->config.stereoMaxDisparity;
      double thisDisparity = ms->config.stereoDisparity.at<uchar>(y,x);
      double thisDepth = ms->config.stereoDepth.at<double>(y,x);
      ms->config.stereoViewerImage.at<cv::Vec3b>(y,x) = sIm1.at<cv::Vec3d>(y,x);
      ms->config.stereoViewerImage.at<cv::Vec3b>(y,x+imW) = sIm2.at<cv::Vec3d>(y,x);
      ms->config.stereoViewerImage.at<cv::Vec3b>(y+imH,x) = Vec3d(255*(thisDisparity/disparityMax),0,0);
      ms->config.stereoViewerImage.at<cv::Vec3b>(y+imH,x+imW) = Vec3d(0,0,255*(thisDepth/1.0));
    }
  }

  if (ms->config.shouldIRender) {
    ms->config.stereoViewerWindow->updateImage(ms->config.stereoViewerImage);
  }
}
END_WORD
REGISTER_WORD(StereoDisplay)

WORD(MapWaypoints)
virtual string description() {
  return "Maps objects at locations specified by EePoseWords underneath.";
}
virtual void execute(MachineState * ms) {

  cout << "Entering mapWaypoints" << endl;

  eePose destPose;
  CONSUME_EEPOSE(destPose, ms);

  ms->pushWord("mapWaypoints");

  ms->pushWord("mapLocal");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight");
  ms->pushWord(std::make_shared<IntegerWord>(1));

  ms->pushWord("moveEeToPoseWord");
  ms->pushWord(std::make_shared<EePoseWord>(destPose));

}
END_WORD
REGISTER_WORD(MapWaypoints)

WORD(SetTrackbarLoHi)
virtual void execute(MachineState * ms)
{
  int newLo = 0;
  int newHi = 0;
  GET_ARG(ms, IntegerWord, newHi);
  GET_ARG(ms, IntegerWord, newLo);
  cout << "setTrackbarLoHi oldLo oldHi newLo newHi: " << ms->config.loTrackbarVariable << " " << ms->config.hiTrackbarVariable << " " << newLo << " " << newHi << endl;
  ms->config.loTrackbarVariable = newLo;
  ms->config.hiTrackbarVariable = newHi;
}
END_WORD
REGISTER_WORD(SetTrackbarLoHi)

}
