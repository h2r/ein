WORD(DeliverObject)
virtual void execute() {
  bailAfterGradient = 1;

  pilotTarget.px = -1;
  pilotTarget.py = -1;
  pilotClosestTarget.px = -1;
  pilotClosestTarget.py = -1;
  
  int idxOfFirst = -1;
  vector<BoxMemory> focusedClassMemories = memoriesForClass(focusedClass, &idxOfFirst);
  if (focusedClassMemories.size() == 0) {
    cout << "can't find focused class. " << endl;
    return;
  }
  if (focusedClassMemories.size() > 1) {
    cout << "more than one bounding box for class.  Using first." << focusedClassMemories.size() << endl;
  }
  BoxMemory memory = focusedClassMemories[0];
  //currentEEPose = memory.cameraPose;
  currentEEPose = memory.aimedPose;
  lastPickHeight = memory.pickedPose.pz;
  lastPrePickHeight = memory.aimedPose.pz;

  { // set the old box's lastMappedTime to moments after the start of time
    int iStart=-1, iEnd=-1, jStart=-1, jEnd=-1;
    int iTop=-1, iBot=-1, jTop=-1, jBot=-1;
    double z = trueEEPose.position.z + currentTableZ;
    {
      double x, y;
      int i, j;
      pixelToGlobal(memory.top.px-mapBlueBoxPixelSkirt, memory.top.py-mapBlueBoxPixelSkirt, z, &x, &y);
      mapxyToij(x, y, &i, &j);
      iTop=i;
      jTop=j;
    }
    {
      double x, y;
      int i, j;
      pixelToGlobal(memory.bot.px+mapBlueBoxPixelSkirt, memory.bot.py+mapBlueBoxPixelSkirt, z, &x, &y);
      mapxyToij(x, y, &i, &j);
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
	if (i >= 0 && i < mapWidth && j >= 0 && j < mapHeight) {
	  objectMap[i + mapWidth * j].lastMappedTime = ros::Time(0.001);
	  randomizeNanos(&objectMap[i + mapWidth * j].lastMappedTime);
	}
      }
    }
  }
  
  { // remove this blue box; above stamping would cause it to be naturally eliminated IF it was 
    // observed in a searched region...
    vector<BoxMemory> newMemories;
    for (int i = 0; i < blueBoxMemories.size(); i++) {
      if (i != idxOfFirst) {
	cout << "Retaining blue box " << i << " while booting " << idxOfFirst << endl; 
	newMemories.push_back(blueBoxMemories[i]);
      }
    }
    blueBoxMemories = newMemories;
  }

  pushWord("clearStackIntoMappingPatrol"); 
  pushWord("moveToNextMapPosition");
  pushWord("synchronicServoDoNotTakeClosest"); 
  pushWord("openGripper"); 
  pushWord("cruisingSpeed"); 
  pushWord("waitUntilAtCurrentPosition"); 
  pushWord("tryToMoveToTheLastPrePickHeight");   
  pushWord("departureSpeed");
  pushWord("placeObjectInDeliveryZone");
  pushWord("ifGrasp");
  pushWord("executePreparedGrasp"); 
  //pushWord("prepareForAndExecuteGraspFromMemory"); 
  //pushWord("gradientServo");
  //pushCopies("density", densityIterationsForGradientServo); 
  //pushCopies("resetTemporalMap", 1); 
  //pushWord("synchronicServo"); 
  //pushWord("visionCycle");
  //pushWord("synchronicServoTakeClosest"); 
  pushWord("waitUntilAtCurrentPosition");
  pushWord("setPickModeToStaticMarginals"); 
  pushWord("sampleHeight"); 
  pushWord("setBoundingBoxModeToMapping"); 
  pushWord("openGripper");
}
END_WORD

WORD(PlaceObjectInDeliveryZone)
virtual void execute() {
  pushWord("openGripper"); 
  pushWord("tryToMoveToTheLastPickHeight");   
  pushWord("approachSpeed"); 
  pushWord("waitUntilAtCurrentPosition"); 
  pushWord("assumeDeliveryPose");
  pushWord("cruisingSpeed");
  pushWord("waitUntilAtCurrentPosition"); 
  pushWord("tryToMoveToTheLastPrePickHeight");   
  pushWord("departureSpeed");
}
END_WORD

WORD(ClearStackIntoMappingPatrol)
virtual void execute() {
  clearStack();
  pushWord("mappingPatrol");
  execute_stack = 1;
}
END_WORD


WORD(MappingPatrol)
CODE(196727) // capslock + W
virtual void execute() {
  cout << "Mapping patrol" << endl;
  bailAfterSynchronic = 1;
  bailAfterGradient = 1;
  acceptingFetchCommands = 1;

  pushWord("mappingPatrol");
  //pushWord("bringUpAllNonessentialSystems");
  //pushWord("endStackCollapse");
  pushWord("moveToNextMapPosition");
  pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
  //pushWord("setRandomPositionAndOrientationForHeightLearning");
  //pushWord("recordAllBlueBoxes");
  pushWord("filterBoxMemories");
  pushWord("shiftIntoGraspGear1");
  pushWord("lockTargetIfBlueBoxes");
  //pushWord("collapseStack");
  pushWord("gradientServoIfBlueBoxes");
  pushWord("mapClosestBlueBox");
  pushWord("goClassifyBlueBoxes"); 
  pushWord("synchronicServo"); 
  pushWord("visionCycleNoClassify");
  pushWord("synchronicServoTakeClosest");
  pushWord("waitUntilAtCurrentPosition"); 
  pushWord("moveToNextMapPosition");
  pushWord("sampleHeight"); 
  pushWord("setBoundingBoxModeToMapping");
  pushWord("shiftIntoGraspGear1");
  pushWord("cruisingSpeed");
  //pushWord("shutdownAllNonessentialSystems");
  //pushWord("bringUpAllNonessentialSystems");
}
END_WORD

WORD(ToggleShouldIDoIK)
virtual void execute() {
  shouldIDoIK = !shouldIDoIK;
}
END_WORD

WORD(ToggleShouldIRender)
virtual void execute() {
  shouldIRender = !shouldIRender;
}
END_WORD

WORD(ToggleDrawClearanceMap)
virtual void execute() {
  drawClearanceMap = !drawClearanceMap;
}
END_WORD

WORD(ToggleDrawIKMap)
virtual void execute() {
  drawIKMap = !drawIKMap;
}
END_WORD

WORD(ToggleUseGlow)
virtual void execute() {
  useGlow = !useGlow;
}
END_WORD

WORD(ToggleUseFade)
virtual void execute() {
  useFade = !useFade;
}
END_WORD

WORD(FillClearanceMap)
int pursuitProximity = 5;
int searchProximity = 20;
virtual void execute() {
  {
    int proximity = pursuitProximity;
    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapHeight; j++) {
	if ( cellIsSearched(i, j) ) {
	  int iIStart = max(0, i-proximity);
	  int iIEnd = min(mapWidth-1, i+proximity);
	  int iJStart = max(0, j-proximity);
	  int iJEnd = min(mapHeight-1, j+proximity);

	  int reject = 0;
	  for (int iI = iIStart; iI <= iIEnd; iI++) {
	    for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	      if (  ( cellIsSearched(iI, iJ) ) && 
		    ( ikMap[iI + mapWidth * iJ] != 0 ) &&
		    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity )  ) {
		reject = 1;
	      }
	    }
	  }

	  if (reject) {
	    clearanceMap[i + mapWidth * j] = 0;
	  } else {
	    clearanceMap[i + mapWidth * j] = 1;
	  }
	} else {
	  clearanceMap[i + mapWidth * j] = 0;
	}
      }
    }
  }
  {
    int proximity = searchProximity;
    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapHeight; j++) {
	if ( cellIsSearched(i, j) ) {
	  int iIStart = max(0, i-proximity);
	  int iIEnd = min(mapWidth-1, i+proximity);
	  int iJStart = max(0, j-proximity);
	  int iJEnd = min(mapHeight-1, j+proximity);

	  int reject = 0;
	  for (int iI = iIStart; iI <= iIEnd; iI++) {
	    for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	      if (  ( cellIsSearched(iI, iJ) ) && 
		    ( ikMap[iI + mapWidth * iJ] != 0 ) &&
		    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity )  ) {
		reject = 1;
	      }
	    }
	  }

	  if (reject) {
	  } else {
	    clearanceMap[i + mapWidth * j] = 2;
	  }
	} else {
	}
      }
    }
  }
}
END_WORD

WORD(SaveIkMap)
virtual void execute() {
  ofstream ofile;
  string fileName = data_directory + "/" + left_or_right_arm + "IkMap";
  cout << "Saving ikMap to " << fileName << endl;
  ofile.open(fileName, ios::trunc | ios::binary);
  ofile.write((char*)ikMap, sizeof(int)*mapWidth*mapHeight);
  ofile.close();
}
END_WORD

WORD(LoadIkMap)
virtual void execute() {
  ifstream ifile;
  string fileName = data_directory + "/" + left_or_right_arm + "IkMap";
  cout << "Loading ikMap from " << fileName << endl;
  ifile.open(fileName, ios::binary);
  ifile.read((char*)ikMap, sizeof(int)*mapWidth*mapHeight);
  ifile.close();
}
END_WORD

WORD(FillIkMap)
// store these here and create accessors if they need to change
int currentI = 0;
int currentJ = 0;
int cellsPerQuery = 100;
virtual void execute() {
  int queries = 0;
  int i=currentI, j=currentJ;
  for (; i < mapWidth; i++) {
    if (queries < cellsPerQuery) {
      for (; j < mapHeight; j++) {
	if (queries < cellsPerQuery) {
	  if ( cellIsSearched(i, j) ) {
	    double X, Y;
	    mapijToxy(i, j, &X, &Y);

	    eePose nextEEPose = currentEEPose;
	    nextEEPose.px = X;
	    nextEEPose.py = Y;

	    baxter_core_msgs::SolvePositionIK thisIkRequest;
	    endEffectorAngularUpdate(&nextEEPose);
	    fillIkRequest(&nextEEPose, &thisIkRequest);

	    bool likelyInCollision = 0;
	    int thisIkCallResult = ikClient.call(thisIkRequest);
	    int ikResultFailed = willIkResultFail(thisIkRequest, thisIkCallResult, &likelyInCollision);
	    int foundGoodPosition = !ikResultFailed;
	    //ikMap[i + mapWidth * j] = ikResultFailed;
	    //ikMap[i + mapWidth * j] = 1;
	    //cout << i << " " << j << endl;
	    if (ikResultFailed) {
	      ikMap[i + mapWidth * j] = 1;
	    } else {
	      if (likelyInCollision) {
		ikMap[i + mapWidth * j] = 2;
	      } else {
		ikMap[i + mapWidth * j] = 0;
	      }
	    }
	    queries++;
	  }
	} else {
	  break;
	}
      }
      // reset here so we don't bash the initial restart
      if ( !(j < mapHeight) ) {
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

  if (i >= mapWidth) {
    i = 0;
  } else {
    pushWord("fillIkMap");
  }
  if (j >= mapHeight) {
    j = 0;
  }

  currentI = i;
  currentJ = j;
}
END_WORD

WORD(MoveToNextMapPosition)
int maxNextTries = 100;
virtual void execute() {
  for (int tries = 0; tries < maxNextTries; tries++) {
    //ros::Time oldestTime = ros::Time::now();
    int oldestI=-1, oldestJ=-1;
    int foundASpot = 0;
    for (int scanRestarter = 0; scanRestarter < 2; scanRestarter++) {
      ros::Time oldestTime = lastScanStarted;
      for (int i = 0; i < mapWidth; i++) {
	for (int j = 0; j < mapHeight; j++) {
	  if (cellIsSearched(i, j) &&
	      (objectMap[i + mapWidth * j].lastMappedTime <= oldestTime) &&
	      (clearanceMap[i + mapWidth * j] == 2) &&
	      (ikMap[i + mapWidth * j] == 0) ) {
	    oldestTime = objectMap[i + mapWidth * j].lastMappedTime;
	    oldestI = i;
	    oldestJ = j;
	    foundASpot = 1;
	  }
	}
      }

      if (!foundASpot) {
	lastScanStarted = ros::Time::now();
	cout << "All spots visited. Restarting scan." << endl;
      }
    }

    if (oldestI == -1 || oldestJ == -1) {
      cout << "moveToNextMapPosition failed to find a position. Clearing callstack." << endl;
      clearStack();
      pushCopies("beep", 15); // beep
      return;
    }

    double oldestX, oldestY;
    mapijToxy(oldestI, oldestJ, &oldestX, &oldestY);

    eePose nextEEPose = currentEEPose;
    nextEEPose.px = oldestX;
    nextEEPose.py = oldestY;

    baxter_core_msgs::SolvePositionIK thisIkRequest;
    fillIkRequest(&nextEEPose, &thisIkRequest);

    bool likelyInCollision = 0;
    int thisIkCallResult = ikClient.call(thisIkRequest);
    int ikResultFailed = willIkResultFail(thisIkRequest, thisIkCallResult, &likelyInCollision);
    int foundGoodPosition = !ikResultFailed;

    if (foundGoodPosition) {
      currentEEPose.px = oldestX;
      currentEEPose.py = oldestY;
      cout << "This pose was accepted by ikClient:" << endl;
      cout << "Next EE Position (x,y,z): " << nextEEPose.px << " " << nextEEPose.py << " " << nextEEPose.pz << endl;
      cout << "Next EE Orientation (x,y,z,w): " << nextEEPose.qx << " " << nextEEPose.qy << " " << nextEEPose.qz << " " << nextEEPose.qw << endl;
      pushWord("waitUntilAtCurrentPosition");
      cout << "moveToNextMapPosition tries foundGoodPosition oldestI oldestJ oldestX oldestY: "  << tries << " " << foundGoodPosition << " "  << oldestI << " " << oldestJ << " " << oldestX << " " << oldestY << endl;
      break;
    } else {
      cout << "moveToNextMapPosition tries foundGoodPosition oldestI oldestJ: "  << tries << " " << foundGoodPosition << " "  << oldestI << " " << oldestJ << " " << oldestX << " " << oldestY << endl;
      cout << "Try number try: " << tries << ", adding point to ikMap oldestI oldestJ ikMap[.]: " << " " << oldestI << " " << oldestJ;
      if (ikResultFailed) {
	ikMap[oldestI + mapWidth * oldestJ] = 1;
      } else {
	if (likelyInCollision) {
	  ikMap[oldestI + mapWidth * oldestJ] = 2;
	} else {
	  ikMap[oldestI + mapWidth * oldestJ] = 0;
	}
      }
      cout << " " << ikMap[oldestI + mapWidth * oldestJ] << endl;
    }
  }
}
END_WORD

WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)
virtual void execute() {
  object_recognition_msgs::RecognizedObjectArray roa;
  visualization_msgs::MarkerArray ma; 
 
  roa.objects.resize(0);

  roa.header.stamp = ros::Time::now();
  roa.header.frame_id = "/base";


  for (int class_i = 0; class_i < classLabels.size(); class_i++) {
    string class_label = classLabels[class_i];
    if (class_label != "background") {
      eePose centroid;
      centroid.px = 0;
      centroid.py = 0;
      centroid.pz = 0;
      int class_count = 0;
      for (int j = 0; j < blueBoxMemories.size(); j++) {
        if (blueBoxMemories[j].labeledClassIndex == class_i) {
          centroid.px += blueBoxMemories[j].centroid.px;
          centroid.py += blueBoxMemories[j].centroid.py;
          centroid.pz += blueBoxMemories[j].centroid.pz;
          class_count += 1;
        }
      }
      if (class_count == 0) {
        continue;
      }
      centroid.px = centroid.px / class_count;
      centroid.py = centroid.py / class_count;
      centroid.pz = centroid.pz / class_count;
      int closest_idx = -1;
      double min_square_dist = VERYBIGNUMBER;

      for (int j = 0; j < blueBoxMemories.size(); j++) {
        if (blueBoxMemories[j].labeledClassIndex == class_i) {
          double square_dist = 
            squareDistanceEEPose(centroid, blueBoxMemories[j].centroid);
          if (square_dist < min_square_dist) {
            min_square_dist = square_dist;
            closest_idx = j;
          }
        }
      }


      if (closest_idx != -1) {
        geometry_msgs::Pose pose;
        int aI = roa.objects.size();
        roa.objects.resize(roa.objects.size() + 1);
        ma.markers.resize(ma.markers.size() + 1);

        pose.position.x = blueBoxMemories[closest_idx].centroid.px;
        pose.position.y = blueBoxMemories[closest_idx].centroid.py;
        pose.position.z = blueBoxMemories[closest_idx].centroid.pz;

        cout << "blueBoxMemories: " << blueBoxMemories[closest_idx].centroid.px << endl;
        cout << "pose: " << pose.position.x << endl;

        roa.objects[aI].pose.pose.pose.position = pose.position;

        cout << "roa objects x: " << roa.objects[aI].pose.pose.pose.position.x << endl;
        roa.objects[aI].type.key = class_label;

        ma.markers[aI].pose = roa.objects[aI].pose.pose.pose;
        cout << "marker pose x: " << ma.markers[aI].pose.position.x << endl;
        roa.objects[aI].header = roa.header;
        ma.markers[aI].header = roa.header;

        ma.markers[aI].type =  visualization_msgs::Marker::SPHERE;
        ma.markers[aI].scale.x = 0.15;
        ma.markers[aI].scale.y = 0.15;
        ma.markers[aI].scale.z = 0.15;
        ma.markers[aI].color.a = 0.5;
        ma.markers[aI].color.r = 0.9;
        ma.markers[aI].color.g = 0.9;
        ma.markers[aI].color.b = 0.0;
        ma.markers[aI].id = aI;
        ma.markers[aI].lifetime = ros::Duration(1.0);



      }
    }
  }

  rec_objs_blue_memory.publish(roa);
  markers_blue_memory.publish(ma);

}
END_WORD


WORD(RecordAllBlueBoxes)
virtual void execute() {
  cout << "Recording blue boxes: " << bTops.size() << endl;
  for (int c = 0; c < bTops.size(); c++) {
    BoxMemory box;
    box.bTop = bTops[c];
    box.bBot = bBots[c];
    box.cameraPose = currentEEPose;
    box.top = pixelToGlobalEEPose(box.bTop.x, box.bTop.y, trueEEPose.position.z + currentTableZ);
    box.bot = pixelToGlobalEEPose(box.bBot.x, box.bBot.y, trueEEPose.position.z + currentTableZ);
    box.centroid.px = (box.top.px + box.bot.px) * 0.5;
    box.centroid.py = (box.top.py + box.bot.py) * 0.5;
    box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
    box.cameraTime = ros::Time::now();
    box.labeledClassIndex = bLabels[c];
    blueBoxMemories.push_back(box);
  }

}
END_WORD

WORD(InitializeMap)
virtual void execute() {
  initializeMap();

}
END_WORD;

bool isInGripperMask(int x, int y) {
  if ( (x >= g1xs && x <= g1xe && y >= g1ys && y <= g1ye) ||
       (x >= g2xs && x <= g2xe && y >= g2ys && y <= g2ye) ) {
    return true;
  } else {
    return false;
  }
}

WORD(MapEmptySpace)
virtual void execute() {
  for (int px = grayTop.x+mapGrayBoxPixelSkirt; px < grayBot.x-mapGrayBoxPixelSkirt; px++) {
    for (int py = grayTop.y+mapGrayBoxPixelSkirt; py < grayBot.y-mapGrayBoxPixelSkirt; py++) {

      if (mask_gripper && isInGripperMask(px, py)) {
	continue;
      }

      //int blueBoxIdx = blueBoxForPixel(px, py);
      int blueBoxIdx = skirtedBlueBoxForPixel(px, py, mapFreeSpacePixelSkirt);

      if (blueBoxIdx == -1) {
        double x, y;
        double z = trueEEPose.position.z + currentTableZ;

        pixelToGlobal(px, py, z, &x, &y);
        int i, j;
        mapxyToij(x, y, &i, &j);

//        if (ros::Time::now() - objectMap[i + mapWidth * j].lastMappedTime > mapMemoryTimeout) {
//          objectMap[i + mapWidth * j].b = 0;
//          objectMap[i + mapWidth * j].g = 0;
//          objectMap[i + mapWidth * j].r = 0;
//          objectMap[i + mapWidth * j].pixelCount = 0;
//        }


        objectMap[i + mapWidth * j].lastMappedTime = ros::Time::now();
        randomizeNanos(&objectMap[i + mapWidth * j].lastMappedTime);
        
        objectMap[i + mapWidth * j].detectedClass = -2;

//	{
//	  objectMap[i + mapWidth * j].b += (int) cam_img.at<cv::Vec3b>(py, px)[0];
//	  objectMap[i + mapWidth * j].g += (int) cam_img.at<cv::Vec3b>(py, px)[1];
//	  objectMap[i + mapWidth * j].r += (int) cam_img.at<cv::Vec3b>(py, px)[2];
//        objectMap[i + mapWidth * j].pixelCount += 1.0;
//	}
	//const double spaceDecay = 0.996; // 0.7 ^ 0.01
	const double spaceDecay = 0.99821; // 0.7 ^ 0.005
	{
	  objectMap[i + mapWidth * j].b = 
	    ( spaceDecay*double(objectMap[i + mapWidth * j].b) + 
		    (1.0-spaceDecay)*double(cam_img.at<cv::Vec3b>(py, px)[0]) );
	  objectMap[i + mapWidth * j].g = 
	    ( spaceDecay*double(objectMap[i + mapWidth * j].g) + 
		    (1.0-spaceDecay)*double(cam_img.at<cv::Vec3b>(py, px)[1]) );
	  objectMap[i + mapWidth * j].r = 
	    ( spaceDecay*double(objectMap[i + mapWidth * j].r) + 
		    (1.0-spaceDecay)*double(cam_img.at<cv::Vec3b>(py, px)[2]) );
	  objectMap[i + mapWidth * j].pixelCount = 
	    ( spaceDecay*double(objectMap[i + mapWidth * j].pixelCount) + 
		    (1.0-spaceDecay)*double(1.0) );
	}

      }
    }
  }
}
END_WORD;





WORD(MapClosestBlueBox)
virtual void execute() {
  if (pilotClosestBlueBoxNumber == -1) {
    cout << "Not changing because closest bbox is " << pilotClosestBlueBoxNumber << endl;
    return;
  }

  int c = pilotClosestBlueBoxNumber;
  BoxMemory box;
  box.bTop = bTops[c];
  box.bBot = bBots[c];
  box.cameraPose = currentEEPose;
  box.top = pixelToGlobalEEPose(box.bTop.x, box.bTop.y, trueEEPose.position.z + currentTableZ);
  box.bot = pixelToGlobalEEPose(box.bBot.x, box.bBot.y, trueEEPose.position.z + currentTableZ);
  box.centroid.px = (box.top.px + box.bot.px) * 0.5;
  box.centroid.py = (box.top.py + box.bot.py) * 0.5;
  box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
  box.cameraTime = ros::Time::now();
  box.labeledClassIndex = bLabels[c];
  
  int i, j;
  mapxyToij(box.centroid.px, box.centroid.py, &i, &j);

  // this only does the timestamp to avoid obsessive behavior
  mapBox(box);
  
  //if ( !positionIsSearched(box.centroid.px, box.centroid.py) && 
       //!isCellInPursuitZone(i, j) ) 
  //if (!positionIsSearched(box.centroid.px, box.centroid.py)) 
  if ( !positionIsSearched(box.centroid.px, box.centroid.py) || 
       !isBoxMemoryIKPossible(box) ) 
  {
    return;
  } else {
    vector<BoxMemory> newMemories;
    for (int i = 0; i < blueBoxMemories.size(); i++) {
      if (!boxMemoryIntersectCentroid(box, blueBoxMemories[i])) {
	newMemories.push_back(blueBoxMemories[i]);
      }
    }
    newMemories.push_back(box);
    blueBoxMemories = newMemories;
  }
}
END_WORD


WORD(FilterBoxMemories)
virtual void execute() {
  set<int> boxMemoryIndexesToKeep;

  for (int b_i = 0; b_i < blueBoxMemories.size(); b_i++) {
    int keep = 0;
    BoxMemory b = blueBoxMemories[b_i];
    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapHeight; j++) {
        if (boxMemoryIntersectsMapCell(b, i, j)) {
          //if (b.cameraTime.sec > objectMap[i + mapWidth * j].lastMappedTime.sec) {
          ros::Duration diff = objectMap[i + mapWidth * j].lastMappedTime - b.cameraTime;

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
    newMemories.push_back(blueBoxMemories[*it]);
  }
  blueBoxMemories = newMemories;
}
END_WORD;

WORD(ClearBlueBoxMemories)
CODE(196709) // capslock + E
virtual void execute() {
  cout << "Clearing blue box memory: " << blueBoxMemories.size() << endl;
  blueBoxMemories.resize(0);
}
END_WORD

WORD(VisionCycle)
CODE(131153)  // capslock + q
virtual void execute() {
  pushWord("mapEmptySpace");
  pushWord("goClassifyBlueBoxes"); 
  pushWord("goFindBlueBoxes"); 
  pushCopies("density", 1); 
  pushWord("hover");
  //pushCopies("resetTemporalMap", 1); 
  //pushCopies("density", 1); 
}
END_WORD

WORD(Density)
virtual void execute() {
  pushWord("densityA");
  //pushWord("waitUntilImageCallbackReceived");
  pushCopies("waitUntilImageCallbackReceived", 5);
}
END_WORD

WORD(DensityA)
CODE(131121)     // capslock + 1
virtual void execute() {
  substituteLatestImageQuantities();
  goCalculateDensity();
}
END_WORD

WORD(AccumulatedDensity)
virtual void execute() {
  substituteAccumulatedImageQuantities();
  goCalculateDensity();
  renderAccumulatedImageAndDensity();
  //goAccumulateForAerial();
}
END_WORD

WORD(ResetAccumulatedDensity)
virtual void execute() {
  resetAccumulatedImageAndMass();
}
END_WORD

WORD(ResetTemporalMap)
CODE(1179737) // capslock + numlock + y
virtual void execute() {
  if (temporalDensity != NULL && preDensity != NULL) {
    //cout << "preDensity<<<<***" << endl;
    Size sz = objectViewerImage.size();
    int imW = sz.width;
    int imH = sz.height;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
        temporalDensity[y*imW+x] = preDensity[y*imW+x];
      }
    }
  }
}
END_WORD

WORD(GoFindBlueBoxes)
CODE(131122) // capslock + 2
virtual void execute() {
  goFindBlueBoxes();
}
END_WORD


WORD(GoClassifyBlueBoxes)
CODE(131123) // capslock + 3
virtual void execute() {
  lastVisionCycle = ros::Time::now();
  oscilStart = ros::Time::now();
  goClassifyBlueBoxes();
}
END_WORD
