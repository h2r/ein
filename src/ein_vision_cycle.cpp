WORD(DeliverObject)
virtual void execute() {
  bailAfterGradient = 1;

  pilotTarget.px = -1;
  pilotTarget.py = -1;
  pilotClosestTarget.px = -1;
  pilotClosestTarget.py = -1;
  
  vector<BoxMemory> focusedClassMemories = memoriesForClass(focusedClass);
  if (focusedClassMemories.size() == 0) {
    cout << "can't find focused class. " << endl;
    return;
  }
  if (focusedClassMemories.size() > 1) {
    cout << "more than one bounding box for class.  Using first." << focusedClassMemories.size() << endl;
    return;
  }
  BoxMemory memory = focusedClassMemories[0];
  currentEEPose = memory.cameraPose;

  
  pushWord("placeObjectInDeliveryZone");
  pushWord("assertYesGrasp");
  pushWord("prepareForAndExecuteGraspFromMemorySimple"); 
  pushWord("gradientServo");
  pushWord("waitUntilAtCurrentPosition");
  pushWord("setPickModeToStaticMarginals"); 
  pushWord("setBoundingBoxModeToStaticMarginals"); 
  pushWord("synchronicServoDoNotTakeClosest"); 

}
END_WORD

WORD(PlaceObjectInDeliveryZone)
virtual void execute() {
  
  pushWord("assumeDeliveryPose");
}
END_WORD

WORD(MappingPatrol)
CODE(196727) // capslock + W
virtual void execute() {
  cout << "Mapping patrol" << endl;
  bailAfterSynchronic = 1;

  pushWord("mappingPatrol");
  pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
  pushWord("moveToNextMapPosition");
  //pushWord("setRandomPositionAndOrientationForHeightLearning");
  //pushWord("recordAllBlueBoxes");
  pushWord("mapClosestBlueBox");
  pushWord("synchronicServo"); 
  pushWord("visionCycle");
  pushWord("synchronicServoTakeClosest");
  pushCopies("noop", 5);
}
END_WORD

WORD(MoveToNextMapPosition)
virtual void execute() {
  ros::Time oldestTime = ros::Time::now();
  int oldestI=-1, oldestJ=-1;

  for (int i = 0; i < mapWidth; i++) {
    for (int j = 0; j < mapHeight; j++) {
      if (cellIsSearched(i, j) &&
          (objectMap[i + mapWidth * j].lastMappedTime <= oldestTime)) {
        oldestTime = objectMap[i + mapWidth * j].lastMappedTime;
        oldestI = i;
        oldestJ = j;
      }
    }
  }
  double oldestX, oldestY;
  mapijToxy(oldestI, oldestJ, &oldestX, &oldestY);

  currentEEPose.px = oldestX;
  currentEEPose.py = oldestY;
  pushWord("waitUntilAtCurrentPosition");
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



void mapBox(BoxMemory boxMemory) {
  for (double px = boxMemory.bTop.x; px <= boxMemory.bBot.x; px++) {
    for (double py = boxMemory.bTop.y; py <= boxMemory.bBot.y; py++) {
      double x, y;
      double z = trueEEPose.position.z + currentTableZ;


      pixelToGlobal(px, py, z, x, y);
      int i, j;
      mapxyToij(x, y, &i, &j);

      objectMap[i + mapWidth * j].lastMappedTime = ros::Time::now();
      objectMap[i + mapWidth * j].detectedClass = boxMemory.labeledClassIndex;


      if (cam_img.rows != 0 && cam_img.cols != 0) {
        objectMap[i + mapWidth * j].b += (int) cam_img.at<cv::Vec3b>(py, px)[0];
        objectMap[i + mapWidth * j].g += (int) cam_img.at<cv::Vec3b>(py, px)[1];
        objectMap[i + mapWidth * j].r += (int) cam_img.at<cv::Vec3b>(py, px)[2];
        objectMap[i + mapWidth * j].pixelCount += 1.0;
      }
    }
  }
}

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
  for (int px = grayTop.x; px < grayBot.x; px++) {
    for (int py = grayTop.y; py < grayBot.y; py++) {

      if (mask_gripper && isInGripperMask(px, py)) {
	continue;
      }

      int blueBoxIdx = blueBoxForPixel(px, py);
      if (blueBoxIdx == -1) {
        double x, y;
        double z = trueEEPose.position.z + currentTableZ;

        pixelToGlobal(px, py, z, x, y);
        int i, j;
        mapxyToij(x, y, &i, &j);
        objectMap[i + mapWidth * j].lastMappedTime = ros::Time::now();
        randomizeNanos(&objectMap[i + mapWidth * j].lastMappedTime);
        
        objectMap[i + mapWidth * j].detectedClass = -2;
        objectMap[i + mapWidth * j].b += (int) cam_img.at<cv::Vec3b>(py, px)[0];
        objectMap[i + mapWidth * j].g += (int) cam_img.at<cv::Vec3b>(py, px)[1];
        objectMap[i + mapWidth * j].r += (int) cam_img.at<cv::Vec3b>(py, px)[2];
        objectMap[i + mapWidth * j].pixelCount += 1.0;
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
  
  mapBox(box);

  vector<BoxMemory> newMemories;
  
  for (int i = 0; i < blueBoxMemories.size(); i++) {
    if (!boxMemoryIntersects(box, blueBoxMemories[i])) {
      newMemories.push_back(blueBoxMemories[i]);
    }
  }
  newMemories.push_back(box);
  

  blueBoxMemories = newMemories;

}
END_WORD




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
  pushCopies("density", 4); 
  pushCopies("resetTemporalMap", 1); 
  pushCopies("density", 1); 
}
END_WORD

WORD(Density)
CODE(131121)     // capslock + 1
virtual void execute() {
  goCalculateDensity();
  goCalculateObjectness();
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
