

WORD(SetTargetClassToLastLabelLearned)
CODE(1179730)     // capslock + numlock + r
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int i = 0; i < ms->config.numClasses; i++) {
    if (ms->config.lastLabelLearned.compare(ms->config.classLabels[i]) == 0) {
      ms->config.targetClass = i;
      ms->config.focusedClass = ms->config.targetClass;
      ms->config.focusedClassLabel = ms->config.classLabels[ms->config.focusedClass];
      cout << "lastLabelLearned classLabels[targetClass]: " << ms->config.lastLabelLearned << " " << ms->config.classLabels[ms->config.targetClass] << endl;
      changeTargetClass(ms, ms->config.targetClass);
    }
  }

  ms->pushWord("drawMapRegisters"); // render register 1
  // ATTN 10
  //ms->pushWord(196360); // loadPriorGraspMemory
  //ms->pushWord(1179721); // set graspMemories from classGraspMemories
  switch (ms->config.currentPickMode) {
  case STATIC_PRIOR:
    {
      ms->pushWord(196360); // loadPriorGraspMemory
    }
    return;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      ms->pushWord(1179721); // set graspMemories from classGraspMemories
      //ms->pushWord(196360); // loadPriorGraspMemory
    }
    break;
  case STATIC_MARGINALS:
    {
      ms->pushWord(1179721); // set graspMemories from classGraspMemories
      //ms->pushWord(196360); // loadPriorGraspMemory
    }
    return;
  default:
    {
      assert(0);
    }
    return;
  }
}
END_WORD
REGISTER_WORD(SetTargetClassToLastLabelLearned)


WORD(SetLastLabelLearned)
CODE(1179732)    // capslock + numlock + t 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.lastLabelLearned = ms->config.focusedClassLabel;
  cout << "lastLabelLearned: " << ms->config.lastLabelLearned << endl;
}
END_WORD
REGISTER_WORD(SetLastLabelLearned)

WORD(TrainModels)
CODE(131142)     // capslock + f
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->config.classLabels.resize(0);
  ms->config.classPoseModels.resize(0);

  ms->pushWord("clearBlueBoxMemories");


  // snoop folders
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s/objects/", ms->config.data_directory.c_str());
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(buf);
      thisFullFileName = thisFullFileName + "/" + thisFileName;

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
        ms->config.classLabels.push_back(thisFileName);
        ms->config.classPoseModels.push_back("B");
      }
    }
  }

  if ((ms->config.classLabels.size() != ms->config.classPoseModels.size()) || (ms->config.classLabels.size() < 1)) {
    cout << "Label and pose model list size problem. Not proceeding to train." << endl;
    return;
  }

  cout << "Reinitializing and retraining. " << endl;
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    cout << ms->config.classLabels[i] << " " << ms->config.classPoseModels[i] << endl;
  }

  ms->config.rewrite_labels = 1;
  ms->config.retrain_vocab = 1;
  ms->config.reextract_knn = 1;

  // delete things that will be reallocated
  if (ms->config.bowTrainer)
    delete ms->config.bowTrainer;
  if (ms->config.kNN)
    delete ms->config.kNN;

  for (int i = 0; i < ms->config.classPosekNNs.size(); i++) {
    if (ms->config.classPosekNNs[i])
      delete ms->config.classPosekNNs[i];
  }

  //  detectorsInit() will reset numClasses
  detectorsInit(ms);

  // reset numNewClasses
  ms->config.newClassCounter = 0;

  // XXX reset anything else
}
END_WORD
REGISTER_WORD(TrainModels)


WORD(VisionCycleNoClassify)
CODE(196721)     // capslock + Q
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->pushWord("mapEmptySpace");
  ms->pushWord("goFindBlueBoxes"); // blue boxes
  ms->pushCopies("density", 1); // density
  ms->pushWord("hover"); // blue boxes
}
END_WORD
REGISTER_WORD(VisionCycleNoClassify)

WORD(RecordExampleAsFocusedClass)
CODE(131148)     // capslock + l 
virtual void execute(std::shared_ptr<MachineState> ms)       {
  if ((ms->config.focusedClass > -1) && (ms->config.bTops.size() == 1)) {
    string thisLabelName = ms->config.focusedClassLabel;
    Mat crop = ms->config.cam_img(cv::Rect(ms->config.bTops[0].x, ms->config.bTops[0].y, ms->config.bBots[0].x-ms->config.bTops[0].x, ms->config.bBots[0].y-ms->config.bTops[0].y));
    char buf[1000];
    string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/rgb/";
    sprintf(buf, "%s%s%s_%d.ppm", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), ms->config.cropCounter);
    imwrite(buf, crop);
    ms->config.cropCounter++;
  }
}
END_WORD
REGISTER_WORD(RecordExampleAsFocusedClass)

WORD(RecordAllExamplesFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  if ( ms->config.focusedClass > -1 ) {
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      string thisLabelName = ms->config.focusedClassLabel;
      Mat crop = ms->config.cam_img(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      char buf[1000];
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/rgb/";
      sprintf(buf, "%s%s%s_%d.ppm", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), ms->config.cropCounter);
      imwrite(buf, crop);
      ms->config.cropCounter++;
    }
  }
}
END_WORD
REGISTER_WORD(RecordAllExamplesFocusedClass)

WORD(SetRandomOrientationForPhotospin)
CODE(1310722)     // capslock + numlock + "
virtual void execute(std::shared_ptr<MachineState> ms) {
  // this ensures that we explore randomly within each grasp gear sector
  double arcFraction = 0.125;
  double noTheta = arcFraction * 3.1415926 * ((drand48() - 0.5) * 2.0);
  ms->config.currentEEDeltaRPY.pz += noTheta;
}
END_WORD
REGISTER_WORD(SetRandomOrientationForPhotospin)

WORD(RgbScan)
CODE(131143)      // capslock + g
virtual void execute(std::shared_ptr<MachineState> ms)       {
  // ATTN 16

  ms->pushCopies('e', 5);
  ms->pushCopies('a', 5);
  ms->pushWord(196711); // photospin
  ms->pushCopies('q', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('q', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('d', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('d', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('e', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('e', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('a', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('q', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin

  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord(1245246); // uniformly sample height
  pushSpeedSign(ms, MOVE_FAST);
}
END_WORD
REGISTER_WORD(RgbScan)


WORD(PhotoSpin)
CODE(196711)      // capslock + G
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int angleCounter = 0; angleCounter < ms->config.totalGraspGears; angleCounter++) {
    //ms->pushWord(131148); // save crop as focused class if there is only one
    ms->pushWord("recordAllExamplesFocusedClass");
    ms->pushWord(196721); // vision cycle no classify
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord(1310722); // set random orientation for photospin.
    ms->pushWord(196712); // increment grasp gear
  }
  ms->pushWord("shiftIntoGraspGear1"); // change gear to 1
}
END_WORD
REGISTER_WORD(PhotoSpin)

WORD(SetTargetReticleToTheMaxMappedPosition)
CODE(1048678)  // numlock + f
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.trX = ms->config.rmcX + ms->config.rmDelta*(ms->config.maxX-ms->config.rmHalfWidth);
  ms->config.trY = ms->config.rmcY + ms->config.rmDelta*(ms->config.maxY-ms->config.rmHalfWidth);
}
END_WORD
REGISTER_WORD(SetTargetReticleToTheMaxMappedPosition)

WORD(DownsampleIrScan)
CODE(1048690) // numlock + r
virtual void execute(std::shared_ptr<MachineState> ms) {
  // replace unsampled regions with the lowest z reading, highest reading in those maps because they are inverted
  // 
  double highestReading = -VERYBIGNUMBER;
  double highestEpsilonMassReading = -VERYBIGNUMBER;
  double readingFloor = -1;
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      for (int rrx = rx*10; rrx < (rx+1)*10; rrx++) {
        for (int rry = ry*10; rry < (ry+1)*10; rry++) {
          if (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > 0.0) {
            //if ((hiRangeMap[rrx + rry*ms->config.hrmWidth] > highestReading) && (ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth] >= readingFloor))
            if ((ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth] > highestEpsilonMassReading) && (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > EPSILON))
              highestEpsilonMassReading = ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth];

            if ((ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth] > highestReading) && (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > 0))
              highestReading = ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth];
          }
        }
      }
    }
  }


  if (highestReading <= -VERYBIGNUMBER) {
    highestReading = 0;
  }

	
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      double thisSum = 0;
      double numSamples = 0;
      for (int rrx = rx*10; rrx < (rx+1)*10; rrx++) {
        for (int rry = ry*10; rry < (ry+1)*10; rry++) {
          numSamples += 1.0;
          if (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > 0.0) {
            thisSum += ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth];
          } else {
            thisSum += 0;
          }
        }
      }
      ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth] = thisSum/numSamples;
    }
  }
}
END_WORD
REGISTER_WORD(DownsampleIrScan)



WORD(ScanObject)
CODE(196708)     // capslock + D
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "ENTERING WHOLE FOODS VIDEO MAIN." << endl;
  cout << "Program will pause shortly. Please adjust height for bounding box servo before unpausing." << endl;
  cout << "Program will pause a second time. Please adjust height for IR scan before unpausing." << endl;
  cout << "Program will pause a third time. Please remove any applied contrast agents." << endl;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  // so that closest servoing doesn't go into gradient servoing.
  ms->config.targetClass = -1;


  // this automatically changes learning mode
          
  if (0) {
    ms->pushWord("beginHeightLearning"); // begin bounding box learning

    ms->pushWord("changeToHeight1"); // change to height 1
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }

  if (1) {
    ms->pushWord("saveLearnedModels");
    ms->pushWord("loadPriorGraspMemoryAnalytic");
    // set target class to the lastLabelLearned 
    ms->pushWord(1179730);
    ms->pushWord(131142); // reinitialize and retrain everything
  }

  // set lastLabelLearned
  ms->pushWord(1179732);

  if (0) {
    ms->pushWord(131143); // 72 way scan
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord(131143); // 72 way scan
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  }

  ms->pushWord("scanCentered"); // 72 way scan

  // this is a good time to remove a contrast agent
  //ms->pushWord('Y'); // pause stack execution
  //ms->pushCopies("beep", 15); // beep
	  
  { // do density and gradient, save gradient, do medium scan in two directions, save range map
    pushSpeedSign(ms, MOVE_FAST);
    ms->pushWord("saveCurrentClassDepthAndGraspMaps"); // save current depth map to current class
    ms->pushWord("neutralScan"); // neutral scan 
    ms->pushWord('Y'); // pause stack execution
    ms->pushCopies("beep", 15); // beep
    pushSpeedSign(ms, MOVE_FAST);

    ms->pushWord("changeToHeight1"); // change to height 1

    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight3"); // change to height 3
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight2"); // change to height 2
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight1"); // change to height 1
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight0"); // change to height 0
    }
  }

  // ATTN 3
  // start NO bag routine
  ms->pushWord(196720); //  make a new class

  ms->pushWord(131139); // synchronic servo don't take closest
  ms->pushWord(131156); // synchronic servo
  ms->pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
  ms->pushWord("visionCycle"); // vision cycle

  ms->pushWord('Y'); // pause stack execution
  ms->pushCopies("beep", 15); // beep

  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord(1245219); // change to height 2
  pushSpeedSign(ms, MOVE_FAST);
  //ms->pushWord(196672); // go to wholeFoodsCounter1

  ms->pushWord(1179735); // change to counter table
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord('k'); // open gripper
}
END_WORD
REGISTER_WORD(ScanObject)


WORD(PrepareForSearch)
CODE(1114150)     // numlock + &
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX this should be computed here from the ir sensor offset
  ms->config.currentEEPose.px = ms->config.rmcX + ms->config.drX;
  ms->config.currentEEPose.py = ms->config.rmcY + ms->config.drY;
}
END_WORD
REGISTER_WORD(PrepareForSearch)
 

WORD(TurnOnRecordRangeMap)
CODE(1048683) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.recordRangeMap = 1;
}
END_WORD
REGISTER_WORD(TurnOnRecordRangeMap)

WORD(SetRangeMapCenterFromCurrentEEPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Set rmcX and rmcY from ms->config.currentEEPose." << endl;
  ms->config.rmcX = ms->config.currentEEPose.px;
  ms->config.rmcY = ms->config.currentEEPose.py;
  //ms->config.rmcZ = ms->config.currentEEPose.pz - ms->config.eeRange;
}
END_WORD
REGISTER_WORD(SetRangeMapCenterFromCurrentEEPose)

WORD(InitDepthScan)
CODE(1048695) // numlock + w
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Set rmcX and rmcY. Resetting maps. " << ms->config.rmcX << " " << ms->config.trueEEPose.position.x << endl;
  ms->config.rmcX = ms->config.trueEEPose.position.x;
  ms->config.rmcY = ms->config.trueEEPose.position.y;
  ms->config.rmcZ = ms->config.trueEEPose.position.z - ms->config.eeRange;
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      ms->config.rangeMap[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth] = 0;
      // ATTN 17
      //rangeMapReg2[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapMass[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapAccumulator[rx + ry*ms->config.rmWidth] = 0;
    }
  }
  {
    cv::Scalar backColor(128,0,0);
    cv::Point outTop = cv::Point(0,0);
    cv::Point outBot = cv::Point(ms->config.rmiWidth,ms->config.rmiHeight);
    Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop = backColor;
  }
  for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
    for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
      ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapReg1[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapReg2[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapMass[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapAccumulator[rx + ry*ms->config.hrmWidth] = 0;
    }
  }
  {
    cv::Scalar backColor(128,0,0);
    cv::Point outTop = cv::Point(0,0);
    cv::Point outBot = cv::Point(ms->config.hrmiWidth,ms->config.hrmiHeight);
    Mat vCrop = ms->config.hiRangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop = backColor;
  }
  for (int h = 0; h < ms->config.hrmWidth; h++) {
    for (int i = 0; i < ms->config.hrmWidth; i++) {
      ms->config.hiColorRangeMapMass[h + i*ms->config.hrmWidth] = 0;
      for (int j = 0; j < 3; j++) {
        ms->config.hiColorRangeMapAccumulator[h + i*ms->config.hrmWidth + j*ms->config.hrmWidth*ms->config.hrmWidth] = 0;
      }
    }
  }
  for (int pz = 0; pz < ms->config.vmWidth; pz++) {
    for (int py = 0; py < ms->config.vmWidth; py++) {
      for (int px = 0; px < ms->config.vmWidth; px++) {
        ms->config.volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
        ms->config.volumeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
        ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
        ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
        for (int pc = 0; pc < 3; pc++) {
          ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + pc*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] = 0;
        }
      }
    }
  }
  {
    cv::Scalar backColor(128,0,0);
    cv::Point outTop = cv::Point(0,0);
    cv::Point outBot = cv::Point(ms->config.hiColorRangemapImage.cols,ms->config.hiColorRangemapImage.rows);
    Mat vCrop = ms->config.hiColorRangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop = backColor;
  }
}
END_WORD
REGISTER_WORD(InitDepthScan)




WORD(NeutralScan)
CODE(1048622) // numlock + .
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("cruisingSpeed");
  ms->pushWord("neutralScanA");
  ms->pushWord("rasterScanningSpeed");
}
END_WORD
REGISTER_WORD(NeutralScan)

WORD(NeutralScanA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering neutral scan." << endl;
  double lineSpeed = MOVE_FAST;//MOVE_MEDIUM;//MOVE_FAST;
  double betweenSpeed = MOVE_FAST;//MOVE_MEDIUM;//MOVE_FAST;

  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord(1114150); // prepare for search

  ms->pushCopies('q',4);
  ms->pushCopies('a',6);

  ms->pushWord(1048683); // turn on scanning
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord(1114155); // rotate gear

  ms->pushWord("fullRender"); // full render
  ms->pushWord("paintReticles"); // render reticle
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord("drawMapRegisters"); // render register 1
  ms->pushWord("downsampleIrScan"); // load map to register 1
  {
    ms->pushWord(1048678); // target best grasp
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }
  ms->pushWord(1048630); // find best grasp

  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord(1114150); // prepare for search

  ms->pushWord(1048683); // turn on scanning
  ms->pushWord("initDepthScan"); // clear scan history
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("shiftIntoGraspGear1"); 
}
END_WORD
REGISTER_WORD(NeutralScanA)

WORD(NeutralScanB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering neutralScanB." << endl;
  double lineSpeed = ms->config.bDelta;
  double betweenSpeed = ms->config.bDelta;

  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord(1114150); // prepare for search

  ms->pushCopies('q',4);
  ms->pushCopies('a',6);

  ms->pushWord(1048683); // turn on scanning
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord(1114155); // rotate gear

  ms->pushWord("fullRender"); // full render
  ms->pushWord("paintReticles"); // render reticle
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord("drawMapRegisters"); // render register 1
  ms->pushWord("downsampleIrScan"); // load map to register 1
  {
    ms->pushWord(1048678); // target best grasp
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }
  ms->pushWord(1048630); // find best grasp

  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord(1114150); // prepare for search

  ms->pushWord(1048683); // turn on scanning
  ms->pushWord("initDepthScan"); // clear scan history
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("shiftIntoGraspGear1"); 
}
END_WORD
REGISTER_WORD(NeutralScanB)

WORD(NeutralScanH)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering HALF neutral scan." << endl;
  double lineSpeed = MOVE_FAST;//MOVE_MEDIUM;//MOVE_FAST;
  double betweenSpeed = MOVE_FAST;//MOVE_MEDIUM;//MOVE_FAST;

  ms->pushWord("fullRender"); // full render
  ms->pushWord("paintReticles"); // render reticle
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord("drawMapRegisters"); // render register 1
  ms->pushWord("downsampleIrScan"); // load map to register 1
  {
    ms->pushWord(1048678); // target best grasp
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }
  ms->pushWord(1048630); // find best grasp

  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord(1114150); // prepare for search

  ms->pushWord(1048683); // turn on scanning
  ms->pushWord("initDepthScan"); // clear scan history
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("shiftIntoGraspGear1"); 
}
END_WORD
REGISTER_WORD(NeutralScanH)

WORD(SaveAerialGradientMap)
CODE(196730)      // capslock + Z
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
        
  cout << "save aerial gradient ";
  if ((ms->config.focusedClass > -1) && (ms->config.frameGraySobel.rows >1) && (ms->config.frameGraySobel.cols > 1)) {
    string thisLabelName = ms->config.focusedClassLabel;

    char buf[1000];
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/aerialGradient/";
    string this_range_path;

    // ATTN 16
    switch (ms->config.currentThompsonHeightIdx) {
    case 0:
      {
        this_range_path = dirToMakePath + "aerialHeight0Gradients.yml";
      }
      break;
    case 1:
      {
        this_range_path = dirToMakePath + "aerialHeight1Gradients.yml";
      }
      break;
    case 2:
      {
        this_range_path = dirToMakePath + "aerialHeight2Gradients.yml";
      }
      break;
    case 3:
      {
        this_range_path = dirToMakePath + "aerialHeight3Gradients.yml";
      }
      break;
    default:
      {
        assert(0);
        break;
      }
    }

    mkdir(dirToMakePath.c_str(), 0777);

    //int hbb = ms->config.pilotTargetBlueBoxNumber;
    //int hbb = 0;

    int topCornerX = ms->config.reticle.px - (ms->config.aerialGradientReticleWidth/2);
    int topCornerY = ms->config.reticle.py - (ms->config.aerialGradientReticleWidth/2);
    int crows = ms->config.aerialGradientReticleWidth;
    int ccols = ms->config.aerialGradientReticleWidth;

    //int crows = ms->config.bBots[hbb].y - ms->config.bTops[hbb].y;
    //int ccols = ms->config.bBots[hbb].x - ms->config.bTops[hbb].x;
    int maxDim = max(crows, ccols);
    int tRy = (maxDim-crows)/2;
    int tRx = (maxDim-ccols)/2;
    Mat gCrop(maxDim, maxDim, ms->config.frameGraySobel.type());

    cout << "crows ccols: " << crows << " " << ccols << " ";

    for (int x = 0; x < maxDim; x++) {
      for (int y = 0; y < maxDim; y++) {
        int tx = x - tRx;
        int ty = y - tRy;
        int tCtx = topCornerX + tx;
        int tCty = topCornerY + ty;
        if ( (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) &&
             (tCtx > 0) && (tCty > 0) && (tCtx < imW) && (tCty < imH) ) {
          //gCrop.at<double>(y, x) = ms->config.frameGraySobel.at<double>(ms->config.bTops[hbb].y + ty, ms->config.bTops[hbb].x + tx);
          gCrop.at<double>(y, x) = ms->config.frameGraySobel.at<double>(tCty, tCtx);
        } else {
          gCrop.at<double>(y, x) = 0.0;
        }
      }
    }
  
    cout << "about to resize" << endl;

    Size toBecome(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth);
    cv::resize(gCrop, gCrop, toBecome);


    FileStorage fsvO;
    cout << "capslock + Z: Writing: " << this_range_path << endl;

    fsvO.open(this_range_path, FileStorage::WRITE);

    // ATTN 16
    switch (ms->config.currentThompsonHeightIdx) {
    case 0:
      {
        fsvO << "aerialHeight0Gradients" << gCrop;
      }
      break;
    case 1:
      {
        fsvO << "aerialHeight1Gradients" << gCrop;
      }
      break;
    case 2:
      {
        fsvO << "aerialHeight2Gradients" << gCrop;
      }
      break;
    case 3:
      {
        fsvO << "aerialHeight3Gradients" << gCrop;
      }
      break;
    default:
      {
        assert(0);
      }
      break;
    }
    fsvO.release();
  } 
}
END_WORD
REGISTER_WORD(SaveAerialGradientMap)

WORD(InitializeAndFocusOnNewClass)
CODE(196720)     // capslock + P
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.focusedClass = ms->config.numClasses+ms->config.newClassCounter;
  char buf[1024];
  sprintf(buf, "autoClass%d_%s", ms->config.focusedClass, ms->config.left_or_right_arm.c_str());
  string thisLabelName(buf);
  ms->config.focusedClassLabel = thisLabelName;
  ms->config.classLabels.push_back(thisLabelName);
  string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/";
  mkdir(dirToMakePath.c_str(), 0777);
  string rgbDirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/rgb";
  mkdir(rgbDirToMakePath.c_str(), 0777);
  ms->config.newClassCounter++;
}
END_WORD
REGISTER_WORD(InitializeAndFocusOnNewClass)

WORD(SaveCurrentClassDepthAndGraspMaps)
CODE(196705) // capslock + A
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX TODO is this function even ever used anymore?
  if (ms->config.focusedClass > -1) {
    // initialize this if we need to
    guardGraspMemory(ms);
    guardHeightMemory(ms);

    string thisLabelName = ms->config.focusedClassLabel;

    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ir2D/";
    string this_range_path = dirToMakePath + "xyzRange.yml";

    Mat rangeMapTemp(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
	rangeMapTemp.at<double>(y,x) = ms->config.rangeMapReg1[x + y*ms->config.rmWidth];
      } 
    } 

    mkdir(dirToMakePath.c_str(), 0777);

    FileStorage fsvO;
    cout << "capslock + A: Writing: " << this_range_path << endl;
    fsvO.open(this_range_path, FileStorage::WRITE);

    {
      fsvO << "graspZ" << "[" 
	<< ms->config.currentGraspZ 
      << "]";

      if (ms->config.classGraspZs.size() > ms->config.focusedClass) {
	ms->config.classGraspZs[ms->config.focusedClass] = ms->config.currentGraspZ;
      }
      if (ms->config.classGraspZsSet.size() > ms->config.focusedClass) {
	ms->config.classGraspZsSet[ms->config.focusedClass] = 1;
      }
    }

    fsvO << "rangeMap" << rangeMapTemp;
    copyGraspMemoryTriesToClassGraspMemoryTries(ms);
    fsvO << "graspMemoryTries1" << ms->config.classGraspMemoryTries1[ms->config.focusedClass];
    fsvO << "graspMemoryPicks1" << ms->config.classGraspMemoryPicks1[ms->config.focusedClass];
    fsvO << "graspMemoryTries2" << ms->config.classGraspMemoryTries2[ms->config.focusedClass];
    fsvO << "graspMemoryPicks2" << ms->config.classGraspMemoryPicks2[ms->config.focusedClass];
    fsvO << "graspMemoryTries3" << ms->config.classGraspMemoryTries3[ms->config.focusedClass];
    fsvO << "graspMemoryPicks3" << ms->config.classGraspMemoryPicks3[ms->config.focusedClass];
    fsvO << "graspMemoryTries4" << ms->config.classGraspMemoryTries4[ms->config.focusedClass];
    fsvO << "graspMemoryPicks4" << ms->config.classGraspMemoryPicks4[ms->config.focusedClass];

    copyHeightMemoryTriesToClassHeightMemoryTries(ms);
    fsvO << "heightMemoryTries" << ms->config.classHeightMemoryTries[ms->config.focusedClass];
    fsvO << "heightMemoryPicks" << ms->config.classHeightMemoryPicks[ms->config.focusedClass];

    fsvO.release();
  } 
}
END_WORD
REGISTER_WORD(SaveCurrentClassDepthAndGraspMaps)

WORD(ScanCentered)
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushSpeedSign(ms, MOVE_FAST);
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("synchronicServo"); 
  ms->pushWord("visionCycleNoClassify");
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("fillClearanceMap");
  ms->pushWord("loadIkMap");
  ms->config.currentBoundingBoxMode = MAPPING;
  ms->config.bDelta = 0.001;
}
END_WORD
REGISTER_WORD(ScanCentered)

WORD(SetTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.firstTableHeightTime = ros::Time::now();
  ms->config.mostRecentUntabledZLastValue = ms->config.mostRecentUntabledZ;
  ms->pushWord("setTableA");
}
END_WORD
REGISTER_WORD(SetTable)

WORD(SetTableA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ros::Time thisTableHeightTime = ros::Time::now();

  ms->config.mostRecentUntabledZLastValue = ((1.0-ms->config.mostRecentUntabledZDecay)*ms->config.mostRecentUntabledZ) + (ms->config.mostRecentUntabledZDecay*ms->config.mostRecentUntabledZLastValue);

  ms->config.oneTable = ms->config.mostRecentUntabledZLastValue;
  ms->config.rightTableZ = ms->config.oneTable;
  ms->config.leftTableZ = ms->config.oneTable;
  ms->config.bagTableZ = ms->config.oneTable;
  ms->config.counterTableZ = ms->config.oneTable;
  ms->config.pantryTableZ = ms->config.oneTable;
  ms->config.currentTableZ = ms->config.oneTable;

  if ( fabs(thisTableHeightTime.sec - ms->config.firstTableHeightTime.sec) > ms->config.mostRecentUntabledZWait) {
    // do nothing
  } else {
    double utZDelta = fabs(ms->config.mostRecentUntabledZ - ms->config.mostRecentUntabledZLastValue);
    ms->config.endThisStackCollapse = 1;
    ms->pushWord("setTableA");
    cout << "Looks like the table reading hasn't steadied for the wait of " << ms->config.mostRecentUntabledZWait << " ." << endl;
    cout << "  current, last, delta: " << ms->config.mostRecentUntabledZ << " " << ms->config.mostRecentUntabledZLastValue << " " << utZDelta << endl;
  } 
}
END_WORD
REGISTER_WORD(SetTableA)

WORD(SetIROffset)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("setIROffsetA");
  ms->pushWord("cruisingSpeed");
  ms->pushWord("neutralScanH");
  ms->pushWord("iRCalibrationSpeed");
  cout << "Commencing HALF neutral scan for SetIROffset." << endl;
}
END_WORD
REGISTER_WORD(SetIROffset)

WORD(ZeroIROffset)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gear0offset = Eigen::Quaternionf(0.0, 0.0, 0.0, 0.0);
  Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
  ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
}
END_WORD
REGISTER_WORD(ZeroIROffset)

WORD(SetIROffsetA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // find the maximum in the map
  // find the coordinate of the maximum
  // compare the coordinate to the root position
  // adjust offset
  // if adjustment was large, recommend running again
  double minDepth = VERYBIGNUMBER;
  double maxDepth = 0;
  int minX=-1, minY=-1;
  int maxX=-1, maxY=-1;

  for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
    for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
      double thisDepth = ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth];
      if (thisDepth < minDepth) {
	minDepth = thisDepth;
	minX = rx;
	minY = ry;
      }
      if (thisDepth > maxDepth) {
	maxDepth = thisDepth;
	maxX = rx;
	maxY = ry;
      }
    }
  }

  double offByX = ((minX-ms->config.hrmHalfWidth)*ms->config.hrmDelta);
  double offByY = ((minY-ms->config.hrmHalfWidth)*ms->config.hrmDelta);

  cout << "SetIROffsetA, ms->config.hrmHalfWidth minX minY offByX offByY: " << ms->config.hrmHalfWidth << " " << minX << " " << minY << " " << offByX << " " << offByY << endl;

  ms->config.gear0offset = Eigen::Quaternionf(0.0, 
    ms->config.gear0offset.x()+offByX, 
    ms->config.gear0offset.y()+offByY, 
    0.0167228); // z is from TF, good for depth alignment

  Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
  ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
}
END_WORD
REGISTER_WORD(SetIROffsetA)

WORD(SetHeightReticles)
virtual void execute(std::shared_ptr<MachineState> ms) {

  int heightWaits = 100;
  int numPause = 4; 
  

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight3");

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight2");

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight1");

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0");

}
END_WORD
REGISTER_WORD(SetHeightReticles)

WORD(PrintGlobalToPixel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    int eX=0, eY=0;
    //globalToPixel(&eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
    globalToPixelPrint(ms, &eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
  }
}
END_WORD
REGISTER_WORD(PrintGlobalToPixel)

WORD(SetHeightReticlesA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  ms->config.heightReticles[ms->config.currentThompsonHeightIdx].px = darkX;
  ms->config.heightReticles[ms->config.currentThompsonHeightIdx].py = darkY;

  cout << "setHeightReticles,  currentThompsonHeightIdx: " << ms->config.currentThompsonHeightIdx << endl;
  eePose::print(ms->config.heightReticles[0]); cout << endl;
  eePose::print(ms->config.heightReticles[1]); cout << endl;
  eePose::print(ms->config.heightReticles[2]); cout << endl;
  eePose::print(ms->config.heightReticles[3]); cout << endl;
}
END_WORD
REGISTER_WORD(SetHeightReticlesA)

WORD(MoveCropToCenter)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  ms->config.cropUpperLeftCorner.px = 320;
  ms->config.cropUpperLeftCorner.py = 200;

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  int testResult = ms->config.cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(MoveCropToCenter)

WORD(MoveCropToProperValue)
virtual void execute(std::shared_ptr<MachineState> ms) {
  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  int testResult = ms->config.cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(MoveCropToProperValue)

WORD(MoveCropToCenterVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  double Vx = ms->config.vanishingPointReticle.px - (imW/2);
  double Vy = ms->config.vanishingPointReticle.py - (imH/2);

  ms->config.cropUpperLeftCorner.px += Vx;
  ms->config.cropUpperLeftCorner.py += Vy;
  ms->config.vanishingPointReticle.px -= Vx;
  ms->config.vanishingPointReticle.py -= Vy;

  cout << "MoveCropToCenterVanishingPoint Vx Vy: " << Vx << " " << Vy << endl;

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  int testResult = ms->config.cameraClient.call(ocMessage);
  //cout << "centerVanishingPoint testResult: " << testResult << endl;
  //cout << ocMessage.response.name << endl;
  //cout << ocMessage.response.name << " " << ocMessage.response.settings.controls.size() << endl;

  //cout << "MoveCropToCenterVanishingPoint moving region of interest and vanishing point. Recalibrate vanishing point, height reticles, and magnification factors." << endl;
}
END_WORD
REGISTER_WORD(MoveCropToCenterVanishingPoint)

WORD(MoveToSetVanishingPointHeightLow)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose.pz = ms->config.minHeight - ms->config.currentTableZ;
}
END_WORD
REGISTER_WORD(MoveToSetVanishingPointHeightLow)

WORD(MoveToSetVanishingPointHeightHigh)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose.pz = ((0.75*ms->config.maxHeight)+(0.25*ms->config.minHeight)) - ms->config.currentTableZ;
}
END_WORD
REGISTER_WORD(MoveToSetVanishingPointHeightHigh)

WORD(SetVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {

  ms->config.setVanishingPointIterations = 0;
  // go low, wait
  ms->pushWord("setVanishingPointA");
  // is darkest point in current vp? loop here until it is so then rise and go to B
  ms->pushWord("setVanishingPointPrep");
}
END_WORD
REGISTER_WORD(SetVanishingPoint)

WORD(SetVanishingPointPrep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("darkServo");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToSetVanishingPointHeightLow");
}
END_WORD
REGISTER_WORD(SetVanishingPointPrep)

WORD(SetVanishingPointA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int numPause = 4;

  ms->config.setVanishingPointIterations++;
  ms->pushWord("setVanishingPointB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 100);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToSetVanishingPointHeightHigh");
}
END_WORD
REGISTER_WORD(SetVanishingPointA)

WORD(SetVanishingPointB)
virtual void execute(std::shared_ptr<MachineState> ms) {

  // where is the darkest point now? did it move? move vp to darkest point and possibly run again
  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  int Px = darkX - ms->config.vanishingPointReticle.px;
  int Py = darkY - ms->config.vanishingPointReticle.py;

  ms->config.vanishingPointReticle.px = darkX;
  ms->config.vanishingPointReticle.py = darkY;
  
  cout << "setVanishingPoint Px Py: " << Px << " " << Py << endl;

  if (ms->config.setVanishingPointIterations > ms->config.setVanishingPointTimeout) {
    cout << "setVanishingPoint timed out, continuing..." << endl;
  }

  if ((fabs(Px) < ms->config.setVanishingPointPixelThresh) && (fabs(Py) < ms->config.setVanishingPointPixelThresh)) {
    cout << "vanishing point set, continuing." << endl;
  } else {
    cout << "vanishing point not set, adjusting more. " << ms->config.setVanishingPointIterations << " " << ms->config.setVanishingPointTimeout << endl;
    ms->pushWord("setVanishingPointA");
    ms->pushWord("setVanishingPointPrep");
  }
}
END_WORD
REGISTER_WORD(SetVanishingPointB)


WORD(SetMagnification)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int translationSteps = 5;
  int imCallsToWait = 10;

  int nudgeSteps = 4;

  // move back
  // adjust until close	
  // move back over then down 
  // adjust until close	
  // move over 
  // go to height
  translationSteps = 15;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight3");

  translationSteps = 15;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight2");

  translationSteps = 15;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight1");

  translationSteps = 10;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight0");
}
END_WORD
REGISTER_WORD(SetMagnification)

WORD(SetMagnificationA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // adjust until close	

  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  int magIters = 2000; 
  double magStep = 0.01;

  for (int i = 0; i < magIters; i++) {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    int eX=0, eY=0;
    //globalToPixel(&eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
    globalToPixelPrint(ms, &eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);

    // remember this is flipped!
    double Px = darkY - eY;
    double Py = darkX - eX;

    double xFlip = 1.0;
    double yFlip = 1.0;

    // remember x, y are swapped
    eePose thisFlipReticle = ms->config.heightReticles[ms->config.currentThompsonHeightIdx];
    if (darkX < thisFlipReticle.px) {
      yFlip = -1.0;
    }
    if (darkY < thisFlipReticle.py) {
      xFlip = -1.0;
    }

    cout << "about to adjust m_x, darkX eX Px xFlip darkY eY Py yFlip: " << darkX << " " << eX << " " << Px << " " << xFlip << " " << darkY << " " << eY << " " << Py << " " << yFlip << " ";

    // only do x
    if ((Px*xFlip) > 0) {
      ms->config.m_x += .01;
      ms->config.m_x_h[ms->config.currentThompsonHeightIdx] = ms->config.m_x;
      cout << "m_x++ ";
    } else if ((Px*xFlip) < 0) {
      ms->config.m_x -= .01;
      ms->config.m_x_h[ms->config.currentThompsonHeightIdx] = ms->config.m_x;
      cout << "m_x-- ";
    }

    cout << endl;
  }
}
END_WORD
REGISTER_WORD(SetMagnificationA)

WORD(SetMagnificationB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // adjust until close	

  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  int magIters = 2000; 
  double magStep = 0.01;

  for (int i = 0; i < magIters; i++) {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    int eX=0, eY=0;
    //globalToPixel(&eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
    globalToPixelPrint(ms, &eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);

    // remember this is flipped!
    double Px = darkY - eY;
    double Py = darkX - eX;

    double xFlip = 1.0;
    double yFlip = 1.0;

    // remember x, y are swapped
    eePose thisFlipReticle = ms->config.heightReticles[ms->config.currentThompsonHeightIdx];
    if (darkX < thisFlipReticle.px) {
      yFlip = -1.0;
    }
    if (darkY < thisFlipReticle.py) {
      xFlip = -1.0;
    }

    cout << "about to adjust m_y, darkX eX Px xFlip darkY eY Py yFlip: " << darkX << " " << eX << " " << Px << " " << xFlip << " " << darkY << " " << eY << " " << Py << " " << yFlip << " ";

    // only do y
    if ((Py*yFlip) > 0) {
      ms->config.m_y += .01;
      ms->config.m_y_h[ms->config.currentThompsonHeightIdx] = ms->config.m_y;
      cout << "m_y++ ";
    } else if ((Py*yFlip) < 0) {
      ms->config.m_y -= .01;
      ms->config.m_y_h[ms->config.currentThompsonHeightIdx] = ms->config.m_y;
      cout << "m_y-- ";
    }

    cout << endl;
  }
}
END_WORD
REGISTER_WORD(SetMagnificationB)

WORD(SetGripperMaskOnes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gripperMask.create(ms->config.gripperMaskFirstContrast.size(), CV_8U);
  ms->config.cumulativeGripperMask.create(ms->config.gripperMaskFirstContrast.size(), CV_8U);

  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.gripperMask.at<uchar>(y,x) = 1;
      ms->config.cumulativeGripperMask.at<uchar>(y,x) = 1;
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskOnes)

WORD(SetGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Program paused; please present the first contrast medium." << endl;
  ms->pushWord("setGripperMaskA"); 
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("pauseStackExecution"); 
}
END_WORD
REGISTER_WORD(SetGripperMask)

WORD(SetGripperMaskAA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gripperMaskFirstContrast = ms->config.accumulatedImage.clone();
  ms->config.gripperMaskSecondContrast = ms->config.gripperMaskFirstContrast.clone();

  ms->config.gripperMask.create(ms->config.gripperMaskFirstContrast.size(), CV_8U);

  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[0] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[1] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[2] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      ms->config.gripperMask.at<uchar>(y,x) = 0;
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskAA)

WORD(InitCumulativeGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.cumulativeGripperMask.create(ms->config.accumulatedImage.size(), CV_8U);
  Size sz = ms->config.cumulativeGripperMask.size();
  int imW = sz.width;
  int imH = sz.height;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.cumulativeGripperMask.at<uchar>(y,x) = 0;
    }
  }
}
END_WORD
REGISTER_WORD(InitCumulativeGripperMask)

WORD(SetGripperMaskA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Program paused; please present the second contrast medium." << endl;
  ms->pushWord("setGripperMaskB"); 
  ms->pushWord("setGripperMaskBA"); 
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("pauseStackExecution"); 
  ms->pushWord("setGripperMaskAA"); 
  ms->pushWord("initCumulativeGripperMask"); 
}
END_WORD
REGISTER_WORD(SetGripperMaskA)

WORD(SetGripperMaskB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Thank you. Don't forget to save your mask!" << endl;
}
END_WORD
REGISTER_WORD(SetGripperMaskB)

WORD(SetGripperMaskBA)
virtual void execute(std::shared_ptr<MachineState> ms) {

  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  int dilationPixels = 10;
  double baseThresh = 5;
  double multiThresh = 3*baseThresh*baseThresh;

  cout << "  multiThresh dilationPixels: " << multiThresh << " " << dilationPixels << endl;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[0] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[1] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[2] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      double maskDiff = 
      ((ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[0] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[0])*
      (ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[0] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[0])) +
      ((ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[1] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[1])*
      (ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[1] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[1])) +
      ((ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[2] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[2])*
      (ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[2] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[2]));

      if(maskDiff < 1000) {
	//cout << multiThresh << " " << maskDiff << endl;
      }

      if (maskDiff > multiThresh) {
	ms->config.gripperMask.at<uchar>(y,x) = 1;
      } else {
	ms->config.gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }

  Mat tmpMask = ms->config.gripperMask.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) == 0) {
	int xmin = max(0, x - dilationPixels);
	int xmax = min(imW-1, x + dilationPixels);
	int ymin = max(0, y - dilationPixels);
	int ymax = min(imH-1, y + dilationPixels);
	for (int xp = xmin; xp < xmax; xp++) {
	  for (int yp = ymin; yp < ymax; yp++) {
	    ms->config.gripperMask.at<uchar>(yp,xp) = 0;
	  }
	}
      }
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskBA)

WORD(SetGripperMaskWithMotion)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("setGripperMaskWithMotionA");
  ms->pushWord("initCumulativeGripperMask");
}
END_WORD
REGISTER_WORD(SetGripperMaskWithMotion)

WORD(SetGripperMaskWithMotionA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int maskMotions = 25;
  cout << "Setting gripper mask with motion, iterations: " << maskMotions << endl;

  for (int m = 0; m < maskMotions; m++) {
    // watch it as it develops
    ms->pushWord("saveGripperMask");
    ms->pushWord("setGripperMaskCB");

    // once observed always observed
    ms->pushWord("setGripperMaskCA");

    ms->pushWord("setGripperMaskBA"); 
    ms->pushWord("accumulatedDensity");
    ms->pushCopies("waitUntilImageCallbackReceived", 10);
    ms->pushWord("resetAccumulatedDensity");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");

    // move speed not set so that you can control for aliasing from repl
    ms->pushWord("yDown");
  }

  ms->pushWord("setGripperMaskAA"); 
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(SetGripperMaskWithMotionA)

WORD(SetGripperMaskCA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.cumulativeGripperMask = max(ms->config.cumulativeGripperMask, ms->config.gripperMask);
}
END_WORD
REGISTER_WORD(SetGripperMaskCA)

WORD(SetGripperMaskCB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gripperMask = ms->config.cumulativeGripperMask.clone();
  cout << "Thank you. Don't forget to save your mask!" << endl;
}
END_WORD
REGISTER_WORD(SetGripperMaskCB)

WORD(LoadGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename = ms->config.data_directory + "/config/" + ms->config.left_or_right_arm + "GripperMask.bmp";
  cout << "Loading gripper mask from " << filename << endl;
  Mat tmpMask = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  cout << "  tmpMask.type() tmpMask.size(): " << tmpMask.type() << " " << tmpMask.size() << endl;

  ms->config.gripperMask.create(tmpMask.size(), CV_8U);
  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) > 0) {
	ms->config.gripperMask.at<uchar>(y,x) = 1;
      } else {
	ms->config.gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }
  
}
END_WORD
REGISTER_WORD(LoadGripperMask)

WORD(SaveGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename = ms->config.data_directory + "/config/" + ms->config.left_or_right_arm + "GripperMask.bmp";
  cout << "Saving gripper mask to " << filename << endl;
  imwrite(filename, 255*ms->config.gripperMask);
}
END_WORD
REGISTER_WORD(SaveGripperMask)

WORD(CalibrateRGBCameraIntrinsics)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("setMagnification");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");

  ms->pushWord("setHeightReticles");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");

  ms->pushWord("setVanishingPoint");
  ms->pushWord("moveCropToCenter");

  int tablePeek = 5;
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", tablePeek);
  ms->pushWord("setTable");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xDown", tablePeek);
  ms->pushWord("setMovementSpeedMoveFast");

  ms->pushWord("saveRegister1");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(CalibrateRGBCameraIntrinsics)

WORD(AssumeCalibrationPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose = ms->config.calibrationPose;
}
END_WORD
REGISTER_WORD(AssumeCalibrationPose)

WORD(LoadCalibration)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string fileName = ms->config.data_directory + "/config/" + ms->config.left_or_right_arm + "Calibration.yml";
  cout << "Loading calibration file from " << fileName << endl;
  loadCalibration(ms, fileName);
}
END_WORD
REGISTER_WORD(LoadCalibration)

WORD(SaveCalibration)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string fileName = ms->config.data_directory + "/config/" + ms->config.left_or_right_arm + "Calibration.yml";
  cout << "Saving calibration file from " << fileName << endl;
  saveCalibration(ms, fileName);
}
END_WORD
REGISTER_WORD(SaveCalibration)

WORD(SetColorReticles)
virtual void execute(std::shared_ptr<MachineState> ms) {

  ms->config.bDelta = ms->config.cReticleIndexDelta;
  ms->config.currentEEPose.pz = ms->config.firstCReticleIndexDepth;

  // leave it in a canonical state
  ms->pushWord("setMovementSpeedMoveFast");

  int * ii = &(pMachineState->config.scrI);
  (*ii) = 0;

  for (int i = 0; i < ms->config.numCReticleIndeces; i++) {
    ms->pushWord("zUp");
    ms->pushWord("setColorReticlesA");
    ms->pushWord("accumulatedDensity");
    ms->pushCopies("waitUntilImageCallbackReceived", 100);
    ms->pushWord("resetAccumulatedDensity");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
  }
}
END_WORD
REGISTER_WORD(SetColorReticles)

WORD(SetColorReticlesA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int lightX = 0;
  int lightY = 0;
  findLight(ms, &lightX, &lightY);

  ms->config.pilotTarget.px = lightX;
  ms->config.pilotTarget.py = lightY;

  int * ii = &(pMachineState->config.scrI);
  ms->config.xCR[(*ii)] = lightX;
  ms->config.yCR[(*ii)] = lightY;

  (*ii)++;
}
END_WORD
REGISTER_WORD(SetColorReticlesA)

WORD(ScanObjectFast)
virtual void execute(std::shared_ptr<MachineState> ms) {

  int retractCm = 10;
  
  cout << "BEGINNING SCANOBJECTFAST" << endl;
  cout << "Program will pause shortly. Please adjust height and object so that arm would grip if closed and so that the gripper will clear the object during a scan once raised 5cm." << endl;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  // so that closest servoing doesn't go into gradient servoing.
  ms->config.targetClass = -1;

  // set lastLabelLearned
  ms->pushWord(1179732);

  ms->pushWord("setMovementSpeedMoveFast");
  ms->config.currentBoundingBoxMode = MAPPING; // this is here because it is for the rgbScan
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("fullImpulse");
  ms->pushWord("setMovementSpeedMoveVerySlow");

  ms->pushWord("changeToHeight1"); 
  ms->pushWord("comeToHover");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");

  { // do density and gradient, save gradient, do medium scan in two directions, save range map
    ms->pushWord("saveCurrentClassDepthAndGraspMaps"); // save current depth map to current class
    ms->pushWord("preAnnotateCenterGrasp"); 
    //ms->pushWord("neutralScanB");  
    { // empty scan
      ms->pushWord(1114150); // prepare for search

      //ms->pushCopies('q',4);
      //ms->pushCopies('a',6);

      ms->pushWord(1048683); // turn on scanning
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord(1114155); // rotate gear

      ms->pushWord("fullRender"); // full render
      ms->pushWord("paintReticles"); // render reticle
      ms->pushWord("shiftIntoGraspGear1"); // change to first gear
      ms->pushWord("drawMapRegisters"); // render register 1
      ms->pushWord("downsampleIrScan"); // load map to register 1
      {
	ms->pushWord(1048678); // target best grasp
	ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
	ms->pushWord("shiftIntoGraspGear1"); // change to first gear
      }
      ms->pushWord(1048630); // find best grasp

      //scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
      ms->pushWord(1114150); // prepare for search

      ms->pushWord(1048683); // turn on scanning
      ms->pushWord("initDepthScan"); // clear scan history
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("shiftIntoGraspGear1"); 
    }

    ms->pushWord("setMovementSpeedMoveEvenFaster");
    //ms->pushWord("fasterRasterScanningSpeed");

    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushCopies("zDown", retractCm); 
    ms->pushWord("comeToHover");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("moveToRegister1");
    ms->pushWord("quarterImpulse");

    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("changeToHeight3"); // change to height 3
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("changeToHeight2"); // change to height 2
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("changeToHeight1"); // change to height 1
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("changeToHeight0"); // change to height 0
    }
  }


  ms->pushWord("fullImpulse");

  ms->pushWord("saveRegister1");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("zUp", 2*retractCm); 
  ms->pushWord("setMovementSpeedMoveFast");
  ms->pushWord("recordGraspZ");

  ms->pushWord('Y'); // pause stack execution
  ms->pushWord("quarterImpulse");
  ms->pushWord(196720); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");
  ms->pushCopies("yDown", 25);
  ms->pushWord("setMovementSpeedMoveFast");
  ms->pushWord("assumeCalibrationPose");
  ms->pushWord("fullImpulse");
}
END_WORD
REGISTER_WORD(ScanObjectFast)

WORD(RecordGraspZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // uses ms->config.currentEEPose instead of ms->config.trueEEPose so that we can set it below the table
  double flushZ = -(ms->config.currentTableZ) + ms->config.pickFlushFactor;
  ms->config.currentGraspZ = ms->config.currentEEPose.pz - flushZ;
  cout << "recordGraspZ flushZ currentGraspZ: " << flushZ << " " << ms->config.currentGraspZ << " " << endl;
}
END_WORD
REGISTER_WORD(RecordGraspZ)

WORD(Start3dGraspAnnotation)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Starting 3d Grasp Annotation" << endl;
  ms->config.bailAfterSynchronic = 1;
  ms->config.bailAfterGradient = 1;

  ms->pushWord("lock3dGraspBase");

  ms->pushWord("gradientServoIfBlueBoxes");
  ms->pushWord("mapClosestBlueBox");
  ms->pushWord("goClassifyBlueBoxes"); 
  ms->pushWord("synchronicServo"); 
  ms->pushWord("visionCycleNoClassify");
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("sampleHeight"); 
  ms->pushWord("setBoundingBoxModeToMapping");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("cruisingSpeed");
}
END_WORD
REGISTER_WORD(Start3dGraspAnnotation)

WORD(Save3dGrasps)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.focusedClass > -1) {
    guard3dGrasps(ms);
    string thisLabelName = ms->config.focusedClassLabel;
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/3dGrasps/";
    string this_grasp_path = dirToMakePath + "3dGrasps.yml";

    mkdir(dirToMakePath.c_str(), 0777);

    FileStorage fsvO;
    cout << "save3dGrasps: Writing: " << this_grasp_path << endl;
    fsvO.open(this_grasp_path, FileStorage::WRITE);

    fsvO << "grasps" << "{";
    {
      int tng = ms->config.class3dGrasps[ms->config.focusedClass].size();
      fsvO << "size" <<  tng;
      fsvO << "graspPoses" << "[" ;
      for (int i = 0; i < tng; i++) {
	ms->config.class3dGrasps[ms->config.focusedClass][i].writeToFileStorage(fsvO);
      }
      fsvO << "]";
    }
    fsvO << "}";

    fsvO.release();

    if (0) {
      guard3dGrasps(ms);
      string thisLabelName = ms->config.focusedClassLabel;
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/3dGrasps/";
      string this_grasp_path = dirToMakePath + "3dGrasps.yml";

      FileStorage fsvI;
      cout << "Reading grasp information from " << this_grasp_path << " ...";
      fsvI.open(this_grasp_path, FileStorage::READ);

      FileNode anode = fsvI["grasps"];
      {
	FileNode bnode = anode["size"];
	FileNodeIterator itb = bnode.begin();
	int tng = *itb;
	ms->config.class3dGrasps[ms->config.focusedClass].resize(0);

	FileNode cnode = anode["graspPoses"];
	FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	int numLoadedPoses = 0;
	for ( ; itc != itc_end; itc++, numLoadedPoses++) {
	  eePose buf;
	  buf.readFromFileNodeIterator(itc);
	  cout << " read pose: " << buf;
	  ms->config.class3dGrasps[ms->config.focusedClass].push_back(buf);
	}
	if (numLoadedPoses != tng) {
	  ROS_ERROR_STREAM("Did not load the expected number of poses.");
	}
	cout << "Expected to load " << tng << " poses, loaded " << numLoadedPoses << " ..." << endl;
      }

      cout << "done.";
    }

  } 
}
END_WORD
REGISTER_WORD(Save3dGrasps)

WORD(Lock3dGraspBase)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if ( (ms->config.bLabels.size() > 0) && (ms->config.pilotClosestBlueBoxNumber != -1) ) {
    ms->config.c3dPoseBase = ms->config.currentEEPose;
    ms->config.c3dPoseBase.pz = -ms->config.currentTableZ;
    cout << endl
	 << "The base for 3d grasp annotation is now locked and you are in zero-G mode." << endl 
	 << "Please move the gripper to a valid grasping pose and use \"add3dGrasp\" to record a grasp point." << endl
	 << "You can record more than one grasp point in a row." << endl
	 << "When you are done, make sure to save to disk and to exit zero-G mode." << endl;
    ms->config.zero_g_toggle = 1;
  } else {
    cout << "Tried to lock c3dPoseBase but failed. Clearing stack." << endl;
    ms->clearStack();
  }
}
END_WORD
REGISTER_WORD(Lock3dGraspBase)

WORD(Add3dGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Adding 3d grasp" << endl;
  eePose this3dGrasp = ms->config.currentEEPose;
  this3dGrasp = this3dGrasp.minusP(ms->config.c3dPoseBase);
  this3dGrasp = this3dGrasp.multQ( ms->config.c3dPoseBase.invQ() );

  int tnc = ms->config.class3dGrasps.size();
  if ( (ms->config.targetClass > 0) && (ms->config.targetClass < tnc) ) {
    ms->config.class3dGrasps[ms->config.targetClass].push_back(this3dGrasp);
  }
}
END_WORD
REGISTER_WORD(Add3dGrasp)

WORD(AssumeCurrent3dGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_backoffDistance = 0.05;
  int t3dGraspIndex = ms->config.current3dGraspIndex;
  eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][t3dGraspIndex];  

  ms->config.currentEEPose.pz = -ms->config.currentTableZ;
  ms->config.currentEEPose = ms->config.currentEEPose.plusP(toApply);
  ms->config.currentEEPose = ms->config.currentEEPose.multQ(toApply);

  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = ms->config.currentEEPose.plusP(p_backoffDistance * localUnitZ);

  int increments = floor(p_backoffDistance / MOVE_FAST); 

  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushCopies("localZDown", increments);
  ms->pushWord("setMovementSpeedMoveFast");
  ms->pushWord("approachSpeed");
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
}
END_WORD
REGISTER_WORD(AssumeCurrent3dGrasp)

WORD(PreAnnotateCenterGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardGraspMemory(ms);
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = 1;
      ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = 0; 
      ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*1] = 1;
      ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*1] = 0; 
      ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*2] = 1;
      ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*2] = 0; 
      ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*3] = 1;
      ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*3] = 0; 
      //ms->config.classGraspMemoryTries1[ms->config.targetClass].at<double>(y,x) = 1;
      //ms->config.classGraspMemoryPicks1[ms->config.targetClass].at<double>(y,x) = 0;
      //ms->config.classGraspMemoryTries2[ms->config.targetClass].at<double>(y,x) = 1;
      //ms->config.classGraspMemoryPicks2[ms->config.targetClass].at<double>(y,x) = 0;
      //ms->config.classGraspMemoryTries3[ms->config.targetClass].at<double>(y,x) = 1;
      //ms->config.classGraspMemoryPicks3[ms->config.targetClass].at<double>(y,x) = 0;
      //ms->config.classGraspMemoryTries4[ms->config.targetClass].at<double>(y,x) = 1;
      //ms->config.classGraspMemoryPicks4[ms->config.targetClass].at<double>(y,x) = 0;
      ms->config.rangeMap[x + y*ms->config.rmWidth] = 0;
      ms->config.rangeMapReg1[x + y*ms->config.rmWidth] = 0;
      //ms->config.classRangeMaps[ms->config.targetClass].at<double>(y,x) = 0;
    } 
  } 
  //ms->config.classGraspMemoryTries1[ms->config.targetClass].at<double>(ms->config.rmHalfWidth,ms->config.rmHalfWidth) = 1;
  //ms->config.classGraspMemoryPicks1[ms->config.targetClass].at<double>(ms->config.rmHalfWidth,ms->config.rmHalfWidth) = 1;
  ms->config.graspMemoryTries[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = 1;
  ms->config.graspMemoryPicks[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = 1; 
  ms->config.rangeMap[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth] = ms->config.currentGraspZ;
  ms->config.rangeMapReg1[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth] = ms->config.currentGraspZ;
  //ms->config.classRangeMaps[ms->config.targetClass].at<double>(ms->config.rmHalfWidth,ms->config.rmHalfWidth) = 1;
}
END_WORD
REGISTER_WORD(PreAnnotateCenterGrasp)

