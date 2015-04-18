

WORD(SetTargetClassToLastLabelLearned)
CODE(1179730)     // capslock + numlock + r
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int i = 0; i < numClasses; i++) {
    if (lastLabelLearned.compare(classLabels[i]) == 0) {
      targetClass = i;
      focusedClass = targetClass;
      focusedClassLabel = classLabels[focusedClass];
      cout << "lastLabelLearned classLabels[targetClass]: " << lastLabelLearned << " " << classLabels[targetClass] << endl;
      changeTargetClass(targetClass);
    }
  }

  pushWord("drawMapRegisters"); // render register 1
  // ATTN 10
  //pushWord(196360); // loadPriorGraspMemory
  //pushWord(1179721); // set graspMemories from classGraspMemories
  switch (currentPickMode) {
  case STATIC_PRIOR:
    {
      pushWord(196360); // loadPriorGraspMemory
    }
    return;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      pushWord(1179721); // set graspMemories from classGraspMemories
      //pushWord(196360); // loadPriorGraspMemory
    }
    break;
  case STATIC_MARGINALS:
    {
      pushWord(1179721); // set graspMemories from classGraspMemories
      //pushWord(196360); // loadPriorGraspMemory
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
  lastLabelLearned = focusedClassLabel;
  cout << "lastLabelLearned: " << lastLabelLearned << endl;
}
END_WORD
REGISTER_WORD(SetLastLabelLearned)

WORD(TrainModels)
CODE(131142)     // capslock + f
virtual void execute(std::shared_ptr<MachineState> ms)       {
  classLabels.resize(0);
  classPoseModels.resize(0);

  pushWord("clearBlueBoxMemories");


  // snoop folders
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s", data_directory.c_str());
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
        classLabels.push_back(thisFileName);
        classPoseModels.push_back("B");
      }
    }
  }

  if ((classLabels.size() != classPoseModels.size()) || (classLabels.size() < 1)) {
    cout << "Label and pose model list size problem. Not proceeding to train." << endl;
    return;
  }

  cout << "Reinitializing and retraining. " << endl;
  for (int i = 0; i < classLabels.size(); i++) {
    cout << classLabels[i] << " " << classPoseModels[i] << endl;
  }

  rewrite_labels = 1;
  retrain_vocab = 1;
  reextract_knn = 1;
  trainOnly = 0;


  // delete things that will be reallocated
  if (bowtrainer)
    delete bowtrainer;
  if (kNN)
    delete kNN;

  for (int i = 0; i < classPosekNNs.size(); i++) {
    if (classPosekNNs[i])
      delete classPosekNNs[i];
  }

  //  detectorsInit() will reset numClasses
  detectorsInit();

  // reset numNewClasses
  newClassCounter = 0;

  // XXX reset anything else
}
END_WORD
REGISTER_WORD(TrainModels)


WORD(VisionCycleNoClassify)
CODE(196721)     // capslock + Q
virtual void execute(std::shared_ptr<MachineState> ms)       {
  pushWord("mapEmptySpace");
  pushWord("goFindBlueBoxes"); // blue boxes
  pushCopies("density", 1); // density
  pushWord("hover"); // blue boxes
}
END_WORD
REGISTER_WORD(VisionCycleNoClassify)

WORD(RecordExampleAsFocusedClass)
CODE(131148)     // capslock + l 
virtual void execute(std::shared_ptr<MachineState> ms)       {
  if ((focusedClass > -1) && (bTops.size() == 1)) {
    string thisLabelName = focusedClassLabel;
    Mat crop = cam_img(cv::Rect(bTops[0].x, bTops[0].y, bBots[0].x-bTops[0].x, bBots[0].y-bTops[0].y));
    char buf[1000];
    string this_crops_path = data_directory + "/" + thisLabelName + "/rgb/";
    sprintf(buf, "%s%s%s_%d.ppm", this_crops_path.c_str(), thisLabelName.c_str(), run_prefix.c_str(), cropCounter);
    imwrite(buf, crop);
    cropCounter++;
  }
}
END_WORD
REGISTER_WORD(RecordExampleAsFocusedClass)

WORD(RecordAllExamplesFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  if ( focusedClass > -1 ) {
    for (int c = 0; c < bTops.size(); c++) {
      string thisLabelName = focusedClassLabel;
      Mat crop = cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      char buf[1000];
      string this_crops_path = data_directory + "/" + thisLabelName + "/rgb/";
      sprintf(buf, "%s%s%s_%d.ppm", this_crops_path.c_str(), thisLabelName.c_str(), run_prefix.c_str(), cropCounter);
      imwrite(buf, crop);
      cropCounter++;
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
  currentEEDeltaRPY.pz += noTheta;
}
END_WORD
REGISTER_WORD(SetRandomOrientationForPhotospin)

WORD(RgbScan)
CODE(131143)      // capslock + g
virtual void execute(std::shared_ptr<MachineState> ms)       {
  // ATTN 16

  pushCopies('e', 5);
  pushCopies('a', 5);
  pushWord(196711); // photospin
  pushCopies('q', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('q', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('d', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('d', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('e', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('e', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('a', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin
  pushCopies('q', 5);
  pushWord(1245246); // uniformly sample height
  pushWord(196711); // photospin

  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushWord(1245246); // uniformly sample height
  pushSpeedSign(MOVE_FAST);
}
END_WORD
REGISTER_WORD(RgbScan)


WORD(PhotoSpin)
CODE(196711)      // capslock + G
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int angleCounter = 0; angleCounter < totalGraspGears; angleCounter++) {
    //pushWord(131148); // save crop as focused class if there is only one
    pushWord("recordAllExamplesFocusedClass");
    pushWord(196721); // vision cycle no classify
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord(1310722); // set random orientation for photospin.
    pushWord(196712); // increment grasp gear
  }
  pushWord("shiftIntoGraspGear1"); // change gear to 1
}
END_WORD
REGISTER_WORD(PhotoSpin)

WORD(SetTargetReticleToTheMaxMappedPosition)
CODE(1048678)  // numlock + f
virtual void execute(std::shared_ptr<MachineState> ms) {
  trX = rmcX + rmDelta*(maxX-rmHalfWidth);
  trY = rmcY + rmDelta*(maxY-rmHalfWidth);
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
  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      for (int rrx = rx*10; rrx < (rx+1)*10; rrx++) {
        for (int rry = ry*10; rry < (ry+1)*10; rry++) {
          if (hiRangeMapMass[rrx + rry*hrmWidth] > 0.0) {
            //if ((hiRangeMap[rrx + rry*hrmWidth] > highestReading) && (hiRangeMap[rrx + rry*hrmWidth] >= readingFloor))
            if ((hiRangeMap[rrx + rry*hrmWidth] > highestEpsilonMassReading) && (hiRangeMapMass[rrx + rry*hrmWidth] > EPSILON))
              highestEpsilonMassReading = hiRangeMap[rrx + rry*hrmWidth];

            if ((hiRangeMap[rrx + rry*hrmWidth] > highestReading) && (hiRangeMapMass[rrx + rry*hrmWidth] > 0))
              highestReading = hiRangeMap[rrx + rry*hrmWidth];
          }
        }
      }
    }
  }


  if (highestReading <= -VERYBIGNUMBER) {
    highestReading = 0;
  }

	
  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      double thisSum = 0;
      double numSamples = 0;
      for (int rrx = rx*10; rrx < (rx+1)*10; rrx++) {
        for (int rry = ry*10; rry < (ry+1)*10; rry++) {
          numSamples += 1.0;
          if (hiRangeMapMass[rrx + rry*hrmWidth] > 0.0) {
            thisSum += hiRangeMap[rrx + rry*hrmWidth];
          } else {
            thisSum += 0;
          }
        }
      }
      rangeMapReg1[rx + ry*rmWidth] = thisSum/numSamples;
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

  eepReg2 = rssPose;
  eepReg4 = rssPose;

  // so that closest servoing doesn't go into gradient servoing.
  targetClass = -1;


  // this automatically changes learning mode
          
  if (0) {
    pushWord("beginHeightLearning"); // begin bounding box learning

    pushWord("changeToHeight1"); // change to height 1
    pushWord("shiftIntoGraspGear1"); // change to first gear
  }

  if (1) {
    pushWord("saveLearnedModels");
    pushWord("loadPriorGraspMemoryAnalytic");
    // set target class to the lastLabelLearned 
    pushWord(1179730);
    pushWord(131142); // reinitialize and retrain everything
  }

  // set lastLabelLearned
  pushWord(1179732);

  if (0) {
    pushWord(131143); // 72 way scan
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord(131143); // 72 way scan
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  }

  pushWord("scanCentered"); // 72 way scan

  // this is a good time to remove a contrast agent
  //pushWord('Y'); // pause stack execution
  //pushCopies("beep", 15); // beep
	  
  { // do density and gradient, save gradient, do medium scan in two directions, save range map
    pushSpeedSign(MOVE_FAST);
    pushWord("saveCurrentClassDepthAndGraspMaps"); // save current depth map to current class
    pushWord("neutralScan"); // neutral scan 
    pushWord('Y'); // pause stack execution
    pushCopies("beep", 15); // beep
    pushSpeedSign(MOVE_FAST);

    pushWord("changeToHeight1"); // change to height 1

    {
      pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      pushWord("gradientServoPrep");
      pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      pushWord("changeToHeight3"); // change to height 3
    }
    {
      pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      pushWord("gradientServoPrep");
      pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      pushWord("changeToHeight2"); // change to height 2
    }
    {
      pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      pushWord("gradientServoPrep");
      pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      pushWord("changeToHeight1"); // change to height 1
    }
    {
      pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      pushWord("gradientServoPrep");
      pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      pushWord("changeToHeight0"); // change to height 0
    }
  }

  // ATTN 3
  // start NO bag routine
  pushWord(196720); //  make a new class

  pushWord(131139); // synchronic servo don't take closest
  pushWord(131156); // synchronic servo
  pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
  pushWord("visionCycle"); // vision cycle

  pushWord('Y'); // pause stack execution
  pushCopies("beep", 15); // beep

  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushWord("shiftIntoGraspGear1"); // change to first gear
  pushWord(1245219); // change to height 2
  pushSpeedSign(MOVE_FAST);
  //pushWord(196672); // go to wholeFoodsCounter1

  pushWord(1179735); // change to counter table
  pushWord("shiftIntoGraspGear1"); // change to first gear
  pushWord('k'); // open gripper
}
END_WORD
REGISTER_WORD(ScanObject)


WORD(PrepareForSearch)
CODE(1114150)     // numlock + &
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.px = rmcX + drX;
  currentEEPose.py = rmcY + drY;
}
END_WORD
REGISTER_WORD(PrepareForSearch)
 

WORD(TurnOnRecordRangeMap)
CODE(1048683) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  recordRangeMap = 1;
}
END_WORD
REGISTER_WORD(TurnOnRecordRangeMap)

WORD(SetRangeMapCenterFromCurrentEEPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Set rmcX and rmcY from currentEEPose." << endl;
  rmcX = currentEEPose.px;
  rmcY = currentEEPose.py;
  //rmcZ = currentEEPose.pz - eeRange;
}
END_WORD
REGISTER_WORD(SetRangeMapCenterFromCurrentEEPose)

WORD(InitDepthScan)
CODE(1048695) // numlock + w
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Set rmcX and rmcY. Resetting maps. " << rmcX << " " << trueEEPose.position.x << endl;
  rmcX = trueEEPose.position.x;
  rmcY = trueEEPose.position.y;
  rmcZ = trueEEPose.position.z - eeRange;
  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      rangeMap[rx + ry*rmWidth] = 0;
      rangeMapReg1[rx + ry*rmWidth] = 0;
      // ATTN 17
      //rangeMapReg2[rx + ry*rmWidth] = 0;
      rangeMapMass[rx + ry*rmWidth] = 0;
      rangeMapAccumulator[rx + ry*rmWidth] = 0;
    }
  }
  {
    cv::Scalar backColor(128,0,0);
    cv::Point outTop = cv::Point(0,0);
    cv::Point outBot = cv::Point(rmiWidth,rmiHeight);
    Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop = backColor;
  }
  for (int rx = 0; rx < hrmWidth; rx++) {
    for (int ry = 0; ry < hrmWidth; ry++) {
      hiRangeMap[rx + ry*hrmWidth] = 0;
      hiRangeMapReg1[rx + ry*hrmWidth] = 0;
      hiRangeMapReg2[rx + ry*hrmWidth] = 0;
      hiRangeMapMass[rx + ry*hrmWidth] = 0;
      hiRangeMapAccumulator[rx + ry*hrmWidth] = 0;
    }
  }
  {
    cv::Scalar backColor(128,0,0);
    cv::Point outTop = cv::Point(0,0);
    cv::Point outBot = cv::Point(hrmiWidth,hrmiHeight);
    Mat vCrop = hiRangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop = backColor;
  }
  for (int h = 0; h < hrmWidth; h++) {
    for (int i = 0; i < hrmWidth; i++) {
      hiColorRangeMapMass[h + i*hrmWidth] = 0;
      for (int j = 0; j < 3; j++) {
        hiColorRangeMapAccumulator[h + i*hrmWidth + j*hrmWidth*hrmWidth] = 0;
      }
    }
  }
  for (int pz = 0; pz < vmWidth; pz++) {
    for (int py = 0; py < vmWidth; py++) {
      for (int px = 0; px < vmWidth; px++) {
        volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
        volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
        volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
        vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
        for (int pc = 0; pc < 3; pc++) {
          vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + pc*vmWidth*vmWidth*vmWidth] = 0;
        }
      }
    }
  }
  {
    cv::Scalar backColor(128,0,0);
    cv::Point outTop = cv::Point(0,0);
    cv::Point outBot = cv::Point(hiColorRangemapImage.cols,hiColorRangemapImage.rows);
    Mat vCrop = hiColorRangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop = backColor;
  }
}
END_WORD
REGISTER_WORD(InitDepthScan)




WORD(NeutralScan)
CODE(1048622) // numlock + .
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushWord("cruisingSpeed");
  pushWord("neutralScanA");
  pushWord("rasterScanningSpeed");
}
END_WORD
REGISTER_WORD(NeutralScan)

WORD(NeutralScanA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering neutral scan." << endl;
  double lineSpeed = MOVE_FAST;//MOVE_MEDIUM;//MOVE_FAST;
  double betweenSpeed = MOVE_FAST;//MOVE_MEDIUM;//MOVE_FAST;

  scanXdirection(lineSpeed, betweenSpeed); // load scan program
  pushWord(1114150); // prepare for search

  pushCopies('q',4);
  pushCopies('a',6);

  pushWord(1048683); // turn on scanning
  pushNoOps(60);
  pushWord(1114155); // rotate gear

  pushWord("fullRender"); // full render
  pushWord("paintReticles"); // render reticle
  pushWord("shiftIntoGraspGear1"); // change to first gear
  pushWord("drawMapRegisters"); // render register 1
  pushWord("downsampleIrScan"); // load map to register 1
  {
    pushWord(1048678); // target best grasp
    pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    pushWord("shiftIntoGraspGear1"); // change to first gear
  }
  pushWord(1048630); // find best grasp

  scanXdirection(lineSpeed, betweenSpeed); // load scan program
  pushWord(1114150); // prepare for search

  pushWord(1048683); // turn on scanning
  pushWord("initDepthScan"); // clear scan history
}
END_WORD
REGISTER_WORD(NeutralScanA)

WORD(SaveAerialGradientMap)
CODE(196730)      // capslock + Z
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
        
  cout << "save aerial gradient ";
  if ((focusedClass > -1) && (frameGraySobel.rows >1) && (frameGraySobel.cols > 1)) {
    string thisLabelName = focusedClassLabel;

    char buf[1000];
    string dirToMakePath = data_directory + "/" + thisLabelName + "/aerialGradient/";
    string this_range_path;

    // ATTN 16
    switch (currentThompsonHeightIdx) {
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

    //int hbb = pilotTargetBlueBoxNumber;
    //int hbb = 0;

    int topCornerX = reticle.px - (aerialGradientReticleWidth/2);
    int topCornerY = reticle.py - (aerialGradientReticleWidth/2);
    int crows = aerialGradientReticleWidth;
    int ccols = aerialGradientReticleWidth;

    //int crows = bBots[hbb].y - bTops[hbb].y;
    //int ccols = bBots[hbb].x - bTops[hbb].x;
    int maxDim = max(crows, ccols);
    int tRy = (maxDim-crows)/2;
    int tRx = (maxDim-ccols)/2;
    Mat gCrop(maxDim, maxDim, frameGraySobel.type());

    cout << "crows ccols: " << crows << " " << ccols << " ";

    for (int x = 0; x < maxDim; x++) {
      for (int y = 0; y < maxDim; y++) {
        int tx = x - tRx;
        int ty = y - tRy;
        int tCtx = topCornerX + tx;
        int tCty = topCornerY + ty;
        if ( (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) &&
             (tCtx > 0) && (tCty > 0) && (tCtx < imW) && (tCty < imH) ) {
          //gCrop.at<double>(y, x) = frameGraySobel.at<double>(bTops[hbb].y + ty, bTops[hbb].x + tx);
          gCrop.at<double>(y, x) = frameGraySobel.at<double>(tCty, tCtx);
        } else {
          gCrop.at<double>(y, x) = 0.0;
        }
      }
    }
  
    cout << "about to resize" << endl;

    Size toBecome(aerialGradientWidth, aerialGradientWidth);
    cv::resize(gCrop, gCrop, toBecome);


    FileStorage fsvO;
    cout << "capslock + Z: Writing: " << this_range_path << endl;

    fsvO.open(this_range_path, FileStorage::WRITE);

    // ATTN 16
    switch (currentThompsonHeightIdx) {
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

    lastAerialGradient = gCrop;
    fsvO.release();
  } 
}
END_WORD
REGISTER_WORD(SaveAerialGradientMap)

WORD(InitializeAndFocusOnNewClass)
CODE(196720)     // capslock + P
virtual void execute(std::shared_ptr<MachineState> ms) {
  focusedClass = numClasses+newClassCounter;
  char buf[1024];
  sprintf(buf, "autoClass%d_%s", focusedClass, left_or_right_arm.c_str());
  string thisLabelName(buf);
  focusedClassLabel = thisLabelName;
  classLabels.push_back(thisLabelName);
  string dirToMakePath = data_directory + "/" + thisLabelName + "/";
  mkdir(dirToMakePath.c_str(), 0777);
  string rgbDirToMakePath = data_directory + "/" + thisLabelName + "/rgb";
  mkdir(rgbDirToMakePath.c_str(), 0777);
  newClassCounter++;
}
END_WORD
REGISTER_WORD(InitializeAndFocusOnNewClass)

WORD(SaveCurrentClassDepthAndGraspMaps)
CODE(196705) // capslock + A
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (focusedClass > -1) {
    // initialize this if we need to
    guardGraspMemory();
    guardHeightMemory();

    string thisLabelName = focusedClassLabel;

    char buf[1000];
    string dirToMakePath = data_directory + "/" + thisLabelName + "/ir2D/";
    string this_range_path = dirToMakePath + "xyzRange.yml";

    Mat rangeMapTemp(rmWidth, rmWidth, CV_64F);
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
	rangeMapTemp.at<double>(y,x) = rangeMapReg1[x + y*rmWidth];
      } 
    } 

    mkdir(dirToMakePath.c_str(), 0777);

    FileStorage fsvO;
    cout << "capslock + A: Writing: " << this_range_path << endl;
    fsvO.open(this_range_path, FileStorage::WRITE);
    fsvO << "rangeMap" << rangeMapTemp;
    copyGraspMemoryTriesToClassGraspMemoryTries();
    fsvO << "graspMemoryTries1" << classGraspMemoryTries1[focusedClass];
    fsvO << "graspMemoryPicks1" << classGraspMemoryPicks1[focusedClass];
    fsvO << "graspMemoryTries2" << classGraspMemoryTries2[focusedClass];
    fsvO << "graspMemoryPicks2" << classGraspMemoryPicks2[focusedClass];
    fsvO << "graspMemoryTries3" << classGraspMemoryTries3[focusedClass];
    fsvO << "graspMemoryPicks3" << classGraspMemoryPicks3[focusedClass];
    fsvO << "graspMemoryTries4" << classGraspMemoryTries4[focusedClass];
    fsvO << "graspMemoryPicks4" << classGraspMemoryPicks4[focusedClass];

    copyHeightMemoryTriesToClassHeightMemoryTries();
    fsvO << "heightMemoryTries" << classHeightMemoryTries[focusedClass];
    fsvO << "heightMemoryPicks" << classHeightMemoryPicks[focusedClass];

    lastRangeMap = rangeMapTemp;
    fsvO.release();
  } 
}
END_WORD
REGISTER_WORD(SaveCurrentClassDepthAndGraspMaps)

WORD(ScanCentered)
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushSpeedSign(MOVE_FAST);
  pushWord("rgbScan");
  pushWord("rgbScan");
  pushWord("rgbScan");
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushWord("synchronicServo"); 
  pushWord("visionCycleNoClassify");
  pushWord("synchronicServoTakeClosest");
  pushWord("fillClearanceMap");
  pushWord("loadIkMap");
  currentBoundingBoxMode = MAPPING;
  bDelta = 0.001;
}
END_WORD
REGISTER_WORD(ScanCentered)

WORD(SetTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
  firstTableHeightTime = ros::Time::now();
  mostRecentUntabledZLastValue = mostRecentUntabledZ;
  pushWord("setTableA");
}
END_WORD
REGISTER_WORD(SetTable)

WORD(SetTableA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ros::Time thisTableHeightTime = ros::Time::now();

  mostRecentUntabledZLastValue = ((1.0-mostRecentUntabledZDecay)*mostRecentUntabledZ) + (mostRecentUntabledZDecay*mostRecentUntabledZLastValue);

  oneTable = mostRecentUntabledZLastValue;
  rightTableZ = oneTable;
  leftTableZ = oneTable;
  bagTableZ = oneTable;
  counterTableZ = oneTable;
  pantryTableZ = oneTable;
  currentTableZ = oneTable;

  if ( fabs(thisTableHeightTime.sec - firstTableHeightTime.sec) > mostRecentUntabledZWait) {
    // do nothing
  } else {
    double utZDelta = fabs(mostRecentUntabledZ - mostRecentUntabledZLastValue);
    endThisStackCollapse = 1;
    pushWord("setTableA");
    cout << "Looks like the table reading hasn't steadied to within the wait of " << mostRecentUntabledZWait << " ." << endl;
    cout << "  current, last, delta: " << mostRecentUntabledZ << " " << mostRecentUntabledZLastValue << " " << utZDelta << endl;
  } 
}
END_WORD
REGISTER_WORD(SetTableA)

WORD(SetIROffset)
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushWord("setIROffsetA");
  pushWord("cruisingSpeed");
  pushWord("neutralScanA");
  pushWord("iRCalibrationSpeed");
}
END_WORD
REGISTER_WORD(SetIROffset)

WORD(SetIROffsetA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // find the maximum in the map
  // find the coordinate of the maximum
  // compare the coordinate to the root position
  // adjust offset
  // if adjustment was large, recommend running again
}
END_WORD
REGISTER_WORD(SetIROffsetA)

WORD(SetCameraReticles)
virtual void execute(std::shared_ptr<MachineState> ms) {

  // setReticlesA
  // go to height
  // setReticlesA
  // go to height

}
END_WORD
REGISTER_WORD(SetCameraReticles)

WORD(SetCameraReticlesA)
virtual void execute(std::shared_ptr<MachineState> ms) {

}
END_WORD
REGISTER_WORD(SetCameraReticlesA)

WORD(MoveCropToCenter)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  cropUpperLeftCorner.px = 320;
  cropUpperLeftCorner.py = 200;

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = cropUpperLeftCorner.py;
  int testResult = cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(MoveCropToCenter)

WORD(MoveCropToCenterVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  double Vx = vanishingPointReticle.px - (imW/2);
  double Vy = vanishingPointReticle.py - (imH/2);

  cropUpperLeftCorner.px += Vx;
  cropUpperLeftCorner.py += Vy;
  vanishingPointReticle.px -= Vx;
  vanishingPointReticle.py -= Vy;

  cout << "MoveCropToCenterVanishingPoint Vx Vy: " << Vx << " " << Vy << endl;

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = cropUpperLeftCorner.py;
  int testResult = cameraClient.call(ocMessage);
  //cout << "centerVanishingPoint testResult: " << testResult << endl;
  //cout << ocMessage.response.name << endl;
  //cout << ocMessage.response.name << " " << ocMessage.response.settings.controls.size() << endl;

  //cout << "MoveCropToCenterVanishingPoint moving region of interest and vanishing point. Recalibrate vanishing point, height reticles, and magnification factors." << endl;
}
END_WORD
REGISTER_WORD(MoveCropToCenterVanishingPoint)

WORD(MoveToSetVanishingPointHeightLow)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.pz = minHeight - currentTableZ;
}
END_WORD
REGISTER_WORD(MoveToSetVanishingPointHeightLow)

WORD(MoveToSetVanishingPointHeightHigh)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose.pz = maxHeight - currentTableZ;
}
END_WORD
REGISTER_WORD(MoveToSetVanishingPointHeightHigh)

WORD(SetVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {

  setVanishingPointIterations = 0;
  // go low, wait
  pushWord("setVanishingPointA");
  // is darkest point in current vp? loop here until it is so then rise and go to B
  pushWord("setVanishingPointPrep");
}
END_WORD
REGISTER_WORD(SetVanishingPoint)

WORD(SetVanishingPointPrep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushWord("darkServo");
  pushWord("waitUntilAtCurrentPosition");
  pushWord("moveToSetVanishingPointHeightLow");
}
END_WORD
REGISTER_WORD(SetVanishingPointPrep)

WORD(SetVanishingPointA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  
  setVanishingPointIterations++;
  currentEEPose.pz = -currentTableZ + 0.4;
  pushWord("setVanishingPointB");
  pushWord("accumulatedDensity");
  pushCopies("waitUntilImageCallbackReceived", 10);
  pushWord("resetAccumulatedDensity");
  pushWord("comeToStop");
  pushWord("waitUntilAtCurrentPosition");
  pushWord("moveToSetVanishingPointHeightHigh");
}
END_WORD
REGISTER_WORD(SetVanishingPointA)

WORD(SetVanishingPointB)
virtual void execute(std::shared_ptr<MachineState> ms) {

  // where is the darkest point now? did it move? move vp to darkest point and possibly run again
  int darkX = 0;
  int darkY = 0;
  findDarkness(&darkX, &darkY);

  int Px = darkX - vanishingPointReticle.px;
  int Py = darkY - vanishingPointReticle.py;

  vanishingPointReticle.px = darkX;
  vanishingPointReticle.py = darkY;
  
  cout << "setVanishingPoint Px Py: " << Px << " " << Py << endl;

  if (setVanishingPointIterations > setVanishingPointTimeout) {
    cout << "setVanishingPoint timed out, continuing..." << endl;
  }

  if ((fabs(Px) < setVanishingPointPixelThresh) && (fabs(Py) < setVanishingPointPixelThresh)) {
    cout << "vanishing point set, continuing." << endl;
  } else {
    cout << "vanishing point not set, adjusting more. " << setVanishingPointIterations << " " << setVanishingPointTimeout << endl;
    pushWord("setVanishingPointA");
    pushWord("setVanishingPointPrep");
  }
}
END_WORD
REGISTER_WORD(SetVanishingPointB)

WORD(SetMagnification)
virtual void execute(std::shared_ptr<MachineState> ms) {

  // move back
  // adjust until close	
  // move back over then down 
  // adjust until close	
  // move over 
  // go to height

  // move back
  // adjust until close	
  // move back over then down 
  // adjust until close	
  // move over 
  // go to height
}
END_WORD
REGISTER_WORD(SetMagnification)

WORD(SetMagnificationA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // adjust until close	
}
END_WORD
REGISTER_WORD(SetMagnificationA)

WORD(SetGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Program paused; please present the first contrast medium." << endl;
  pushWord("setGripperMaskA"); 
  pushWord("accumulatedDensity");
  pushCopies("waitUntilImageCallbackReceived", 10);
  pushWord("resetAccumulatedDensity");
  pushWord("comeToStop");
  pushWord("pauseStackExecution"); 
}
END_WORD
REGISTER_WORD(SetGripperMask)

WORD(SetGripperMaskA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Program paused; please present the second contrast medium." << endl;
  pushWord("setGripperMaskB"); 
  pushWord("accumulatedDensity");
  pushCopies("waitUntilImageCallbackReceived", 10);
  pushWord("resetAccumulatedDensity");
  pushWord("comeToStop");
  pushWord("pauseStackExecution"); 

  gripperMaskFirstContrast = accumulatedImage.clone();
  gripperMaskSecondContrast = gripperMaskFirstContrast.clone();
  gripperMask.create(gripperMaskFirstContrast.size(), CV_8U);

  Size sz = gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      gripperMaskFirstContrast.at<Vec3d>(y,x)[0] = (accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      gripperMaskFirstContrast.at<Vec3d>(y,x)[1] = (accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      gripperMaskFirstContrast.at<Vec3d>(y,x)[2] = (accumulatedImage.at<Vec3d>(y,x)[2] / denom);
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskA)

WORD(SetGripperMaskB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Thank you. Don't forget to save your mask!" << endl;

  Size sz = gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  int dilationPixels = 10;
  double baseThresh = 10;
  double multiThresh = 3*baseThresh*baseThresh;

  cout << "  multiThresh dilationPixels: " << multiThresh << " " << dilationPixels << endl;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      gripperMaskSecondContrast.at<Vec3d>(y,x)[0] = (accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      gripperMaskSecondContrast.at<Vec3d>(y,x)[1] = (accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      gripperMaskSecondContrast.at<Vec3d>(y,x)[2] = (accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      double maskDiff = 
      ((gripperMaskFirstContrast.at<Vec3d>(y,x)[0] - gripperMaskSecondContrast.at<Vec3d>(y,x)[0])*
      (gripperMaskFirstContrast.at<Vec3d>(y,x)[0] - gripperMaskSecondContrast.at<Vec3d>(y,x)[0])) +
      ((gripperMaskFirstContrast.at<Vec3d>(y,x)[1] - gripperMaskSecondContrast.at<Vec3d>(y,x)[1])*
      (gripperMaskFirstContrast.at<Vec3d>(y,x)[1] - gripperMaskSecondContrast.at<Vec3d>(y,x)[1])) +
      ((gripperMaskFirstContrast.at<Vec3d>(y,x)[2] - gripperMaskSecondContrast.at<Vec3d>(y,x)[2])*
      (gripperMaskFirstContrast.at<Vec3d>(y,x)[2] - gripperMaskSecondContrast.at<Vec3d>(y,x)[2]));

      if(maskDiff < 1000) {
	cout << multiThresh << " " << maskDiff << endl;
      }

      if (maskDiff > multiThresh) {
	gripperMask.at<uchar>(y,x) = 1;
      } else {
	gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }

  Mat tmpMask = gripperMask.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) == 0) {
	int xmin = max(0, x - dilationPixels);
	int xmax = min(imW-1, x + dilationPixels);
	int ymin = max(0, y - dilationPixels);
	int ymax = min(imH-1, y + dilationPixels);
	for (int xp = xmin; xp < xmax; xp++) {
	  for (int yp = ymin; yp < ymax; yp++) {
	    gripperMask.at<uchar>(yp,xp) = 0;
	  }
	}
      }
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskB)

WORD(LoadGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename = data_directory + "/" + left_or_right_arm + "GripperMask.bmp";
  cout << "Loading gripper mask from " << filename << endl;
  Mat tmpMask = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  cout << "  tmpMask.type() tmpMask.size(): " << tmpMask.type() << " " << tmpMask.size() << endl;

  gripperMask.create(tmpMask.size(), CV_8U);
  Size sz = gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) > 0) {
	gripperMask.at<uchar>(y,x) = 1;
      } else {
	gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }
  
}
END_WORD
REGISTER_WORD(LoadGripperMask)

WORD(SaveGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename = data_directory + "/" + left_or_right_arm + "GripperMask.bmp";
  cout << "Saving gripper mask to " << filename << endl;
  imwrite(filename, 255*gripperMask);
}
END_WORD
REGISTER_WORD(SaveGripperMask)
