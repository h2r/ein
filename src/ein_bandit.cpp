WORD(SaveLearnedModels)
CODE(1245281)     // capslock + numlock + A
virtual void execute()       {
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
        rangeMapTemp.at<double>(y,x) = classRangeMaps[focusedClass].at<double>(y,x);
      } 
    } 

    mkdir(dirToMakePath.c_str(), 0777);

    FileStorage fsvO;
    cout << "capslock + numlock + A: Writing: " << this_range_path << endl;
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



WORD(SetRandomPositionAndOrientationForHeightLearning)
CODE( 1179687)     // capslock + numlock + '
virtual void execute() {
  double noX = bbLearnPerturbScale * ((drand48() - 0.5) * 2.0);
  double noY = bbLearnPerturbScale * ((drand48() - 0.5) * 2.0);
  noX = noX + (((noX > 0) - 0.5) * 2) * bbLearnPerturbBias;
  noY = noY + (((noY > 0) - 0.5) * 2) * bbLearnPerturbBias;
  double noTheta = 3.1415926 * ((drand48() - 0.5) * 2.0);
  
  currentEEPose.px += noX;
  currentEEPose.py += noY;
  currentEEPose.oz += noTheta;
}
END_WORD

WORD(BeginHeightLearning)
CODE(1245242)     // capslock + numlock + :
virtual void execute()       {
  eepReg3 = rssPose;
  heightAttemptCounter = 0;
  heightSuccessCounter = 0;
  thompsonPickHaltFlag = 0;
  thompsonHeightHaltFlag = 0;
  pushWord(1179707); // continue height learning
  pushWord(65568+3); // record register 3

  pushWord(131139); // synchronic servo don't take closest
  pushWord("synchronicServo"); // synchronic servo
  pushWord(196707); // synchronic servo take closest
  pushWord("visionCycle"); // vision cycle
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  { // prepare to servo
    //currentEEPose.pz = wholeFoodsCounter1.pz+.1;
    pushWord(1245248); // change to height 1
  }
  //pushWord(1179723); // change height inference mode to LEARNING_SAMPLING
  pushWord('3'); // recall register 3
}
END_WORD


WORD(ContinueHeightLearning)
CODE(1179707)     // capslock + numlock + ;
  virtual void execute()       {
  cout << "continuing bounding box learning with currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
  synServoLockFrames = 0;
  currentGradientServoIterations = 0;

  // ATTN 16
  // ATTN 19
  if (thompsonHardCutoff) {
    if (heightAttemptCounter < thompsonTries - 1) {
      // push this program 
      pushWord(1179707); // begin bounding box learning
    } else {
      pushCopies("beep", 15); // beep
    }
  }
  if (thompsonAdaptiveCutoff) {
    if ( (thompsonHeightHaltFlag) ||
         (heightAttemptCounter >= thompsonTries - 1) ) {
      cout << "Clearing call stack. thompsonHeightHaltFlag = " << thompsonHeightHaltFlag << 
        " and we did " << heightAttemptCounter << " tries." << endl;
      clearStack();
      pushCopies("beep", 15); // beep
      return;
    } else {
      if (heightAttemptCounter < thompsonTries - 1) {
        // push this program 
        pushWord(1179707); // begin bounding box learning
      } else {
        cout << "Clearing call stack. thompsonHeightHaltFlag = " << thompsonHeightHaltFlag << 
          " and we did " << heightAttemptCounter << " tries." << endl;
        clearStack();
      }
    }
  }

  pushWord("recordHeightLearnTrial"); 

  pushWord("synchronicServoDoNotTakeClosest"); 
  pushWord("synchronicServo"); // synchronic servo
  pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
  pushWord("visionCycle"); // vision cycle
  //pushWord(1179695); // check to see if bounding box is unique (early outting if not)
  pushWord("visionCycle"); // vision cycle
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushWord("setRandomPositionAndOrientationForHeightLearning"); // set random position for bblearn

  pushWord(65568+4); // record register 4

  // servo to object, which will early out if it times out 
  pushWord(131139); // synchronic servo don't take closest
  pushWord("synchronicServo"); // synchronic servo
  pushWord(196707); // synchronic servo take closest
  pushWord("visionCycle"); // vision cycle
  //pushWord(1179695); // check to see if bounding box is unique (early outting if not)
  pushWord("visionCycle"); // vision cycle
  pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  pushWord("setRandomPositionAndOrientationForHeightLearning"); // set random position for bblearn

  pushWord("sampleHeight"); // sample height

  pushWord("changeToPantryTable"); // change to pantry table
  pushWord('3'); // recall register 3
}
END_WORD

WORD(RecordHeightLearnTrial)
CODE(1179694)     // capslock + numlock + .
  virtual void execute()       {
  // Distances for the eraser
  //0.04, 2.57e-05, 0.0005, 0.0009, 0.007, 0.0006
  // ATTN 17
  double distance = squareDistanceEEPose(currentEEPose, eepReg4);
  cout << "cartesian distance from start: " << sqrt(distance) << endl;
  cout << "bbLearnThresh: " << bbLearnThresh << endl;
  if (distance < bbLearnThresh*bbLearnThresh) {
    Quaternionf q1(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
    Quaternionf q2(eepReg4.qw, eepReg4.qx, eepReg4.qy, eepReg4.qz);
    double quaternionDistance = unsignedQuaternionDistance(q1, q2);
    cout << "quat distance from start: " << quaternionDistance << endl;
    cout << "bbQuatThresh: " << bbQuatThresh << endl;
    if (quaternionDistance < bbQuatThresh)
      recordBoundingBoxSuccess();
    else
      recordBoundingBoxFailure();
  } else {
    recordBoundingBoxFailure();
  }
}
END_WORD




WORD(LoadSampledGraspMemory)
CODE(131117)     // capslock + -
virtual void execute() {
  loadSampledGraspMemory();
  drawMapRegisters();
}
END_WORD

WORD(LoadMarginalGraspMemory)
CODE(131133)     // capslock + =
virtual void execute() {
  loadMarginalGraspMemory();
  drawMapRegisters();
}
END_WORD

WORD(LoadPriorGraspMemoryAnalytic)
CODE(196360)     // capslock + backspace
virtual void execute() {
  loadPriorGraspMemory(ANALYTIC_PRIOR);
  copyGraspMemoryTriesToClassGraspMemoryTries();
  loadMarginalGraspMemory();
  
  // shows mus before we converted them to alphas and betas,
  // smoothing the values based on eccentricity.  
  //copyGraspMemoryRegister(graspMemoryReg1, graspMemorySample);
  
  drawMapRegisters();
  cout << "class " << classLabels[targetClass] << " number ";
}
END_WORD


WORD(LoadPriorGraspMemoryUniform)
CODE(261896)     // capslock + shift + backspace
virtual void execute() {
  loadPriorGraspMemory(UNIFORM_PRIOR);
  copyGraspMemoryTriesToClassGraspMemoryTries();
  loadMarginalGraspMemory();
  
  // shows mus before we converted them to alphas and betas,
  // smoothing the values based on eccentricity.  
  //copyGraspMemoryRegister(graspMemoryReg1, graspMemorySample);
  
  drawMapRegisters();
  cout << "class " << classLabels[targetClass] << " number ";
}
END_WORD

WORD(LoadSampledHeightMemory)
CODE(1179693)     // capslock + numlock + -
virtual void execute() {
  loadSampledHeightMemory();
  drawHeightMemorySample();
}
END_WORD

WORD(LoadMarginalHeightMemory)
CODE(1179709)     // capslock + numlock + =
virtual void execute() {
  loadMarginalHeightMemory();
  drawHeightMemorySample();
}
END_WORD

WORD(LoadPriorHeightMemoryAnalytic)
CODE(1244936)     // capslock + numlock + backspace
virtual void execute() {
  loadPriorHeightMemory(ANALYTIC_PRIOR);
  copyHeightMemoryTriesToClassHeightMemoryTries();
  loadMarginalHeightMemory();
  drawHeightMemorySample();
}
END_WORD


WORD(LoadPriorHeightMemoryUniform)
CODE(1310472)     // capslock + numlock + shift + backspace
virtual void execute() {
  loadPriorHeightMemory(UNIFORM_PRIOR);
  copyHeightMemoryTriesToClassHeightMemoryTries();
  loadMarginalHeightMemory();
  drawHeightMemorySample();
}
END_WORD



WORD(SetPickModeToStaticPrior)
CODE(1179731)     // capslock + numlock + s
virtual void execute() {
  currentPickMode = STATIC_PRIOR;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD

WORD(SetPickModeToLearningSampling)
CODE(1179716)     // capslock + numlock + d
virtual void execute() {
  currentPickMode = LEARNING_SAMPLING;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD

WORD(SetPickModeToLearningAlgorithmC)
CODE(1245284) // capslock + numlock + D
virtual void execute() {
  currentPickMode = LEARNING_ALGORITHMC;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD

WORD(SetPickModeToStaticMarginals)
CODE(1179718)     // capslock + numlock + f
virtual void execute() {
  currentPickMode = STATIC_MARGINALS;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD


WORD(SetBoundingBoxModeToMapping)
virtual void execute() {
  currentBoundingBoxMode = MAPPING;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD

WORD(SetBoundingBoxModeToStaticPrior)
CODE(1179722)     // capslock + numlock + j
virtual void execute() {
  currentBoundingBoxMode = STATIC_PRIOR;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD

WORD(SetBoundingBoxModeToLearningSampling)
CODE(1179723)     // capslock + numlock + k
virtual void execute() {
  currentBoundingBoxMode = LEARNING_SAMPLING;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD

WORD(SetBoundingBoxModeToLearningAlgorithmC)
CODE(1245291)     // capslock + numlock + K
virtual void execute() {
  currentBoundingBoxMode = LEARNING_ALGORITHMC;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD

WORD(SetBoundingBoxModeToStaticMarginals)
CODE(1179724)     // capslock + numlock + l
virtual void execute() {
  currentBoundingBoxMode = STATIC_MARGINALS;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD

WORD(UniformlySampleHeight)
CODE(1245246)      // capslock + numlock + >
virtual void execute() {
  int thisRandThompsonHeight = lrand48() % hmWidth;
  if (currentBoundingBoxMode == MAPPING) {
    thisRandThompsonHeight = mappingHeightIdx;
    cout << "UniformlySampleHeight going to mappingHeightIdx: " << mappingHeightIdx << endl;
  }
  currentThompsonHeight = convertHeightIdxToGlobalZ(thisRandThompsonHeight);
  currentThompsonHeightIdx = thisRandThompsonHeight;
  currentEEPose.pz = currentThompsonHeight;
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD




WORD(SetGraspMemoriesFromClassGraspMemories)  
// capslock + numlock + i
CODE(1179721)
virtual void execute() {
  copyClassGraspMemoryTriesToGraspMemoryTries();
}
END_WORD


WORD(CopyClassGraspMemoryTriesToGraspMemoryTries)    
// capslock + numlock + i
CODE(1179721)
virtual void execute()
{
  copyClassGraspMemoryTriesToGraspMemoryTries();
}
END_WORD



WORD(SetHeightMemoriesFromClassHeightMemories)
// capslock + numlock + I 
CODE(1245289)
virtual void execute()
{
        cout << "Loading height memories." << endl;
        if ((classHeightMemoryTries[targetClass].rows > 1) && (classHeightMemoryPicks[targetClass].cols == 1)) {
          cout << "targetClass: " << targetClass << " " << classLabels[targetClass] << endl;
          for (int i = 0; i < hmWidth; i++) {
            heightMemoryPicks[i] = classHeightMemoryPicks[targetClass].at<double>(i, 0);
            heightMemoryTries[i] = classHeightMemoryTries[targetClass].at<double>(i, 0);
            cout << "picks: " << heightMemoryPicks[i] << endl;
            cout << "tries: " << heightMemoryTries[i] << endl;
          }
        } else {
	  cout << "Whoops, tried to set height memories but they don't exist for this class:" << targetClass << " " << classLabels[targetClass] << endl;
        }

}
END_WORD


WORD(SampleHeight)
CODE(1245247)   // capslock + numlock + ?
virtual void execute() {
    
  if (currentBoundingBoxMode != STATIC_PRIOR) {
    if (currentBoundingBoxMode == MAPPING) {
      cout << "SampleHeight going to mappingHeightIdx: " << mappingHeightIdx << endl;
      currentThompsonHeight = convertHeightIdxToGlobalZ(mappingHeightIdx);
      currentThompsonHeightIdx = mappingHeightIdx;
      currentEEPose.pz = currentThompsonHeight;
      m_x = m_x_h[currentThompsonHeightIdx];
      m_y = m_y_h[currentThompsonHeightIdx];
      return;
    } else if (currentBoundingBoxMode == LEARNING_SAMPLING) {
      loadSampledHeightMemory();
    } else if (currentBoundingBoxMode == STATIC_MARGINALS) {
      loadMarginalHeightMemory();
    } else if (currentBoundingBoxMode == LEARNING_ALGORITHMC) {
      loadMarginalHeightMemory();
    } else {
      cout << "Invalid currentBoundingBoxMode. Asserting." << endl;
      assert(0);
    }
    drawHeightMemorySample();
    
    double best_height_prob = 0.0;
    int max_i = -1;
    for (int i = 0; i < hmWidth; i++) {
      
      int thisHeightMaxedOut = 0;
      
      if (currentBoundingBoxMode == LEARNING_SAMPLING) {
        thisHeightMaxedOut = ( (heightMemoryTries[i] >= bbLearningMaxTries) );
      } else if (currentBoundingBoxMode == LEARNING_ALGORITHMC) {
        double successes = heightMemoryPicks[i];
        double failures = heightMemoryTries[i] - heightMemoryPicks[i];
        // returns probability that mu <= d given successes and failures.
        double result = cephes_incbet(successes + 1, failures + 1, algorithmCTarget);
        thisHeightMaxedOut = (result > algorithmCRT);
      }
      
      if ( (heightMemorySample[i] > best_height_prob) &&
           (!thisHeightMaxedOut) ) {
        max_i = i;
        best_height_prob = heightMemorySample[i];
      }
    }
    currentThompsonHeight = convertHeightIdxToGlobalZ(max_i);
    currentThompsonHeightIdx = max_i;
    currentEEPose.pz = currentThompsonHeight;
    m_x = m_x_h[currentThompsonHeightIdx];
    m_y = m_y_h[currentThompsonHeightIdx];
  }
}
END_WORD
