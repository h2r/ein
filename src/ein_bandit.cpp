

WORD(SaveLearnedModels)
CODE(1245281)     // capslock + numlock + A
virtual void execute(std::shared_ptr<MachineState> ms)       {
  if (ms->config.focusedClass > -1) {
    // initialize this if we need to
    guardGraspMemory(ms);
    guardHeightMemory(ms);


    string thisLabelName = ms->config.focusedClassLabel;

    string dirToMakePath = data_directory + "/objects/" + thisLabelName + "/ir2D/";
    string this_range_path = dirToMakePath + "xyzRange.yml";

    Mat rangeMapTemp(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        rangeMapTemp.at<double>(y,x) = classRangeMaps[ms->config.focusedClass].at<double>(y,x);
      } 
    } 

    mkdir(dirToMakePath.c_str(), 0777);

    FileStorage fsvO;
    cout << "capslock + numlock + A: Writing: " << this_range_path << endl;
    fsvO.open(this_range_path, FileStorage::WRITE);
    fsvO << "rangeMap" << rangeMapTemp;

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

    copyGraspMemoryTriesToClassGraspMemoryTries(ms);
    fsvO << "graspMemoryTries1" << classGraspMemoryTries1[ms->config.focusedClass];
    fsvO << "graspMemoryPicks1" << classGraspMemoryPicks1[ms->config.focusedClass];
    fsvO << "graspMemoryTries2" << classGraspMemoryTries2[ms->config.focusedClass];
    fsvO << "graspMemoryPicks2" << classGraspMemoryPicks2[ms->config.focusedClass];
    fsvO << "graspMemoryTries3" << classGraspMemoryTries3[ms->config.focusedClass];
    fsvO << "graspMemoryPicks3" << classGraspMemoryPicks3[ms->config.focusedClass];
    fsvO << "graspMemoryTries4" << classGraspMemoryTries4[ms->config.focusedClass];
    fsvO << "graspMemoryPicks4" << classGraspMemoryPicks4[ms->config.focusedClass];


    copyHeightMemoryTriesToClassHeightMemoryTries(ms);
    fsvO << "heightMemoryTries" << classHeightMemoryTries[ms->config.focusedClass];
    fsvO << "heightMemoryPicks" << classHeightMemoryPicks[ms->config.focusedClass];


    lastRangeMap = rangeMapTemp;
    fsvO.release();
  } 
}
END_WORD
REGISTER_WORD(SaveLearnedModels)



WORD(SetRandomPositionAndOrientationForHeightLearning)
CODE( 1179687)     // capslock + numlock + '
virtual void execute(std::shared_ptr<MachineState> ms) {
  double noX = bbLearnPerturbScale * ((drand48() - 0.5) * 2.0);
  double noY = bbLearnPerturbScale * ((drand48() - 0.5) * 2.0);
  noX = noX + (((noX > 0) - 0.5) * 2) * bbLearnPerturbBias;
  noY = noY + (((noY > 0) - 0.5) * 2) * bbLearnPerturbBias;
  double noTheta = 3.1415926 * ((drand48() - 0.5) * 2.0);
  
  ms->config.currentEEPose.px += noX;
  ms->config.currentEEPose.py += noY;
  ms->config.currentEEDeltaRPY.pz += noTheta;
}
END_WORD
REGISTER_WORD(SetRandomPositionAndOrientationForHeightLearning)

WORD(BeginHeightLearning)
CODE(1245242)     // capslock + numlock + :
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->config.eepReg3 = ms->config.beeHome;
  ms->config.heightAttemptCounter = 0;
  ms->config.heightSuccessCounter = 0;
  thompsonPickHaltFlag = 0;
  thompsonHeightHaltFlag = 0;
  ms->pushWord("continueHeightLearning"); // continue height learning
  ms->pushWord(65568+3); // record register 3

  ms->pushWord(131139); // synchronic servo don't take closest
  ms->pushWord("synchronicServo"); // synchronic servo
  ms->pushWord(196707); // synchronic servo take closest
  ms->pushWord("visionCycle"); // vision cycle
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  { // prepare to servo
    //ms->config.currentEEPose.pz = wholeFoodsCounter1.pz+.1;
    ms->pushWord(1245248); // change to height 1
  }
  //ms->pushWord(1179723); // change height inference mode to LEARNING_SAMPLING
  ms->pushWord('3'); // recall register 3
}
END_WORD
REGISTER_WORD(BeginHeightLearning)


WORD(ContinueHeightLearning)
CODE(1179707)     // capslock + numlock + ;
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "continuing bounding box learning with currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
  ms->config.synServoLockFrames = 0;
  currentGradientServoIterations = 0;

  // ATTN 16
  // ATTN 19
  if (thompsonHardCutoff) {
    if (ms->config.heightAttemptCounter < ms->config.thompsonTries - 1) {
      // push this program 
      ms->pushWord("continueHeightLearning"); // begin bounding box learning
    } else {
      ms->pushCopies("beep", 15); // beep
    }
  }
  if (thompsonAdaptiveCutoff) {
    if ( (thompsonHeightHaltFlag) ||
         (ms->config.heightAttemptCounter >= ms->config.thompsonTries - 1) ) {
      cout << "Clearing call stack. thompsonHeightHaltFlag = " << thompsonHeightHaltFlag << 
        " and we did " << ms->config.heightAttemptCounter << " tries." << endl;
      ms->clearStack();
      ms->pushCopies("beep", 15); // beep
      return;
    } else {
      if (ms->config.heightAttemptCounter < ms->config.thompsonTries - 1) {
        // push this program 
        ms->pushWord("continueHeightLearning"); // begin bounding box learning
      } else {
        cout << "Clearing call stack. thompsonHeightHaltFlag = " << thompsonHeightHaltFlag << 
          " and we did " << ms->config.heightAttemptCounter << " tries." << endl;
        ms->clearStack();
      }
    }
  }

  ms->pushWord("recordHeightLearnTrial"); 

  ms->pushWord("synchronicServoDoNotTakeClosest"); 
  ms->pushWord("synchronicServo"); // synchronic servo
  ms->pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
  ms->pushWord("visionCycle"); // vision cycle
  //ms->pushWord(1179695); // check to see if bounding box is unique (early outting if not)
  ms->pushWord("visionCycle"); // vision cycle
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("setRandomPositionAndOrientationForHeightLearning"); // set random position for bblearn

  ms->pushWord(65568+4); // record register 4

  // servo to object, which will early out if it times out 
  ms->pushWord(131139); // synchronic servo don't take closest
  ms->pushWord("synchronicServo"); // synchronic servo
  ms->pushWord(196707); // synchronic servo take closest
  ms->pushWord("visionCycle"); // vision cycle
  //ms->pushWord(1179695); // check to see if bounding box is unique (early outting if not)
  ms->pushWord("visionCycle"); // vision cycle
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("setRandomPositionAndOrientationForHeightLearning"); // set random position for bblearn

  ms->pushWord("sampleHeight"); // sample height

  ms->pushWord("changeToPantryTable"); // change to pantry table
  ms->pushWord('3'); // recall register 3
}
END_WORD
REGISTER_WORD(ContinueHeightLearning)

WORD(RecordHeightLearnTrial)
CODE(1179694)     // capslock + numlock + .
  virtual void execute(std::shared_ptr<MachineState> ms)       {
  // Distances for the eraser
  //0.04, 2.57e-05, 0.0005, 0.0009, 0.007, 0.0006
  // ATTN 17
  double distance = squareDistanceEEPose(ms->config.currentEEPose, ms->config.eepReg4);
  cout << "cartesian distance from start: " << sqrt(distance) << endl;
  cout << "bbLearnThresh: " << bbLearnThresh << endl;
  if (distance < bbLearnThresh*bbLearnThresh) {
    Quaternionf q1(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    Quaternionf q2(ms->config.eepReg4.qw, ms->config.eepReg4.qx, ms->config.eepReg4.qy, ms->config.eepReg4.qz);
    double quaternionDistance = unsignedQuaternionDistance(q1, q2);
    cout << "quat distance from start: " << quaternionDistance << endl;
    cout << "bbQuatThresh: " << bbQuatThresh << endl;
    if (quaternionDistance < bbQuatThresh)
      recordBoundingBoxSuccess(ms);
    else
      recordBoundingBoxFailure(ms);
  } else {
    recordBoundingBoxFailure(ms);
  }
}
END_WORD
REGISTER_WORD(RecordHeightLearnTrial)




WORD(LoadSampledGraspMemory)
CODE(131117)     // capslock + -
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadSampledGraspMemory(ms);
  drawMapRegisters(ms);
}
END_WORD
REGISTER_WORD(LoadSampledGraspMemory)

WORD(LoadMarginalGraspMemory)
CODE(131133)     // capslock + =
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadMarginalGraspMemory(ms);
  drawMapRegisters(ms);
}
END_WORD
REGISTER_WORD(LoadMarginalGraspMemory)

WORD(LoadPriorGraspMemoryAnalytic)
CODE(196360)     // capslock + backspace
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadPriorGraspMemory(ms, ANALYTIC_PRIOR);
  copyGraspMemoryTriesToClassGraspMemoryTries(ms);
  loadMarginalGraspMemory(ms);
  
  // shows mus before we converted them to alphas and betas,
  // smoothing the values based on eccentricity.  
  //copyGraspMemoryRegister(ms->config.graspMemoryReg1, ms->config.graspMemorySample);
  
  drawMapRegisters(ms);
  cout << "class " << classLabels[ms->config.targetClass] << " number ";
}
END_WORD
REGISTER_WORD(LoadPriorGraspMemoryAnalytic)


WORD(LoadPriorGraspMemoryUniform)
CODE(261896)     // capslock + shift + backspace
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadPriorGraspMemory(ms, UNIFORM_PRIOR);
  copyGraspMemoryTriesToClassGraspMemoryTries(ms);
  loadMarginalGraspMemory(ms);
  
  // shows mus before we converted them to alphas and betas,
  // smoothing the values based on eccentricity.  
  //copyGraspMemoryRegister(ms->config.graspMemoryReg1, ms->config.graspMemorySample);
  
  drawMapRegisters(ms);
  cout << "class " << classLabels[ms->config.targetClass] << " number ";
}
END_WORD
REGISTER_WORD(LoadPriorGraspMemoryUniform)

WORD(LoadSampledHeightMemory)
CODE(1179693)     // capslock + numlock + -
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadSampledHeightMemory(ms);
  drawHeightMemorySample(ms);
}
END_WORD
REGISTER_WORD(LoadSampledHeightMemory)

WORD(LoadMarginalHeightMemory)
CODE(1179709)     // capslock + numlock + =
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadMarginalHeightMemory(ms);
  drawHeightMemorySample(ms);
}
END_WORD
REGISTER_WORD(LoadMarginalHeightMemory)

WORD(LoadPriorHeightMemoryAnalytic)
CODE(1244936)     // capslock + numlock + backspace
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadPriorHeightMemory(ms, ANALYTIC_PRIOR);
  copyHeightMemoryTriesToClassHeightMemoryTries(ms);
  loadMarginalHeightMemory(ms);
  drawHeightMemorySample(ms);
}
END_WORD
REGISTER_WORD(LoadPriorHeightMemoryAnalytic)

WORD(LoadPriorHeightMemoryUniform)
CODE(1310472)     // capslock + numlock + shift + backspace
virtual void execute(std::shared_ptr<MachineState> ms) {
  loadPriorHeightMemory(ms, UNIFORM_PRIOR);
  copyHeightMemoryTriesToClassHeightMemoryTries(ms);
  loadMarginalHeightMemory(ms);
  drawHeightMemorySample(ms);
}
END_WORD
REGISTER_WORD(LoadPriorHeightMemoryUniform)


WORD(SetPickModeToStaticPrior)
CODE(1179731)     // capslock + numlock + s
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentPickMode = STATIC_PRIOR;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD
REGISTER_WORD(SetPickModeToStaticPrior)

WORD(SetPickModeToLearningSampling)
CODE(1179716)     // capslock + numlock + d
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentPickMode = LEARNING_SAMPLING;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD
REGISTER_WORD(SetPickModeToLearningSampling)

WORD(SetPickModeToLearningAlgorithmC)
CODE(1245284) // capslock + numlock + D
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentPickMode = LEARNING_ALGORITHMC;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD
REGISTER_WORD(SetPickModeToLearningAlgorithmC)

WORD(SetPickModeToStaticMarginals)
CODE(1179718)     // capslock + numlock + f
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentPickMode = STATIC_MARGINALS;
  cout << "currentPickMode = " << pickModeToString(currentPickMode) << endl;
}
END_WORD
REGISTER_WORD(SetPickModeToStaticMarginals)


WORD(SetBoundingBoxModeToMapping)
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentBoundingBoxMode = MAPPING;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD
REGISTER_WORD(SetBoundingBoxModeToMapping)

WORD(SetBoundingBoxModeToStaticPrior)
CODE(1179722)     // capslock + numlock + j
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentBoundingBoxMode = STATIC_PRIOR;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD
REGISTER_WORD(SetBoundingBoxModeToStaticPrior)

WORD(SetBoundingBoxModeToLearningSampling)
CODE(1179723)     // capslock + numlock + k
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentBoundingBoxMode = LEARNING_SAMPLING;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD
REGISTER_WORD(SetBoundingBoxModeToLearningSampling)

WORD(SetBoundingBoxModeToLearningAlgorithmC)
CODE(1245291)     // capslock + numlock + K
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentBoundingBoxMode = LEARNING_ALGORITHMC;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD
REGISTER_WORD(SetBoundingBoxModeToLearningAlgorithmC)

WORD(SetBoundingBoxModeToStaticMarginals)
CODE(1179724)     // capslock + numlock + l
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentBoundingBoxMode = STATIC_MARGINALS;
  cout << "currentBoundingBoxMode  =  " << pickModeToString(currentBoundingBoxMode) << endl;
}
END_WORD
REGISTER_WORD(SetBoundingBoxModeToStaticMarginals)

WORD(UniformlySampleHeight)
CODE(1245246)      // capslock + numlock + >
virtual void execute(std::shared_ptr<MachineState> ms) {
  int thisRandThompsonHeight = lrand48() % ms->config.hmWidth;
  if (currentBoundingBoxMode == MAPPING) {
    thisRandThompsonHeight = mappingHeightIdx;
    cout << "UniformlySampleHeight going to mappingHeightIdx: " << mappingHeightIdx << endl;
  }
  currentThompsonHeight = convertHeightIdxToGlobalZ(ms, thisRandThompsonHeight);
  currentThompsonHeightIdx = thisRandThompsonHeight;
  ms->config.currentEEPose.pz = currentThompsonHeight;
  m_x = m_x_h[currentThompsonHeightIdx];
  m_y = m_y_h[currentThompsonHeightIdx];
}
END_WORD
REGISTER_WORD(UniformlySampleHeight)



WORD(SetGraspMemoriesFromClassGraspMemories)  
// capslock + numlock + i
CODE(1179721)
virtual void execute(std::shared_ptr<MachineState> ms) {
  copyClassGraspMemoryTriesToGraspMemoryTries(ms);
}
END_WORD
REGISTER_WORD(SetGraspMemoriesFromClassGraspMemories)  


WORD(SetHeightMemoriesFromClassHeightMemories)
// capslock + numlock + I 
CODE(1245289)
virtual void execute(std::shared_ptr<MachineState> ms)
{
        cout << "Loading height memories." << endl;
        if ((classHeightMemoryTries[ms->config.targetClass].rows > 1) && (classHeightMemoryPicks[ms->config.targetClass].cols == 1)) {
          cout << "targetClass: " << ms->config.targetClass << " " << classLabels[ms->config.targetClass] << endl;
          for (int i = 0; i < ms->config.hmWidth; i++) {
            ms->config.heightMemoryPicks[i] = classHeightMemoryPicks[ms->config.targetClass].at<double>(i, 0);
            ms->config.heightMemoryTries[i] = classHeightMemoryTries[ms->config.targetClass].at<double>(i, 0);
            cout << "picks: " << ms->config.heightMemoryPicks[i] << endl;
            cout << "tries: " << ms->config.heightMemoryTries[i] << endl;
          }
        } else {
	  cout << "Whoops, tried to set height memories but they don't exist for this class:" << ms->config.targetClass << " " << classLabels[ms->config.targetClass] << endl;
        }

}
END_WORD
REGISTER_WORD(SetHeightMemoriesFromClassHeightMemories)

WORD(SampleHeight)
CODE(1245247)   // capslock + numlock + ?
virtual void execute(std::shared_ptr<MachineState> ms) {
    
  if (currentBoundingBoxMode != STATIC_PRIOR) {
    if (currentBoundingBoxMode == MAPPING) {
      cout << "SampleHeight going to mappingHeightIdx: " << mappingHeightIdx << endl;
      currentThompsonHeight = convertHeightIdxToGlobalZ(ms, mappingHeightIdx);
      currentThompsonHeightIdx = mappingHeightIdx;
      ms->config.currentEEPose.pz = currentThompsonHeight;
      m_x = m_x_h[currentThompsonHeightIdx];
      m_y = m_y_h[currentThompsonHeightIdx];
      return;
    } else if (currentBoundingBoxMode == LEARNING_SAMPLING) {
      loadSampledHeightMemory(ms);
    } else if (currentBoundingBoxMode == STATIC_MARGINALS) {
      loadMarginalHeightMemory(ms);
    } else if (currentBoundingBoxMode == LEARNING_ALGORITHMC) {
      loadMarginalHeightMemory(ms);
    } else {
      cout << "Invalid currentBoundingBoxMode. Asserting." << endl;
      assert(0);
    }
    drawHeightMemorySample(ms);
    
    double best_height_prob = 0.0;
    int max_i = -1;
    for (int i = 0; i < ms->config.hmWidth; i++) {
      
      int thisHeightMaxedOut = 0;
      
      if (currentBoundingBoxMode == LEARNING_SAMPLING) {
        thisHeightMaxedOut = ( (ms->config.heightMemoryTries[i] >= bbLearningMaxTries) );
      } else if (currentBoundingBoxMode == LEARNING_ALGORITHMC) {
        double successes = ms->config.heightMemoryPicks[i];
        double failures = ms->config.heightMemoryTries[i] - ms->config.heightMemoryPicks[i];
        // returns probability that mu <= d given successes and failures.
        double result = cephes_incbet(successes + 1, failures + 1, algorithmCTarget);
        thisHeightMaxedOut = (result > algorithmCRT);
      }
      
      if ( (ms->config.heightMemorySample[i] > best_height_prob) &&
           (!thisHeightMaxedOut) ) {
        max_i = i;
        best_height_prob = ms->config.heightMemorySample[i];
      }
    }
    currentThompsonHeight = convertHeightIdxToGlobalZ(ms, max_i);
    currentThompsonHeightIdx = max_i;
    ms->config.currentEEPose.pz = currentThompsonHeight;
    m_x = m_x_h[currentThompsonHeightIdx];
    m_y = m_y_h[currentThompsonHeightIdx];
  } else {
    cout << "SampleHeight going to mappingHeightIdx: " << mappingHeightIdx << endl;
    currentThompsonHeight = convertHeightIdxToGlobalZ(ms, mappingHeightIdx);
    currentThompsonHeightIdx = mappingHeightIdx;
    ms->config.currentEEPose.pz = currentThompsonHeight;
    m_x = m_x_h[currentThompsonHeightIdx];
    m_y = m_y_h[currentThompsonHeightIdx];
  }
}
END_WORD
REGISTER_WORD(SampleHeight)
