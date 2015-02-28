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
  currentThompsonHeight = convertHeightIdxToGlobalZ(thisRandThompsonHeight);
  currentThompsonHeightIdx = thisRandThompsonHeight;
  currentEEPose.pz = currentThompsonHeight;
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
    if (currentBoundingBoxMode == LEARNING_SAMPLING) {
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
  }
}
END_WORD
