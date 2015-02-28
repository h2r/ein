WORD(LoadMarginalHeightMemory)
CODE(1179709)  // capslock + numlock + =
virtual void execute()
{
  loadMarginalHeightMemory();
  drawHeightMemorySample();
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
