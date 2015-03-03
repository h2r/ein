WORD(VisionPatrol)
CODE(196727) // capslock + W
virtual void execute() {
  cout << "vision patrol" << endl;
  pushWord("visionPatrol");
  pushWord("setRandomPositionAndOrientationForHeightLearning");
  pushWord("recordBlueBoxes");
  pushWord("visionCycle");
}
END_WORD

WORD(RecordBlueBoxes)
virtual void execute() {
  cout << "Recording blue boxes: " << bTops.size() << endl;
  for (int c = 0; c < bTops.size(); c++) {
    BoxMemory box;
    box.bTop = bTops[c];
    box.bBot = bBots[c];
    box.cameraPose = currentEEPose;
    box.cameraTime = ros::Time::now();
    box.labeledClassIndex = bLabels[c];
    blueBoxMemories.push_back(box);
  }

}
END_WORD

WORD(VisionCycle)
CODE(131153)  // capslock + q
virtual void execute() {
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
